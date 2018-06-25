#include "force_distribution.h"

namespace force_distribution
{
 
ForceDistribution::ForceDistribution() : invdyn(inertias, xM),
  nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~"))   // initialize node
  {
    //***************** INITIALIZE VARIABLES ***************//
    
    // 6x1 initialization
    qbpr = qbvr = qbar = Eigen::MatrixXd::Zero(6,1);
    gait_phase = Eigen::MatrixXi::Zero(6,1);

    //***************** NODE HANDLES ***************//
    sp_subscriber_  = nh_.subscribe("/corin/motion_plan", 1, &ForceDistribution::setpoint_callback, this);
   
    // ----- Force Output ----- //
    // cf_pub_ = nh_.advertise<std_msgs::Float64>("/corin/lf_q1_controller/command", 1); 

  }
  
  void ForceDistribution::setpoint_callback(const corin_control::MotionPlan& data)
  {
      // check for array size - Re^24 (6 for floating base, 18 for joints)
    if ( (data.setpoint.positions.size() == 24) && 
          (data.setpoint.velocities.size() == 24) &&
          (data.setpoint.accelerations.size() == 24) &&
          (data.gait_phase.size() == 6)  )
    {
      // Base State
      for (int i=0; i < 6; i++){
        qbpr(i) = data.setpoint.positions[i];
        qbvr(i) = data.setpoint.velocities[i];
        qbar(i) = data.setpoint.accelerations[i];
        gait_phase(i) = data.gait_phase[i];
      }

      // Leg joint angles
      for (int i=6; i < 24; i++){    
        qpr(i-6) = data.setpoint.positions[i];         // rcg: reference joint space position
        qvr(i-6) = data.setpoint.velocities[i];        // rcg: reference joint space velocity
        qar(i-6) = data.setpoint.accelerations[i];     // joint space acceleration
      }

      qp_solution();  
    }
    else{
      std::cout << "Size mismatch, pass... " << std::endl;
    }
  }
    
  void ForceDistribution::qp_solution()
  {
    std::cout << "Solving force distribution...." << std::endl;

    using namespace Eigen;
    using namespace std;
    
    // no. contact points
    int n_contact = 6 - gait_phase.sum();
    // cout << n_contact << " " << gait_phase.sum() << endl;

    // gravity vector, Re^3
    Matrix3d world_X_base;
    Vector3d gv(0,0,-9.81), g;
    world_X_base = AngleAxisd(qbpr(5), Vector3d::UnitZ())
                  *AngleAxisd(qbpr(4), Vector3d::UnitY())
                  *AngleAxisd(qbpr(3), Vector3d::UnitX());
    
    g = world_X_base.transpose() * gv;
    // cout << world_X_base << endl;
    // cout << gv << endl;
    // cout << g  << endl;

    // compute CoM location Re^3
    auto x_com = getWholeBodyCOM(inertias, qpr, xH);
    // cout << x_com << endl;

    // compute Composite Rigid Body Inertia Re^3x3
    auto Ig = CRBI();
    // cout << "================" <<endl;
    // cout << Ig.block<3,3>(0,0) << endl;

    // CoM to foot position, in world frame
    Vector3d fr_world_v3_com_X_foot[6];
    fr_world_v3_com_X_foot[0] = world_X_base*(-x_com + xH.fr_trunk_X_LF_foot(qpr).block<3,1>(0,3));
    fr_world_v3_com_X_foot[1] = world_X_base*(-x_com + xH.fr_trunk_X_LM_foot(qpr).block<3,1>(0,3));
    fr_world_v3_com_X_foot[2] = world_X_base*(-x_com + xH.fr_trunk_X_LR_foot(qpr).block<3,1>(0,3));
    fr_world_v3_com_X_foot[3] = world_X_base*(-x_com + xH.fr_trunk_X_RF_foot(qpr).block<3,1>(0,3));
    fr_world_v3_com_X_foot[4] = world_X_base*(-x_com + xH.fr_trunk_X_RM_foot(qpr).block<3,1>(0,3));
    fr_world_v3_com_X_foot[5] = world_X_base*(-x_com + xH.fr_trunk_X_RR_foot(qpr).block<3,1>(0,3));

    /* formulate QP problem, Af = B */
    // constructing B
    VectorXd B(6);
    B.block<3,1>(0,0) = inertias.getTotalMass()*(qbpr.block<3,1>(0,0) + g);
    B.block<3,1>(3,0) = Ig.block<3,3>(0,0)*qbpr.block<3,1>(3,0);
    // cout << B << endl;

    // constructing A
    int pc = 0;
    MatrixXd A(6,3*n_contact);
    // cycles through all six legs
    for (int j=0; j<6; j++){
      // amend matrix if legs are in support phase
      if (gait_phase(j) == 0){
        A.block<3,3>(0,pc*3) = MatrixXd::Identity(3, 3);
        A.block<3,3>(3,pc*3) = skew_this_vector(fr_world_v3_com_X_foot[j]);
        pc++;
        // cout << "Amending " << gait_phase(i) << " " << i << " " << pc << endl;
      }
    }
    // cout << A << endl;

    // friction constraint
    MatrixXd C(6,3), Cineq(6*n_contact, 3*n_contact);
    MatrixXd D(6,1), Dineq(6*n_contact, 1);

    Cineq.setZero(6*n_contact, 3*n_contact);
    Dineq.setZero(6*n_contact, 1);

    double mu = 0.5;  // friction coefficient
    double fmin = 1;  // min. force
    double fmax = 20; // max. force

    C <<   1, 0, -mu,
          -1, 0, -mu,
           0, 1, -mu,
           0,-1, -mu,
           0, 0, -1,
           0, 0, 1;

    D << 0,0,0,0, fmin, fmax;

    for (int i=0; i<n_contact; i++){
      Cineq.block<6,3>(i*6,i*3) = C;
      Dineq.block<6,1>(i*6,0) = D;
    }
    // cout << "=========================" << endl;
    // cout << Dineq << endl;

    // weightage matrix
    VectorXd sV(6);
    sV << 1, 1, 10, 10, 10, 10;
    MatrixXd wS = sV.asDiagonal();
    
    // construct equation
    MatrixXd Hs = 2*A.transpose()*wS*A;
    MatrixXd Qs = (-2*B.transpose()*wS*A).transpose();

    USING_NAMESPACE_QPOASES
    
    real_t qpH[Hs.size()];   // Hessian Matrix, Re^(3cx3c)
    real_t qpG[Qs.size()];   // Gradient Vector, Re^(3c)
    real_t qpA[n_contact*6]; // Constraint Matrix, 
    // real_t qplbA[];
    // real_t qpubA[];
    // real_t qpLB[2] = { 0.5, -2.0 };
    // real_t qpUB[2] = { 5.0, 2.0 };
    
    // remap from eigen to std array form
    Map<RowVectorXd> v1(Hs.data(), Hs.size());  // first, flatten to 1D row vector
    Map<RowVectorXd>(&qpH[0], Hs.size()) = v1;  // second, convert to array - pointer
    
    Map<RowVectorXd> v2(Qs.data(), Qs.size());
    Map<RowVectorXd>(&qpG[0], Qs.size()) = v2;

    /* Setting up QProblem object. */
    QProblem qformula( n_contact*3, n_contact*6 );

    Options options;
    qformula.setOptions( options );

    /* Solve first QP. */
    int nWSR = 10;
    // qformula.init( qpH, qpG, qpA, qpLB, qpUB, qplbA, qpubA, nWSR );
     // Get and print solution of first QP. 
    // real_t xOpt[n_contact*3];
    // qformula.getPrimalSolution( xOpt );
    // printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1], qformula.getObjVal() );
  }
  
  Eigen::Matrix3d ForceDistribution::skew_this_vector(Eigen::Vector3d data)
  {
   Eigen::Matrix3d skew_m(3,3);
   skew_m(0,1) = -data(2);
   skew_m(0,2) =  data(1);
   skew_m(1,0) =  data(2);
   skew_m(1,2) = -data(0);
   skew_m(2,0) = -data(2);
   skew_m(2,1) =  data(0);

   return skew_m; 
  }

  Eigen::MatrixXd ForceDistribution::CRBI()
  {
    auto inertia_LF_hip = xM.fr_LF_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_LF_hipassembly()*xM.fr_LF_hipassembly_X_fr_trunk(qpr);
    auto inertia_LF_upp = xM.fr_LF_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LF_upperleg()*xM.fr_LF_upperleg_X_fr_trunk(qpr);
    auto inertia_LF_low = xM.fr_LF_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LF_lowerleg()*xM.fr_LF_lowerleg_X_fr_trunk(qpr);

    auto inertia_LM_hip = xM.fr_LM_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_LM_hipassembly()*xM.fr_LM_hipassembly_X_fr_trunk(qpr);
    auto inertia_LM_upp = xM.fr_LM_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LM_upperleg()*xM.fr_LM_upperleg_X_fr_trunk(qpr);
    auto inertia_LM_low = xM.fr_LM_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LM_lowerleg()*xM.fr_LM_lowerleg_X_fr_trunk(qpr);

    auto inertia_LR_hip = xM.fr_LR_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_LR_hipassembly()*xM.fr_LR_hipassembly_X_fr_trunk(qpr);
    auto inertia_LR_upp = xM.fr_LR_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LR_upperleg()*xM.fr_LR_upperleg_X_fr_trunk(qpr);
    auto inertia_LR_low = xM.fr_LR_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LR_lowerleg()*xM.fr_LR_lowerleg_X_fr_trunk(qpr);

    auto inertia_RF_hip = xM.fr_RF_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_RF_hipassembly()*xM.fr_RF_hipassembly_X_fr_trunk(qpr);
    auto inertia_RF_upp = xM.fr_RF_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RF_upperleg()*xM.fr_RF_upperleg_X_fr_trunk(qpr);
    auto inertia_RF_low = xM.fr_RF_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RF_lowerleg()*xM.fr_RF_lowerleg_X_fr_trunk(qpr);

    auto inertia_RM_hip = xM.fr_RM_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_RM_hipassembly()*xM.fr_RM_hipassembly_X_fr_trunk(qpr);
    auto inertia_RM_upp = xM.fr_RM_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RM_upperleg()*xM.fr_RM_upperleg_X_fr_trunk(qpr);
    auto inertia_RM_low = xM.fr_RM_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RM_lowerleg()*xM.fr_RM_lowerleg_X_fr_trunk(qpr);

    auto inertia_RR_hip = xM.fr_RR_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_RR_hipassembly()*xM.fr_RR_hipassembly_X_fr_trunk(qpr);
    auto inertia_RR_upp = xM.fr_RR_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RR_upperleg()*xM.fr_RR_upperleg_X_fr_trunk(qpr);
    auto inertia_RR_low = xM.fr_RR_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RR_lowerleg()*xM.fr_RR_lowerleg_X_fr_trunk(qpr);

    auto composite_inertia = inertias.getTensor_trunk() +
                              inertia_LF_hip + inertia_LF_upp + inertia_LF_low +
                              inertia_LM_hip + inertia_LM_upp + inertia_LM_low +
                              inertia_LR_hip + inertia_LR_upp + inertia_LR_low +
                              inertia_RF_hip + inertia_RF_upp + inertia_RF_low +
                              inertia_RM_hip + inertia_RM_upp + inertia_RM_low +
                              inertia_RR_hip + inertia_RR_upp + inertia_RR_low;
    return composite_inertia;
  }

  void ForceDistribution::publish_cmd(const Eigen::MatrixXd& data)
  {
    // -----------   Publish cmd  -----------  //
    std_msgs::Float64 float_cmd;

    // float_cmd.data = data(0);   lf_q1_pub.publish(float_cmd);
    // float_cmd.data = data(1);   lf_q2_pub.publish(float_cmd);
    // float_cmd.data = data(2);   lf_q3_pub.publish(float_cmd);

    // float_cmd.data = data(3);   lm_q1_pub.publish(float_cmd);
    // float_cmd.data = data(4);   lm_q2_pub.publish(float_cmd);
    // float_cmd.data = data(5);   lm_q3_pub.publish(float_cmd);

    // float_cmd.data = data(6);   lr_q1_pub.publish(float_cmd);
    // float_cmd.data = data(7);   lr_q2_pub.publish(float_cmd);
    // float_cmd.data = data(8);   lr_q3_pub.publish(float_cmd);

    // float_cmd.data = data(9);    rf_q1_pub.publish(float_cmd);
    // float_cmd.data = data(10);   rf_q2_pub.publish(float_cmd);
    // float_cmd.data = data(11);   rf_q3_pub.publish(float_cmd);

    // float_cmd.data = data(12);   rm_q1_pub.publish(float_cmd);
    // float_cmd.data = data(13);   rm_q2_pub.publish(float_cmd);
    // float_cmd.data = data(14);   rm_q3_pub.publish(float_cmd);

    // float_cmd.data = data(15);   rr_q1_pub.publish(float_cmd);
    // float_cmd.data = data(16);   rr_q2_pub.publish(float_cmd);
    // float_cmd.data = data(17);   rr_q3_pub.publish(float_cmd);
  }
} // namespace force_distribution