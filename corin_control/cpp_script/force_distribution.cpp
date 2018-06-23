#include "force_distribution.h"

namespace force_distribution
{
 
ForceDistribution::ForceDistribution() : invdyn(inertias, xM),
  nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~"))   // initialize node
  {
    //***************** INITIALIZE VARIABLES ***************//
    controller_type = "position";

    active_legs = 1;
    size_n  = 3;

    ts = 0.0125;        // control loop sampling time
    cN = 15;            // PID filter coefficient
    
    Kxx_ = Kxy_ = Kxz_ = 5.0;
    Kdx_ = Kdy_ = Kdz_ = 0.001;

    // 6x1 initialization
    qbpr = qbvr = qbar = Eigen::MatrixXd::Zero(6,1);

    // nxn initialization
    qzd = Kid = Kix = Eigen::MatrixXd::Zero(size_n,size_n);
    PID_Kp = PID_Ki = PID_Kd = Eigen::MatrixXd::Zero(size_n,size_n);

    // nx1 initialization
    f_state = sum_error = Eigen::MatrixXd::Zero(size_n,1);
    tau_ct = tau_cr = contact_force = Eigen::MatrixXd::Zero(size_n,1);

    // 6x1
    con_f = Eigen::MatrixXd::Zero(6,1);

    // Set to default values
    for (int i=0; i < size_n; i++){
      qzd(i,i) = 0.1;                 // joint damping
    }

    q_temp = Eigen::MatrixXd::Zero(18,1);
    q_temp(0) = 1.5;
    // std::cout << q_temp << std::endl;

    //***************** NODE HANDLES ***************//
    sp_subscriber_  = nh_.subscribe("/corin/setpoint_trajectory", 1, &ForceDistribution::setpoint_callback, this);
   
    // ----- Force Output ----- //
    cf_pub_ = nh_.advertise<std_msgs::Float64>("/corin/lf_q1_controller/command", 1); 

    
  }
  
  void ForceDistribution::setpoint_callback(const trajectory_msgs::JointTrajectoryPoint& data)
  {
    // check for array size - Re^24 (6 for floating base, 18 for joints)
    if ( (data.positions.size() == 24) && 
          (data.velocities.size() == 24) &&
          (data.accelerations.size() == 24)   ){
      // Base State
      for (int i=0; i < 6; i++){
        qbpr(i) = data.positions[i];
        qbvr(i) = data.velocities[i];
        qbar(i) = data.accelerations[i];
      }

      // Leg joint angles
      for (int i=6; i < 24; i++){    
        qpr(i-6) = data.positions[i];         // rcg: reference joint space position
        qvr(i-6) = data.velocities[i];        // rcg: reference joint space velocity
        qar(i-6) = data.accelerations[i];     // joint space acceleration
      }

      qp_solution();  
    }
    else{
      std::cout << "Size mismatch, pass... " << std::endl;
    }
    
  }

  void ForceDistribution::qp_solution()
  {
    std::cout << "Solving force distribution...." <<std::endl;

    USING_NAMESPACE_QPOASES
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
    real_t A[1*2] = { 1.0, 1.0 };
    real_t g[2] = { 1.5, 1.0 };
    real_t lb[2] = { 0.5, -2.0 };
    real_t ub[2] = { 5.0, 2.0 };
    real_t lbA[1] = { -1.0 };
    real_t ubA[1] = { 2.0 };

    /* Setting up QProblem object. */
    QProblem example( 2,1 );

    Options options;
    example.setOptions( options );

    /* Solve first QP. */
    int nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

     // Get and print solution of first QP. 
    real_t xOpt[2];
    real_t yOpt[2+1];
    example.getPrimalSolution( xOpt );
    example.getDualSolution( yOpt );
    printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
        xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
  }
  // void ForceDistribution::passivity_controller()
  // {
  //   // -----------  Update gain parameters  ----------- //
  //   nh_.getParam("Leg_impedance/Kxx", Kix(0,0));    nh_.getParam("Leg_impedance/Kxy", Kix(1,1));   nh_.getParam("Leg_impedance/Kxz", Kix(2,2));
  //   nh_.getParam("Leg_impedance/Kdx", Kid(0,0));    nh_.getParam("Leg_impedance/Kdy", Kid(1,1));   nh_.getParam("Leg_impedance/Kdz", Kid(2,2));

  //   nh_.getParam("Leg_fb_q1/PID", PIDq1_);      nh_.getParam("Leg_fb_q2/PID", PIDq2_);      nh_.getParam("Leg_fb_q3/PID", PIDq3_);

  //   PID_Kp(0,0) = PIDq1_[0];    PID_Ki(0,0) = PIDq1_[1];    PID_Kd(0,0) = PIDq1_[2];
  //   PID_Kp(1,1) = PIDq2_[0];    PID_Ki(1,1) = PIDq2_[1];    PID_Kd(1,1) = PIDq2_[2];
  //   PID_Kp(2,2) = PIDq3_[0];    PID_Ki(2,2) = PIDq3_[1];    PID_Kd(2,2) = PIDq3_[2];

  //   // -----------  Initialize variables  ----------- //
  //   // Define variables
  //   Eigen::Matrix<double, 3, 1> qp_e, qv_e, Kd_error;           // Joint position, velocity error; PIDy cumulation
  //   Eigen::Matrix<double, 3, 1> tau_fb, tau_fr, tau_t;          // PID feedback tau; friction compensation tau; total tau

  //   /*
  //   for (int i=0; i < size_n; i++){
  //     // -----------   Friction Compensation   -----------  //
  //     tau_fr[i] = zd[i]*qvr[i];        // Gazebo - damping only

  //     if (qvr[i] < 0.0015 and qvr[i] > 0.0)
  //       tau_cou = 0.0;
  //     else
  //       if (qvr[i] < 0.0)                 // -ve
  //         tau_cou = 0.1212*(-1.0);
  //       else
  //         tau_cou = 0.1212;
  //     tau_fr[i] = 0.00353*qar(i) + zd[i]*qvr[i] + tau_cou;        // motor inertia + viscous + coulomb                    
  //   }*/

  //   // -----------        Error       -----------//
  //   qp_e = qpr - q;                                               // eigen: position error
  //   qv_e = qvr - qd;                                              // eigen: velocity error 

  //   // -----------   Joint PID Feedback  -----------  //
  //   Kd_error  = (PID_Kd*qp_e - f_state)*cN;                       // derivative
  //   sum_error = (sum_error + qp_e)*ts;                            // integral

  //   tau_fb    = PID_Kp*qp_e + PID_Ki*sum_error + Kd_error;        // PID total

  //   f_state  = f_state + ts*Kd_error;                             // filtered derivative

  //   // -----------   Friction Compensation   -----------  //
  //   tau_fr = qzd*qvr;

  //   // std::cout << "Fric: \t" << tau_fr.transpose() <<std::endl;
  //   // std::cout << "Eqvr: \t" << qvr.transpose() << std::endl;
  //   // std::cout << "qzd : \n" << qzd << std::endl;

  //   // ************************************************//
  //   invdyn.id(tau, q, qd, qar);
  //   invdyn.G_terms(tau_gc);

  //   // -----------   Impedance Controller  -----------  //
  //   Eigen::Matrix<double, 3, 1> t2, imp_e, imp_ed, tau_imp;
  //   Eigen::MatrixXd F6ext = Eigen::MatrixXd::Zero(6,1);

  //   imp_e  = (xH.fr_base_X_ee(q).block<3,1>(0,3)) - (t2 = xH.fr_base_X_ee(qpr).block<3,1>(0,3));   // x - x_ref
  //   imp_ed = (jacobians.fr_base_J_ee(q)*qd - jacobians.fr_base_J_ee(qpr)*qvr).block<3,1>(3,0);     // xd - xd_ref
    
  //   F6ext.block<3,1>(3,0) = Kix*imp_e + Kid*imp_ed;
  //   tau_imp = jacobians.fr_base_J_ee(q).transpose()*F6ext;
    
  //   // std::cout << imp_e.transpose() << "\t" << imp_ed.transpose() <<std::endl;
  //   // std::cout << F6ext.transpose() << std::endl;
  //   // std::cout << tau_imp.transpose() << std::endl;
  //   std::cout << "--------------------------------" << std::endl;

  //   // -----------   Ground Contact Force  -----------  //  
  //   tau_ct = jacobians.fr_base_J_ee(q).transpose()*con_f;       // tau = J.'F   J = [Jw Jv].'
  //   auto est_force = ( -(jacobians.fr_base_J_ee(q).block<3,3>(3,0).transpose()).inverse() )*tau_gc;

  //   // std::cout << "est f: " << est_force.transpose() << std::endl;
    
  //   // -----------   Total Commanded Torque  -----------  //
  //   tau_t = tau + tau_fb + tau_fr - tau_ct;
  //   std::cout << tau_cr.transpose() << std::endl;
  //   std::cout << tau_gc.transpose() << std::endl;
  //   // std::cout << tau_fb.transpose() << std::endl;
  //   // std::cout << tau_fr.transpose() << std::endl;
  //   // std::cout << tau_ct.transpose() << std::endl;

  //   // -----------   Publish cmd  -----------  //
  //   publish_cmd(tau_t);
  // }

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