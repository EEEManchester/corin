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

    // compute CoM location Re^3

    // compute Composite Rigid Body Inertia Re^3x3
    
    // formulate QP problem
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