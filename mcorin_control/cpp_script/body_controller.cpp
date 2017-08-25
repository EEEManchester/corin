#include "body_controller.h"
 
namespace body_controller
{
 
bodyController::bodyController() : invdyn(inertias, xM),
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
    std::cout << q_temp << std::endl;
    // dconfig = boost::bind(&dconfig_callback, _1, _2);
    // server.setCallback(dconfig);

    //***************** RETREIVE PARAMS ***************//
    // nh_private_.param<double>("Leg_impedance/Kdx", Kix(0,0), Kxx_);    nh_private_.param<double>("Leg_impedance/Kdx", Kid(0,0), Kdx_);
    // nh_private_.param<double>("Leg_impedance/Kdx", Kix(1,1), Kxy_);    nh_private_.param<double>("Leg_impedance/Kdy", Kid(1,1), Kdy_);
    // nh_private_.param<double>("Leg_impedance/Kdx", Kix(2,2), Kxz_);    nh_private_.param<double>("Leg_impedance/Kdz", Kid(2,2), Kdz_);  

    // nh_private_.param("Leg_fb_q1/PID", PIDq1_, {0.15, 0.0, 0.01});
    // nh_private_.param("Leg_fb_q2/PID", PIDq2_, {0.50, 0.0, 0.01});
    // nh_private_.param("Leg_fb_q3/PID", PIDq3_, {0.30, 0.0, 0.01});
   
    //***************** NODE HANDLES ***************//
    sp_subscriber_  = nh_.subscribe("/mcorin/setpoint", 1, &bodyController::setpoint_callback, this);
   
    // // ----- Gazebo ----- // //
    gz_fforce_sub_  = nh_.subscribe("/corin/leg_1_foot_force", 1, &bodyController::foot_contact_callback, this);
    joint_sub_      = nh_.subscribe("/leg_corin/joint_states", 1, &bodyController::gz_effort_callback, this);

    lf_q1_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lf_q1_controller/command", 1); 
    lf_q2_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lf_q2_controller/command", 1); 
    lf_q3_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lf_q3_controller/command", 1);

    lm_q1_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lm_q1_controller/command", 1); 
    lm_q2_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lm_q2_controller/command", 1);
    lm_q3_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lm_q3_controller/command", 1);

    lr_q1_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lr_q1_controller/command", 1); 
    lr_q2_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lr_q2_controller/command", 1); 
    lr_q3_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/lr_q3_controller/command", 1);

    rf_q1_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rf_q1_controller/command", 1); 
    rf_q2_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rf_q2_controller/command", 1); 
    rf_q3_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rf_q3_controller/command", 1);

    rm_q1_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rm_q1_controller/command", 1); 
    rm_q2_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rm_q2_controller/command", 1); 
    rm_q3_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rm_q3_controller/command", 1);

    rr_q1_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rr_q1_controller/command", 1); 
    rr_q2_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rr_q2_controller/command", 1); 
    rr_q3_pub   = nh_.advertise<std_msgs::Float64>("/mcorin/rr_q3_controller/command", 1);

    // // ----- DXL ----- // //
    // ros::Subscriber sub_joint = n.subscribe("/robot_state/dynamixel_port", 1, dxl_state_callback);
    // lf_q1_pub = n.advertise<std_msgs::Float64>("/leg_1_q1_controller/torque_command", 1);    
    // lf_q2_pub = n.advertise<std_msgs::Float64>("/leg_1_q2_controller/torque_command", 1);

  }
  
  void bodyController::dconfig_callback(mcorin_control::dynreconfigConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Requested");
    std::cout << "dynamic reconfigure" <<std::endl;
    dyn_data = config.leg_impedance_Kxx;
  }

  void bodyController::gz_effort_callback(const sensor_msgs::JointState& msg)
  {
    unsigned char nj;
    nj = size_n;        // number of joints
    
    for (int i=0; i < size_n; i++){
      q(i)   = msg.position[i];       // rcg: current joint space position
      qd(i)  = msg.velocity[i];       // rcg: current joint space velocity

      tau_cr(i,0) = msg.effort[i];    // eigen: current joint space effort
    }
    // force_torque_estimation();
    if (not controller_type.compare("effort"))
      bodyController::passivity_controller(); // leg controller
  }
  void bodyController::dxl_state_callback(const dynamixel_msgs::DxlJointState& msg)
  {
    for (int i=0; i < size_n; i++){
      q(i)  = msg.position[i];
      qd(i) = msg.velocity[i];
    }
    //std::cout << q << std::endl;
    // passivity_controller(nj);
  }

  void bodyController::setpoint_callback(const trajectory_msgs::JointTrajectoryPoint& data)
  {
    std::cout << "in setpoint " <<std::endl;
    // TODO: Deal with empty sizes
    for (int i=0; i < 18; i++){    
      q_temp(i) = data.positions[i];
      // qpr(i) = data.positions[i];         // rcg: reference joint space position
      // qvr(i) = data.velocities[i];        // rcg: reference joint space velocity
      // qar(i) = data.accelerations[i];     // joint space acceleration
      // con_f(3+i,0) = data.effort[i];      // task space effort, piggybag
    }
    if (not controller_type.compare("position"))
      bodyController::position_controller();
  }

  void bodyController::foot_contact_callback(const gazebo_msgs::ContactsState& data)
  {
    if (not data.states.empty()){
      contact_force(0,0) = data.states[0].total_wrench.force.x;
      contact_force(1,0) = data.states[0].total_wrench.force.y;
      contact_force(2,0) = data.states[0].total_wrench.force.z;

      // Transform to SCS frame
      contact_force = xH.fr_base_X_ee(q).block<3,3>(0,0)*contact_force;
    }
    else{
      for (int i=0; i < 3; i++)       // force vector
        contact_force(i,0) = 0.0;
    }
  
    /* Estimate **********************************************/
    // gravity compensation
    auto est_force = ( (jacobians.fr_base_J_ee(q).block<3,3>(3,0).transpose()).inverse() )*(tau_gc-tau_cr);

    // std::cout << "sensor: " << contact_force.transpose() << std::endl;      // reported in body frame
    // std::cout << "estima: " << est_force.transpose() << std::endl;            // reported in body frame
  }

  void bodyController::passivity_controller()
  {
    // -----------  Update gain parameters  ----------- //
    nh_.getParam("Leg_impedance/Kxx", Kix(0,0));    nh_.getParam("Leg_impedance/Kxy", Kix(1,1));   nh_.getParam("Leg_impedance/Kxz", Kix(2,2));
    nh_.getParam("Leg_impedance/Kdx", Kid(0,0));    nh_.getParam("Leg_impedance/Kdy", Kid(1,1));   nh_.getParam("Leg_impedance/Kdz", Kid(2,2));

    nh_.getParam("Leg_fb_q1/PID", PIDq1_);      nh_.getParam("Leg_fb_q2/PID", PIDq2_);      nh_.getParam("Leg_fb_q3/PID", PIDq3_);

    PID_Kp(0,0) = PIDq1_[0];    PID_Ki(0,0) = PIDq1_[1];    PID_Kd(0,0) = PIDq1_[2];
    PID_Kp(1,1) = PIDq2_[0];    PID_Ki(1,1) = PIDq2_[1];    PID_Kd(1,1) = PIDq2_[2];
    PID_Kp(2,2) = PIDq3_[0];    PID_Ki(2,2) = PIDq3_[1];    PID_Kd(2,2) = PIDq3_[2];

    // -----------  Initialize variables  ----------- //
    // Define variables
    Eigen::Matrix<double, 3, 1> qp_e, qv_e, Kd_error;           // Joint position, velocity error; PIDy cumulation
    Eigen::Matrix<double, 3, 1> tau_fb, tau_fr, tau_t;          // PID feedback tau; friction compensation tau; total tau

    /*
    for (int i=0; i < size_n; i++){
      // -----------   Friction Compensation   -----------  //
      tau_fr[i] = zd[i]*qvr[i];        // Gazebo - damping only

      if (qvr[i] < 0.0015 and qvr[i] > 0.0)
        tau_cou = 0.0;
      else
        if (qvr[i] < 0.0)                 // -ve
          tau_cou = 0.1212*(-1.0);
        else
          tau_cou = 0.1212;
      tau_fr[i] = 0.00353*qar(i) + zd[i]*qvr[i] + tau_cou;        // motor inertia + viscous + coulomb                    
    }*/

    // -----------        Error       -----------//
    qp_e = qpr - q;                                               // eigen: position error
    qv_e = qvr - qd;                                              // eigen: velocity error 

    // -----------   Joint PID Feedback  -----------  //
    Kd_error  = (PID_Kd*qp_e - f_state)*cN;                       // derivative
    sum_error = (sum_error + qp_e)*ts;                            // integral

    tau_fb    = PID_Kp*qp_e + PID_Ki*sum_error + Kd_error;        // PID total

    f_state  = f_state + ts*Kd_error;                             // filtered derivative

    // -----------   Friction Compensation   -----------  //
    tau_fr = qzd*qvr;

    // std::cout << "Fric: \t" << tau_fr.transpose() <<std::endl;
    // std::cout << "Eqvr: \t" << qvr.transpose() << std::endl;
    // std::cout << "qzd : \n" << qzd << std::endl;

    // ************************************************//
    invdyn.id(tau, q, qd, qar);
    invdyn.G_terms(tau_gc);

    // -----------   Impedance Controller  -----------  //
    Eigen::Matrix<double, 3, 1> t2, imp_e, imp_ed, tau_imp;
    Eigen::MatrixXd F6ext = Eigen::MatrixXd::Zero(6,1);

    imp_e  = (xH.fr_base_X_ee(q).block<3,1>(0,3)) - (t2 = xH.fr_base_X_ee(qpr).block<3,1>(0,3));   // x - x_ref
    imp_ed = (jacobians.fr_base_J_ee(q)*qd - jacobians.fr_base_J_ee(qpr)*qvr).block<3,1>(3,0);     // xd - xd_ref
    
    F6ext.block<3,1>(3,0) = Kix*imp_e + Kid*imp_ed;
    tau_imp = jacobians.fr_base_J_ee(q).transpose()*F6ext;
    
    // std::cout << imp_e.transpose() << "\t" << imp_ed.transpose() <<std::endl;
    // std::cout << F6ext.transpose() << std::endl;
    // std::cout << tau_imp.transpose() << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // -----------   Ground Contact Force  -----------  //  
    tau_ct = jacobians.fr_base_J_ee(q).transpose()*con_f;       // tau = J.'F   J = [Jw Jv].'
    auto est_force = ( -(jacobians.fr_base_J_ee(q).block<3,3>(3,0).transpose()).inverse() )*tau_gc;

    // std::cout << "est f: " << est_force.transpose() << std::endl;
    
    // -----------   Total Commanded Torque  -----------  //
    tau_t = tau + tau_fb + tau_fr - tau_ct;
    std::cout << tau_cr.transpose() << std::endl;
    std::cout << tau_gc.transpose() << std::endl;
    // std::cout << tau_fb.transpose() << std::endl;
    // std::cout << tau_fr.transpose() << std::endl;
    // std::cout << tau_ct.transpose() << std::endl;

    // -----------   Publish cmd  -----------  //
    publish_cmd(tau_t);
  }

  void bodyController::position_controller()
  {
    // -----------  Update gain parameters  ----------- //
    nh_.getParam("Leg_impedance/Kxx", Kix(0,0));    nh_.getParam("Leg_impedance/Kxy", Kix(1,1));   nh_.getParam("Leg_impedance/Kxz", Kix(2,2));
    nh_.getParam("Leg_impedance/Kdx", Kid(0,0));    nh_.getParam("Leg_impedance/Kdy", Kid(1,1));   nh_.getParam("Leg_impedance/Kdz", Kid(2,2));

    // -----------  Initialize variables  ----------- //
    // Define variables
    Eigen::Matrix<double, 3, 1> qp_e, qv_e, Kd_error;           // Joint position, velocity error; PIDy cumulation
    Eigen::Matrix<double, 3, 1> tau_fb, tau_fr;
    Eigen::Matrix<double, 18, 1> act_cmd;          // PID feedback tau; friction compensation tau; total tau

    // -----------        Error       -----------//
    qp_e = qpr - q;                                               // eigen: position error
    qv_e = qvr - qd;                                              // eigen: velocity error 

    // -----------   Impedance Controller  -----------  //
    Eigen::Matrix<double, 3, 1> t2, imp_e, imp_ed, tau_imp;
    Eigen::MatrixXd F6ext = Eigen::MatrixXd::Zero(6,1);

    // imp_e  = (xH.fr_base_X_ee(q).block<3,1>(0,3)) - (t2 = xH.fr_base_X_ee(qpr).block<3,1>(0,3));   // x - x_ref
    // imp_ed = (jacobians.fr_base_J_ee(q)*qd - jacobians.fr_base_J_ee(qpr)*qvr).block<3,1>(3,0);     // xd - xd_ref
    
    // F6ext.block<3,1>(3,0) = Kix*imp_e + Kid*imp_ed;
    // tau_imp = jacobians.fr_base_J_ee(q).transpose()*F6ext;
    
    // std::cout << imp_e.transpose() << "\t" << imp_ed.transpose() <<std::endl;
    // std::cout << F6ext.transpose() << std::endl;
    // std::cout << tau_imp.transpose() << std::endl;
    // std::cout << "--------------------------------" << std::endl;

    // -----------   Ground Contact Force  -----------  //  
    tau_ct = jacobians.fr_base_J_ee(q).transpose()*con_f;       // tau = J.'F   J = [Jw Jv].'

    // std::cout << "est f: " << est_force.transpose() << std::endl;
    
    // -----------   Total Commanded Torque  -----------  //
    act_cmd = q_temp;
    // act_cmd = qpr;
    std::cout << act_cmd << std::endl;
    // std::cout << tau_fb.transpose() << std::endl;
    // std::cout << tau_fr.transpose() << std::endl;
    // std::cout << tau_ct.transpose() << std::endl;

    // -----------   Publish cmd  -----------  //
    publish_cmd(act_cmd);
  }

  void bodyController::publish_cmd(const Eigen::MatrixXd& data)
  {
    // -----------   Publish cmd  -----------  //
    std_msgs::Float64 float_cmd;

    float_cmd.data = data(0);   lf_q1_pub.publish(float_cmd);
    float_cmd.data = data(1);   lf_q2_pub.publish(float_cmd);
    float_cmd.data = data(2);   lf_q3_pub.publish(float_cmd);

    float_cmd.data = data(3);   lm_q1_pub.publish(float_cmd);
    float_cmd.data = data(4);   lm_q2_pub.publish(float_cmd);
    float_cmd.data = data(5);   lm_q3_pub.publish(float_cmd);

    float_cmd.data = data(6);   lr_q1_pub.publish(float_cmd);
    float_cmd.data = data(7);   lr_q2_pub.publish(float_cmd);
    float_cmd.data = data(8);   lr_q3_pub.publish(float_cmd);

    float_cmd.data = data(9);    rf_q1_pub.publish(float_cmd);
    float_cmd.data = data(10);   rf_q2_pub.publish(float_cmd);
    float_cmd.data = data(11);   rf_q3_pub.publish(float_cmd);

    float_cmd.data = data(12);   rm_q1_pub.publish(float_cmd);
    float_cmd.data = data(13);   rm_q2_pub.publish(float_cmd);
    float_cmd.data = data(14);   rm_q3_pub.publish(float_cmd);

    float_cmd.data = data(15);   rr_q1_pub.publish(float_cmd);
    float_cmd.data = data(16);   rr_q2_pub.publish(float_cmd);
    float_cmd.data = data(17);   rr_q3_pub.publish(float_cmd);
  }
} // namespace body_controller