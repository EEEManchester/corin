#include "stability_margin.h"
#include "kinematics/kdl.h"

namespace stability_margin
{
 
stabilityMargin::stabilityMargin() : invdyn(inertias, xM),
  nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~"))   // initialize node
  {
    //***************** INITIALIZE VARIABLES ***************//

    size_n  = 18;
    robot_mass = inertias.getMass_trunk() + (inertias.getMass_LF_hipassembly() + inertias.getMass_LF_upperleg() + inertias.getMass_LF_lowerleg())*6;

    // nx1 initialization
    tau_cr = contact_force = Eigen::MatrixXd::Zero(size_n,1);

    // 3x3 initialization
    com_composite_rotational_inertia = base_composite_rotational_inertia = Eigen::MatrixXd::Zero(3,3);

    // 6x6 initialization
    com_composite_spatial_inertia = base_composite_spatial_inertia = Eigen::MatrixXd::Zero(6,6);
    xM_fr_base_X_com = Eigen::MatrixXd::Identity(6,6);

    //***************** RETREIVE PARAMS ***************//
    // nh_private_.param<double>("Leg_impedance/Kdx", Kix(0,0), Kxx_);    
    // nh_private_.param("Leg_fb_q1/PID", PIDq1_, {0.15, 0.0, 0.01});
   
    //***************** NODE HANDLES ***************//
    smargin_pub_  = nh_.advertise<std_msgs::Float64>("/mcorin/stability_margin/state", 1); 
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/mcorin/odometry", 1); 

    imu_sub_      = nh_.subscribe("/imu/data", 1, &stabilityMargin::imu_callback, this);

    // // ----- Gazebo ----- // //
    // gz_fforce_sub_  = nh_.subscribe("/corin/leg_1_foot_force", 1, &stabilityMargin::foot_contact_callback, this);
    joint_sub_      = nh_.subscribe("/mcorin/joint_states", 1, &stabilityMargin::gz_state_callback, this);

    // // ----- DXL ----- // //
    // ros::Subscriber sub_joint = n.subscribe("/robot_state/dynamixel_port", 1, dxl_state_callback);
    // q1_pub_torque = n.advertise<std_msgs::Float64>("/leg_1_q1_controller/torque_command", 1);    
    // q2_pub_torque = n.advertise<std_msgs::Float64>("/leg_1_q2_controller/torque_command", 1); 
  }
  
  //********************************* Functions *********************************//
  void stabilityMargin::stability_margin_state()
  {
    // update robot state
    stabilityMargin::center_of_mass();
    stabilityMargin::get_composite_inertia();

    // Compute FK using current joint position - 2D or 3D projections
    Eigen::Vector2d p1 = xH.fr_trunk_X_LF_foot(qp_).block<2,1>(0,3) - trunk_X_com.block<2,1>(0,0);
    Eigen::Vector2d p2 = xH.fr_trunk_X_RF_foot(qp_).block<2,1>(0,3) - trunk_X_com.block<2,1>(0,0);

    // Compute vector perpendicular to support polygon & intersect CoM
    Eigen::Vector2d p1p = ((-p1.dot(p2-p1))/pow((p2-p1).norm(),2))*(p2-p1);
    Eigen::Vector2d dv1 = p1 + p1p;

    // magnitude of vector perpendicular to support polygon
    double dv1_mag = dv1.norm();

    // std::cout << dv1_mag << std::endl;
    // std::cout << composite_rotational_inertia << std::endl;
    std::cout << "---------------" <<std::endl;

    // -----------   Ground Contact Force  -----------  //  
    // tau_ct = jacobians.fr_base_J_ee(q).transpose()*con_f;       // tau = J.'F   J = [Jw Jv].'
    // std::cout << "SM callback" << std::endl;
    
  }

  void stabilityMargin::get_composite_inertia()
  {
    auto inertia_LF_hip = xM.fr_LF_hipassembly_X_fr_trunk(qp_).transpose()*inertias.getTensor_LF_hipassembly()*xM.fr_LF_hipassembly_X_fr_trunk(qp_);
    auto inertia_LF_upp = xM.fr_LF_upperleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_LF_upperleg()*xM.fr_LF_upperleg_X_fr_trunk(qp_);
    auto inertia_LF_low = xM.fr_LF_lowerleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_LF_lowerleg()*xM.fr_LF_lowerleg_X_fr_trunk(qp_);

    auto inertia_LM_hip = xM.fr_LM_hipassembly_X_fr_trunk(qp_).transpose()*inertias.getTensor_LM_hipassembly()*xM.fr_LM_hipassembly_X_fr_trunk(qp_);
    auto inertia_LM_upp = xM.fr_LM_upperleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_LM_upperleg()*xM.fr_LM_upperleg_X_fr_trunk(qp_);
    auto inertia_LM_low = xM.fr_LM_lowerleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_LM_lowerleg()*xM.fr_LM_lowerleg_X_fr_trunk(qp_);

    auto inertia_LR_hip = xM.fr_LR_hipassembly_X_fr_trunk(qp_).transpose()*inertias.getTensor_LR_hipassembly()*xM.fr_LR_hipassembly_X_fr_trunk(qp_);
    auto inertia_LR_upp = xM.fr_LR_upperleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_LR_upperleg()*xM.fr_LR_upperleg_X_fr_trunk(qp_);
    auto inertia_LR_low = xM.fr_LR_lowerleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_LR_lowerleg()*xM.fr_LR_lowerleg_X_fr_trunk(qp_);

    auto inertia_RF_hip = xM.fr_RF_hipassembly_X_fr_trunk(qp_).transpose()*inertias.getTensor_RF_hipassembly()*xM.fr_RF_hipassembly_X_fr_trunk(qp_);
    auto inertia_RF_upp = xM.fr_RF_upperleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_RF_upperleg()*xM.fr_RF_upperleg_X_fr_trunk(qp_);
    auto inertia_RF_low = xM.fr_RF_lowerleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_RF_lowerleg()*xM.fr_RF_lowerleg_X_fr_trunk(qp_);

    auto inertia_RM_hip = xM.fr_RM_hipassembly_X_fr_trunk(qp_).transpose()*inertias.getTensor_RM_hipassembly()*xM.fr_RM_hipassembly_X_fr_trunk(qp_);
    auto inertia_RM_upp = xM.fr_RM_upperleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_RM_upperleg()*xM.fr_RM_upperleg_X_fr_trunk(qp_);
    auto inertia_RM_low = xM.fr_RM_lowerleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_RM_lowerleg()*xM.fr_RM_lowerleg_X_fr_trunk(qp_);

    auto inertia_RR_hip = xM.fr_RR_hipassembly_X_fr_trunk(qp_).transpose()*inertias.getTensor_RR_hipassembly()*xM.fr_RR_hipassembly_X_fr_trunk(qp_);
    auto inertia_RR_upp = xM.fr_RR_upperleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_RR_upperleg()*xM.fr_RR_upperleg_X_fr_trunk(qp_);
    auto inertia_RR_low = xM.fr_RR_lowerleg_X_fr_trunk(qp_).transpose()*inertias.getTensor_RR_lowerleg()*xM.fr_RR_lowerleg_X_fr_trunk(qp_);

    base_composite_spatial_inertia = inertias.getTensor_trunk() +
                                inertia_LF_hip + inertia_LF_upp + inertia_LF_low +
                                inertia_LM_hip + inertia_LM_upp + inertia_LM_low +
                                inertia_LR_hip + inertia_LR_upp + inertia_LR_low +
                                inertia_RF_hip + inertia_RF_upp + inertia_RF_low +
                                inertia_RM_hip + inertia_RM_upp + inertia_RM_low +
                                inertia_RR_hip + inertia_RR_upp + inertia_RR_low;
    base_composite_rotational_inertia = base_composite_spatial_inertia.block<3,3>(0,0);

    com_composite_spatial_inertia = xM_fr_base_X_com.transpose()*base_composite_spatial_inertia*xM_fr_base_X_com;
    com_composite_rotational_inertia = com_composite_spatial_inertia.block<3,3>(0,0);
  }

  void stabilityMargin::center_of_mass()
  {
    Eigen::MatrixXd com_sum = (inertias.getMass_LF_hipassembly()*xH.fr_trunk_X_LF_hipassemblyCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_LF_upperleg()*xH.fr_trunk_X_LF_upperlegCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_LF_lowerleg()*xH.fr_trunk_X_LF_lowerlegCOM(qp_).block<3,1>(0,3)) +
                              (inertias.getMass_LM_hipassembly()*xH.fr_trunk_X_LM_hipassemblyCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_LM_upperleg()*xH.fr_trunk_X_LM_upperlegCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_LM_lowerleg()*xH.fr_trunk_X_LM_lowerlegCOM(qp_).block<3,1>(0,3)) + 
                              (inertias.getMass_LR_hipassembly()*xH.fr_trunk_X_LR_hipassemblyCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_LR_upperleg()*xH.fr_trunk_X_LR_upperlegCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_LR_lowerleg()*xH.fr_trunk_X_LR_lowerlegCOM(qp_).block<3,1>(0,3)) + 
                              (inertias.getMass_RF_hipassembly()*xH.fr_trunk_X_RF_hipassemblyCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_RF_upperleg()*xH.fr_trunk_X_RF_upperlegCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_RF_lowerleg()*xH.fr_trunk_X_RF_lowerlegCOM(qp_).block<3,1>(0,3)) + 
                              (inertias.getMass_RM_hipassembly()*xH.fr_trunk_X_RM_hipassemblyCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_RM_upperleg()*xH.fr_trunk_X_RM_upperlegCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_RM_lowerleg()*xH.fr_trunk_X_RM_lowerlegCOM(qp_).block<3,1>(0,3)) + 
                              (inertias.getMass_RR_hipassembly()*xH.fr_trunk_X_RR_hipassemblyCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_RR_upperleg()*xH.fr_trunk_X_RR_upperlegCOM(qp_).block<3,1>(0,3) + 
                                inertias.getMass_RR_lowerleg()*xH.fr_trunk_X_RR_lowerlegCOM(qp_).block<3,1>(0,3));  

    trunk_X_com = com_sum/robot_mass;
    xM_fr_base_X_com(4,0) =  trunk_X_com(2);  //  z
    xM_fr_base_X_com(5,0) = -trunk_X_com(1);  // -y  
    xM_fr_base_X_com(3,1) = -trunk_X_com(2);  // -z
    xM_fr_base_X_com(5,1) =  trunk_X_com(0);  //  x
    xM_fr_base_X_com(3,2) =  trunk_X_com(1);  //  y
    xM_fr_base_X_com(4,2) = -trunk_X_com(0);  // -x

    std::cout << "com location:  "<< xM_fr_base_X_com << std::endl;
  }

  void stabilityMargin::state_estimation()
  {
    // static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.5) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "trunk"));

    //***************************************************************
    nav_msgs::Odometry odom;

    odom.pose.pose.position.x = 0.1;
    odom.pose.pose.orientation = imu_data.orientation;
    odom. twist.twist.angular  = imu_data.angular_velocity;

    std::cout << imu_data.angular_velocity << std::endl;
    odometry_pub_.publish(odom);
  }
  //********************************* Callbacks *********************************//
  void stabilityMargin::gz_state_callback(const sensor_msgs::JointState& msg)
  {
    unsigned char nj;
    nj = size_n;        // number of joints
    
    for (int i=0; i < size_n; i++){
      qp_(i)  = msg.position[i];       // rcg: current joint space position
      qv_(i)  = msg.velocity[i];       // rcg: current joint space velocity

      tau_cr(i,0) = msg.effort[i];    // eigen: current joint space effort
    }
    // force_torque_estimation();
    stabilityMargin::stability_margin_state(); // leg controller
   
  }
  void stabilityMargin::dxl_state_callback(const dynamixel_msgs::DxlJointState& msg)
  {
    for (int i=0; i < size_n; i++){
      qp_(i)  = msg.position[i];
      qv_(i) = msg.velocity[i];
    }
    //std::cout << q << std::endl;
    // stability_margin_state(nj);
  }

  void stabilityMargin::foot_contact_callback(const gazebo_msgs::ContactsState& data)
  {
    if (not data.states.empty()){
      contact_force(0,0) = data.states[0].total_wrench.force.x;
      contact_force(1,0) = data.states[0].total_wrench.force.y;
      contact_force(2,0) = data.states[0].total_wrench.force.z;

      // Transform to SCS frame
      // contact_force = xH.fr_base_X_ee(qp_).block<3,3>(0,0)*contact_force;
    }
    else{
      for (int i=0; i < 3; i++)       // force vector
        contact_force(i,0) = 0.0;
    }
    // std::cout << contact_force.transpose() << std::endl;      // reported in body frame
  }

  void stabilityMargin::imu_callback(const sensor_msgs::Imu& msg)
  {
    imu_data = msg;
    stabilityMargin::state_estimation();
  }

} // namespace stability_margin