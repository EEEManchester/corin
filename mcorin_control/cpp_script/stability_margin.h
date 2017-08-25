#ifndef STABILITY_MARGIN_H
#define STABILITY_MARGIN_H
 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ContactsState.h"
#include "tf/transform_broadcaster.h"

#include <cmath> 
#include <math.h> 
#include <typeinfo>
#include <Eigen/Dense>
#include <vector>

#include "dynamixel_msgs/DxlJointState.h"

#include "robots/mcorin/declarations.h"
#include "robots/mcorin/transforms.h"
#include "robots/mcorin/jacobians.h"
#include "robots/mcorin/inertia_properties.h"
#include "robots/mcorin/inverse_dynamics.h"

using namespace iit::mcorin;

namespace stability_margin
{
 
class stabilityMargin
{
 
  public:
   
    stabilityMargin();
  private:
   
    //***************** NODE HANDLES ***************//
    ros::NodeHandle nh_;         // public node handle for subscribing, publishing, etc.
    ros::NodeHandle nh_private_; // private node handle for pulling parameter values from the parameter server
   
    //***************** PUBLISHERS AND SUBSCRIBERS ***************//
    ros::Subscriber joint_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher smargin_pub_;
    ros::Publisher odometry_pub_;

    // // ----- Gazebo Specific----- // //
    ros::Subscriber gz_fforce_sub_;

    // // ----- DXL Specific ----- // //
    

    //***************** PARAMETERS ***************//
    std::vector<double> temp1_, PIDq2_, PIDq3_;     // random
    double temp2;                                   // random
    
    //***************** STATE VARIABLES ***************//

    int size_n;                                     // joint state vector size
    double robot_mass;

    tf::TransformBroadcaster br;                    // broadcaster for robot state

    // --- Imu ---- //
    sensor_msgs::Imu imu_data;

    // --- RobCoGen variables and class ---- //
    JointState qp_, qv_, qa_, tau;                  // current joint state
    JointState qpr, qvr, qar;                       // reference joint state

    MotionTransforms xM;
    HomogeneousTransforms xH;
    Jacobians jacobians;

    dyn::InertiaProperties inertias;
    dyn::InverseDynamics invdyn;

    // --- Contact Forces and Torque ---- //
    Eigen::MatrixXd tau_cr;
    Eigen::MatrixXd contact_force;              // nx1;

    // --- State variables ---- //
    Eigen::Vector3d trunk_X_com;
    Eigen::MatrixXd base_composite_spatial_inertia, com_composite_spatial_inertia;
    Eigen::MatrixXd base_composite_rotational_inertia, com_composite_rotational_inertia;
    Eigen::MatrixXd xM_fr_base_X_com;
    
    //***************** CALLBACKS ***************//
    void imu_callback(const sensor_msgs::Imu& msg);

    void gz_state_callback(const sensor_msgs::JointState& msg);
    void dxl_state_callback(const dynamixel_msgs::DxlJointState& msg);
    
    void foot_contact_callback(const gazebo_msgs::ContactsState& data);

    //***************** FUNCTIONS ***************//
    void stability_margin_state();
    void state_estimation();

    void center_of_mass();
    void get_composite_inertia();
    
}; // class

} // namespace STABILITY_MARGIN
 
#endif // stabilityMargin_H