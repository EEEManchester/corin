#ifndef FORCE_DISTRIBUTION_H
#define FORCE_DISTRIBUTION_H
 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "corin_control/MotionPlan.h"

#include <cmath> 
#include <math.h> 
#include <typeinfo>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <qpOASES.hpp>

#include "robots/mcorin/declarations.h"
#include "robots/mcorin/transforms.h"
#include "robots/mcorin/jacobians.h"
#include "robots/mcorin/inertia_properties.h"
#include "robots/mcorin/inverse_dynamics.h"
#include "robots/mcorin/miscellaneous.h"

#include <rbdl/rbdl.h>
using namespace iit::mcorin;

namespace force_distribution
{
 
class ForceDistribution
{
 
  public:
   
    ForceDistribution();

  private:
   
    //***************** NODE HANDLES ***************//
    ros::NodeHandle nh_;         // public node handle for subscribing, publishing, etc.
    ros::NodeHandle nh_private_; // private node handle for pulling parameter values from the parameter server
   
    //***************** PUBLISHERS AND SUBSCRIBERS ***************//
    ros::Subscriber sp_subscriber_;
    // ros::Subscriber joint_sub_;

    ros::Publisher cf_pub_;
    
    // // ----- Gazebo Specific----- // //
    // ros::Subscriber gz_fforce_sub_;

    // // ----- DXL Specific ----- // //
    
    
    //***************** STATE VARIABLES ***************//

    // --- RobCoGen variables and class ---- //
    // JointState q, qd, qdd, tau, tau_gc;             // current joint state
    JointState qpr, qvr, qar;                       // reference joint state

    MotionTransforms xM;
    HomogeneousTransforms xH;
    Jacobians jacobians;

    dyn::InertiaProperties inertias;
    dyn::InverseDynamics invdyn;

    // --- Setpoints ---- //    
    Eigen::MatrixXd qbpr, qbvr, qbar;   // base reference states
    Eigen::MatrixXi gait_phase;         // reference gait phase

    //***************** CALLBACKS ***************//
    // void gz_effort_callback(const sensor_msgs::JointState& msg);
    // void dxl_state_callback(const dynamixel_msgs::DxlJointState& msg);
    // void setpoint_callback(const sensor_msgs::JointState& data);
    // void foot_contact_callback(const gazebo_msgs::ContactsState& data);
    // void setpoint_callback(const trajectory_msgs::JointTrajectoryPoint& data);
    void setpoint_callback(const corin_control::MotionPlan& data);

    //***************** FUNCTIONS ***************//
    void qp_solution();
    
    Eigen::Matrix3d skew_this_vector(Eigen::Vector3d data);
    Eigen::MatrixXd CRBI();

    void publish_cmd(const Eigen::MatrixXd& data);

    
}; // class

} // namespace force_distribution
 
#endif // ForceDistribution_H