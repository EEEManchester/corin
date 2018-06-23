#ifndef FORCE_DISTRIBUTION_H
#define FORCE_DISTRIBUTION_H
 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

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
    
    //***************** DYNAMIC RECONFIGURE ***************//
    // dynamic_reconfigure::Server<mcorin_control::dynreconfigConfig> server;
    // dynamic_reconfigure::Server<mcorin_control::dynreconfigConfig>::CallbackType dconfig;

    // double dyn_data;
    
    //***************** PARAMETERS ***************//
    std::vector<double> PIDq1_, PIDq2_, PIDq3_;     // Joint PID feedback gains
    double Kxx_, Kxy_, Kxz_;                        // Joint impedance spring stiffness
    double Kdx_, Kdy_, Kdz_;                        // Joint impedance damping
    
    //***************** STATE VARIABLES ***************//

    int active_legs;                                // number of active legs
    int size_n;                                     // joint state vector size
    std::string controller_type;                    // controller type: position or effort

    // --- RobCoGen variables and class ---- //
    JointState q, qd, qdd, tau, tau_gc;             // current joint state
    JointState qpr, qvr, qar;                       // reference joint state

    MotionTransforms xM;
    HomogeneousTransforms xH;
    Jacobians jacobians;

    dyn::InertiaProperties inertias;
    dyn::InverseDynamics invdyn;

    // --- Contact Forces and Torque ---- //
    Eigen::MatrixXd con_f;                      // 6x1
    Eigen::MatrixXd tau_ct, tau_cr;             // nx1;
    Eigen::MatrixXd contact_force;              // nx1;
    
    // --- Setpoints ---- //    
    Eigen::MatrixXd qbpr, qbvr, qbar;           // base reference states

    // --- Impedance controller ---- //
    Eigen::MatrixXd Kix, Kid;                   // nxn; gains for impedance controller

    // --- PID feedback controller ---- //
    Eigen::MatrixXd f_state, sum_error;         // nx1; PID static terms
    Eigen::MatrixXd PID_Kp, PID_Ki, PID_Kd;     // nxn; gains for PID controller

    double ts, cN;                              // PID parameters: sampling time, PIDy D term, filter coefficient
    
    // --- Friction compensation ---- //
    Eigen::MatrixXd qzd;                        // joint damping
    Eigen::MatrixXd tau_cou;

    // temp
    Eigen::MatrixXd q_temp;

    //***************** CALLBACKS ***************//
    // void gz_effort_callback(const sensor_msgs::JointState& msg);
    // void dxl_state_callback(const dynamixel_msgs::DxlJointState& msg);
    // void setpoint_callback(const sensor_msgs::JointState& data);
    void setpoint_callback(const trajectory_msgs::JointTrajectoryPoint& data);
    // void foot_contact_callback(const gazebo_msgs::ContactsState& data);

    //***************** FUNCTIONS ***************//
    void qp_solution();
    void publish_cmd(const Eigen::MatrixXd& data);

    
}; // class

} // namespace force_distribution
 
#endif // ForceDistribution_H