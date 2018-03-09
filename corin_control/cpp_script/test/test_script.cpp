// Leg node utilising action clients for sending goals and monitoring state
// Explanation on action available at: http://wiki.ros.org/actionlib/DetailedDescription
// Action class details at: http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1SimpleActionClient.html

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>

#include "robots/slm/declarations.h"
#include "robots/slm/transforms.h"
#include "robots/slm/inertia_properties.h"
#include "robots/slm/inverse_dynamics.h"

using namespace iit::SLM;

MotionTransforms transforms;
JointState q, qd, qdd, tau;

dyn::InertiaProperties inertias;
dyn::InverseDynamics invdyn(inertias, transforms);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_node");    // initializes node
  ros::NodeHandle n;

  ros::Rate loop_rate(2);
  ROS_INFO("First pass done");

  q(0) = 0.5; // set the joint status
  qd(0) = 0.0;
  qdd(0) = 0.01;
  
  Eigen::Matrix<double, 4, 1> F6ext = Eigen::Matrix<double, 4, 1>::Zero(4,1); 
  
  Eigen::MatrixXd test1, test2 = Eigen::MatrixXd::Zero(4,1);
  std::cout << test1.transpose() << std::endl;
  std::cout << test2.transpose() << std::endl;

  std::cout << "tau: " << tau << std::endl;
  invdyn.id(tau, q, qd, qdd);   // compute inverse dynamics joint forces
  std::cout << tau << std::endl;
  // start main loop
  while (ros::ok())
  {
    
    ROS_INFO_STREAM( "PENDING" );
    
    loop_rate.sleep();
  }
  return 0;
}

