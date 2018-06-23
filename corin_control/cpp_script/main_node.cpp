#include <ros/ros.h>
// #include "body_controller.h"
#include "force_distribution.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_control_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS

  // body_controller::bodyController* mcorin_class = new body_controller::bodyController();
  force_distribution::ForceDistribution* ForceClass = new force_distribution::ForceDistribution();

  // instatiate our class object
  ROS_INFO_STREAM( "Corin Force Node Ready");


  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  // dynamic_reconfigure::Server<mcorin_control::dynreconfigConfig> dr_srv;
  // dynamic_reconfigure::Server<mcorin_control::dynreconfigConfig>::CallbackType cb;
  // cb = boost::bind(&body_controller::bodyController::dconfig_callback, mcorin_class, _1, _2);
  // dr_srv.setCallback(cb);

  ros::spin(); // check for new messages and call the callback if we get one

  return 0;
}