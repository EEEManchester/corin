#include <ros/ros.h>
#include "stability_margin.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_control_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS

  //moving_sensor::movingSensor Thing;  // instatiate our class object
  stability_margin::stabilityMargin sMargin;
  ROS_INFO_STREAM( "Class created");
  ros::spin(); // check for new messages and call the callback if we get one

  return 0;
}