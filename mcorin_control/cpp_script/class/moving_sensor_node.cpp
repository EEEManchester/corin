#include <ros/ros.h>
#include "moving_sensor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moving_sensor_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS

  moving_sensor::movingSensor Thing;  // instatiate our class object

  ros::spin(); // check for new messages and call the callback if we get one

  return 0;
}