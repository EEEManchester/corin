// Leg node utilising action clients for sending goals and monitoring state
// Explanation on action available at: http://wiki.ros.org/actionlib/DetailedDescription
// Action class details at: http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1SimpleActionClient.html

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>

#include <rviz_visual_tools/rviz_visual_tools.h>

using namespace std;

int main(int argc, char **argv)
{
  cout << "TESTING" << endl;
  
  // ROS node and rate
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));

  // Create pose
  Eigen::Isometry3d pose;
  pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

  // Publish arrow vector of pose
  ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
  // visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visual_tools_->publishCone(pose, 0.5, rviz_visual_tools::RED, 0.1);

  // Don't forget to trigger the publisher!
  visual_tools_->trigger();

  while (ros::ok())
  {
    loop_rate.sleep();
    
  }
  return 0;
}

