#include "moving_sensor.h"
 
namespace moving_sensor
{
 
movingSensor::movingSensor() :
  nh_(ros::NodeHandle()),             /* This is an initialization list */
  nh_private_(ros::NodeHandle("~"))   /* if you don't know what that means, go read about it */
{
  //***************** RETREIVE PARAMS ***************//
  nh_private_.param<double>("threshold", threshold_, 0.0001);
  // This will pull the "threshold" parameter from the ROS server, and store it in the threshold_ variable.
  // If no value is specified on the ROS param server, then the default value of 0.0001 will be applied
 
  //***************** NODE HANDLES ***************//
  pose_subscriber_ = nh_.subscribe("turtle1/pose", 1, &movingSensor::poseCallback, this);
  // This connects the poseCallback function with the reception of a Pose message on the "turtle1/pose" topic
  // ROS will essentially call the poseCallback function every time it receives a message on that topic.
  // the "1" indicates the length of the queue to hold messages before tossing them.  In this case, our callback
  // function is so fast that 1 is sufficient.
 
  bool_publisher_ = nh_.advertise<std_msgs::Bool>("is_moving", 1);
  // This connects a std_msgs::Bool message on the "is_moving" topic.  The 1 also indicates the length of the queue
  // before tossing messages.  Publishers are generally so fast that 1 almost always works.
}
 
void movingSensor::poseCallback(const turtlesim::PoseConstPtr &msg)
// This function runs every time we get a turtlesim::Pose message on the "turtle1/pose" topic.
// We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
// changing the message, in the case that another node is also listening to it.
{
  std_msgs::Bool out_flag;  // create a new message to store the result of our check in
  if(msg->linear_velocity > threshold_){   // figure out if our velocity is more than the threshold
    out_flag.data = true; // save the result in our new message
  }else{
    out_flag.data = false;
  }
 
  // publish the message to ROS
  bool_publisher_.publish(out_flag);
}
 
} // namespace moving_sensor