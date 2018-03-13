#include "ros/ros.h"
#include "corin_control/JointState.h"
#include "corin_control/Imu.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"

sensor_msgs::JointState current_jointState;
sensor_msgs::Imu gazebo_imu, ahrs_imu;

/*========================================================================
                            Services
  ========================================================================*/

// Service to return motor state and torque for a leg i.e. three motor state and torque
bool getJointState(corin_control::JointState::Request  &req,
                    corin_control::JointState::Response &res)
{
  res.jointState  = current_jointState;                       // motor state
  //ROS_INFO("Request processed");

  return true;
}

// Gazebo Service to return imu state
bool getGzImuState(corin_control::Imu::Request  &req,
                    corin_control::Imu::Response &res)
{
  res.ImuData  = gazebo_imu;                       // imu state
  //ROS_INFO("Request processed");

  return true;
}

// Service to return imu state
bool getAhImuState(corin_control::Imu::Request  &req,
                    corin_control::Imu::Response &res)
{
  res.ImuData  = ahrs_imu;                       // imu state
  //ROS_INFO("Request processed");

  return true;
}
/*========================================================================
                            Subscribers
  ========================================================================*/

// -------------------    Gazebo Motor state     ------------------------------- //
void gz_state_callback(const sensor_msgs::JointState& msg)
{
  current_jointState = msg;
}

// -------------------    Gazebo Imu state     ------------------------------- //
void gz_imu_callback(const sensor_msgs::Imu& msg)
{
  gazebo_imu = msg;
}

// -------------------    myAHRS Imu state    ------------------------------- //
void ahrs_imu_callback(const sensor_msgs::Imu& msg)
{
  ahrs_imu = msg;
}

/*==========================  Main  ===================================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Robot_jointState_server");
  ros::NodeHandle nh_;
  
  //---subscribers---//
  ros::Subscriber joint_sub_  = nh_.subscribe("/corin/joint_states", 1, gz_state_callback);    // subscribe to leg state topic

  ros::Subscriber gz_imu_sub_    = nh_.subscribe("/corin/imu/trunk/data", 1, gz_imu_callback);       // subscribe to gazebo imu topic
  ros::Subscriber ah_imu_sub_    = nh_.subscribe("/imu/data", 1, ahrs_imu_callback);       // subscribe to ahrs imu topic

  //---service server---//
  ros::ServiceServer jointState_service  = nh_.advertiseService("/corin/get_jointState", getJointState);

  ros::ServiceServer imuState_gz_service = nh_.advertiseService("/corin/get_gazebo_imuState", getGzImuState);
  ros::ServiceServer imuState_ah_service = nh_.advertiseService("/corin/get_myAHRS_imuState", getAhImuState);
  ROS_INFO("Ready to get joint states");

  ros::spin();

  return 0;
}
