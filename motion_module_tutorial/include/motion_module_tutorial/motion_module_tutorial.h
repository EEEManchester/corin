#ifndef MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_
#define MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"



class MotionModuleTutorial
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<MotionModuleTutorial>
{
private:
  int           control_cycle_msec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Subscriber sub1_;
  ros::Publisher pub1_;

  void queueThread();

public:
  MotionModuleTutorial();
  virtual ~MotionModuleTutorial();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::Int16::ConstPtr &msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
};



#endif /* MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_ */
