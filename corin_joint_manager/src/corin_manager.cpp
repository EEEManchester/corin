#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
// #include "thormang3_feet_ft_module/feet_force_torque_sensor_module.h"

/* Motion Module Header */
// #include "motion_module_tutorial/motion_module_tutorial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CORIN_Manager");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");  // joint offset - not needed
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");   // low-level joint settings

    std::string init_file   = nh.param<std::string>("init_file_path", "");    // high-level joint settings

    /* gazebo simulation */
    controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }
    else
    {
        ROS_WARN("SET TO HARDWARE CONTROL!");
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);

    /* Add Sensor Module */
    // controller->addSensorModule((robotis_framework::SensorModule*)FeetForceTorqueSensor::getInstance());

    /* Add Motion Module */
    // controller->addMotionModule((robotis_framework::MotionModule*)MotionModuleTutorial::getInstance());

    controller->startTimer();

    ROS_INFO("Start spin!");
    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}
