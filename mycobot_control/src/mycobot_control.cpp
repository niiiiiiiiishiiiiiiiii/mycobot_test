#include <string>
#include "ros/ros.h"
#include "pluginlib/class_loader.hpp"
// #include "mycobot_hardware_interface/mycobot_hw.hpp"
#include "hardware_interface/robot_hw.h"
#include "controller_manager/controller_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mycobot_control");
  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");

  // To ensure the spawner load the controllers
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Start creating class loader.");
  pluginlib::ClassLoader<hardware_interface::RobotHW> robot_hw_loader("mycobot_control", "hardware_interface::RobotHW");
  ROS_INFO("End creating.");
  ROS_INFO("Start loading MyCobotHW.");
  auto hw = robot_hw_loader.createInstance("mycobot_hardware_interface/MyCobotHW");
  ROS_INFO("End loading.");
  
  hw->init(node_handle, private_node_handle);
  // mycobot_hardware_interface::MyCobotHW hw();
  controller_manager::ControllerManager cm(hw.get(), node_handle);
  // controller_manager::ControllerManager cm(&hw);

  ros::Rate rate(20);

  ROS_INFO("mycobot_control started");
  while (ros::ok())
  {
    hw->read(ros::Time::now(), rate.expectedCycleTime());
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    hw->write(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  
  spinner.stop();
  
  return 0;
}