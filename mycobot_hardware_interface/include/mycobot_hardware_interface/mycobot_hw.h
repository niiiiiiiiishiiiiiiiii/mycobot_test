#pragma once
#include <string>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
// #include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "mycobot_libs/mycobot.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"


namespace mycobot_hardware_interface {

  class MyCobotHW : public hardware_interface::RobotHW
  {
  public:
    MyCobotHW() {}
    ~MyCobotHW() {}
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);
    // // adhoc joint state publisher
    // void publishCallback(const ros::TimerEvent&);

  private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    
    // int speed_;
    // mycobot_libs::MyCobot mycobot_;
    ros::Publisher mycobot_set_angles_pub_;
    ros::ServiceClient mycobot_get_angles_client_;

    // we did not intensionally interface joint_state_interface 
    // hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    
    int num_joints_;
    std::vector<std::string> joint_name_;

    std::vector<double> joint_position_state_;
    std::vector<double> joint_velocity_state_;
    std::vector<double> joint_effort_state_;
    std::vector<double> joint_position_command_;

    // // adhoc joint state publisher
    // ros::Publisher mycobot_joint_states_pub_;
    // void publishJointStates();
  };
}
