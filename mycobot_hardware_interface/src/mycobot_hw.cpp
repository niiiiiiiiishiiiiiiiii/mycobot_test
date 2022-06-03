#include <algorithm>

#include "mycobot_hardware_interface/mycobot_hw.h"
#include "pluginlib/class_list_macros.hpp"

#include "mycobot_msgs/MyCobotAngles.h"
#include "mycobot_msgs/GetAngles.h"

namespace mycobot_hardware_interface
{
  bool MyCobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
  {
    node_handle_ = root_nh;
    private_node_handle_ = robot_hw_nh;

    // get hardware information of mycobot
    // std::string usb_port = private_node_handle_.param("mycobot_device_info/", std::string("/dev/ttyUSB0"));
    // int baud_rate = private_node_handle_.param("mycobot_device_info/", 115200);
    // int timeout = private_node_handle_.param("mycobot_device_info/", 10);
    // int buffer_size = private_node_handle_.param("mycobot_device_info/", 200);
    // speed_ = private_node_handle_.param("mycobot_device_info/", 80);
    
    
    // private_node_handle_.getParam("mycobot_device_info", params);
    // // usb port
    // if (params.hasMember("usb_port")) 
    // {
    //   auto param = params["usb_port"];
    //   ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeString);
    //   usb_port = std::string(param);
    // }
    // // baud rate
    // if (params.hasMember("baud_rate"))
    // {
    //   auto param = params["baud_rate"];
    //   ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    //   baud_rate = int(param);
    // }
    // // timeout
    // if (params.hasMember("timeout"))
    // {
    //   auto param = params["timeout"];
    //   ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    //   timeout = int(param);
    // }
    // // buffer_size
    // if (params.hasMember("buffer_size"))
    // {
    //   auto param = params["buffer_size"];
    //   ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    //   buffer_size = int(param);
    // }
    // // buffer_size
    // if (params.hasMember("speed"))
    // {
    //   auto param = params["speed"];
    //   ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    //   speed = int(param);
    // }

    // init mycobot
    // mycobot_.init(usb_port, baud_rate, timeout, buffer_size);
    // speed_ = speed;

    mycobot_set_angles_pub_ = node_handle_.advertise<mycobot_msgs::MyCobotAngles>("set_angles", 10);
    mycobot_get_angles_client_ = node_handle_.serviceClient<mycobot_msgs::GetAngles>("get_angles");

    if(!private_node_handle_.getParam("joints", joint_name_))
    {
      ROS_ERROR("Not found param 'joints'");
      ROS_BREAK();
    }
    num_joints_ = joint_name_.size();

    joint_position_state_.resize(num_joints_);
    joint_velocity_state_.resize(num_joints_);
    joint_effort_state_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
      
    for (size_t i = 0; i < num_joints_; i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(
        joint_name_[i],
        &joint_position_state_[i],
        &joint_velocity_state_[i],
        &joint_effort_state_[i]
      );
      // joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle position_handle(
        joint_state_handle,
        &joint_position_command_[i]
      );
      position_joint_interface_.registerHandle(position_handle);
    }

    // registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    // init adhoc joint state publisher
    // mycobot_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);

    return true;
  }

  void MyCobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    // joint_position_state_ = mycobot_.getRadians();
    mycobot_msgs::GetAngles srv;
    mycobot_get_angles_client_.call(srv);
    joint_position_state_[0] = srv.response.joint1;
    joint_position_state_[1] = srv.response.joint2;
    joint_position_state_[2] = srv.response.joint3;
    joint_position_state_[3] = srv.response.joint4;
    joint_position_state_[4] = srv.response.joint5;
    joint_position_state_[5] = srv.response.joint6;
    // ROS_INFO("%f", joint_position_state_[0]);
  }

  void MyCobotHW::write(const ros::Time& time, const ros::Duration& period)
  { 
    // mycobot_.sendRadians(joint_position_command_, speed_);
    mycobot_msgs::MyCobotAngles msg;
    msg.joint1 = joint_position_command_[0];
    msg.joint2 = joint_position_command_[1];
    msg.joint3 = joint_position_command_[2];
    msg.joint4 = joint_position_command_[3];
    msg.joint5 = joint_position_command_[4];
    msg.joint6 = joint_position_command_[5];
    mycobot_set_angles_pub_.publish(msg);
  }

  // void MyCobotHW::publishCallback(const ros::TimerEvent&)
  // {
  //   publishJointStates();
  // }

  // void MyCobotHW::publishJointStates()
  // {
  //   sensor_msgs::JointState msg;
  //   msg.header.stamp = ros::Time::now();

  //   msg.name = joint_name_;
  //   msg.position = mycobot_.getRadians();

  //   mycobot_joint_states_pub_.publish(msg);
  // }
}

PLUGINLIB_EXPORT_CLASS(mycobot_hardware_interface::MyCobotHW, hardware_interface::RobotHW);
