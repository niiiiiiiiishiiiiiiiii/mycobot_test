#include "mycobot_hardware_interface/mycobot_hw.h"
#include "pluginlib/class_list_macros.hpp"

namespace mycobot_hardware_interface
{
  bool MyCobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
  {
    node_handle_ = root_nh;
    private_node_handle_ = robot_hw_nh;

    // get hardware information of mycobot
    XmlRpc::XmlRpcValue params;
    std::string usb_port = private_node_handle_.param("mycobot_device_info/", std::string("/dev/ttyUSB0"));
    int baud_rate = private_node_handle_.param("mycobot_device_info/", 115200);
    int timeout = private_node_handle_.param("mycobot_device_info/", 10);
    int buffer_size = private_node_handle_.param("mycobot_device_info/", 200);
    speed_ = private_node_handle_.param("mycobot_device_info/", 80);
    
    
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
    mycobot_.init(usb_port, baud_rate, timeout, buffer_size);
    // speed_ = speed;

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
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle position_handle(
        joint_state_handle,
        &joint_position_command_[i]
      );
      position_joint_interface_.registerHandle(position_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    return true;
  }

  void MyCobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_position_state_ = mycobot_.getRadians();
  }

  void MyCobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    mycobot_.sendRadians(joint_position_command_, speed_);
  }
}

PLUGINLIB_EXPORT_CLASS(mycobot_hardware_interface::MyCobotHW, hardware_interface::RobotHW);
