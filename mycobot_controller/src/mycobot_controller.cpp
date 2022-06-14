#include "mycobot_controller/mycobot_controller.hpp"
#include <vector>
#include <memory>
#include <algorithm>

namespace mycobot_controller
{
  MyCobotController::MyCobotController()
  : node_handle_(""),
    private_node_handle_("~")
  {
    // get hardware information of mycobot
    std::string usb_port = private_node_handle_.param("mycobot_device_info/usb_port", std::string("/dev/ttyUSB0"));
    int baud_rate = private_node_handle_.param("mycobot_device_info/baud_rate", 115200);
    int timeout = private_node_handle_.param("mycobot_device_info/timeout", 10);
    int buffer_size = private_node_handle_.param("mycobot_device_info/buffer_size", 200);
    int speed = private_node_handle_.param("mycobot_device_info/speed", 80);

    // init mycobot
    mycobot_.init(usb_port, baud_rate, timeout, buffer_size);
    speed_ = speed;

    if(!private_node_handle_.getParam("joints", joint_name_))
    {
      ROS_ERROR("Not found param 'joints'");
      ROS_BREAK();
    }
    num_joints_ = joint_name_.size();

    joint_states_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    std::fill(joint_states_.begin(), joint_states_.end(), 0.0);
    std::fill(joint_velocity_.begin(), joint_velocity_.end(), 0.0);
    std::fill(joint_effort_.begin(), joint_effort_.end(), 0.0);

    initPublisher();
    initServer();
    initSubscriber();
  }

  void MyCobotController::initPublisher()
  {
    mycobot_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
  }

  void MyCobotController::initServer()
  {
    mycobot_get_angle_server_ = node_handle_.advertiseService("get_angles", &MyCobotController::getAngleServerCallback, this);
  }

  void MyCobotController::initSubscriber()
  {
    mycobot_set_angles_sub_ = node_handle_.subscribe<mycobot_msgs::MyCobotAngles>("set_angles", 10, &MyCobotController::setAnglesCallback, this);
  }

  void MyCobotController::publishCallback(const ros::TimerEvent&)
  {
    publishJointStates();
  }

  void MyCobotController::publishJointStates()
  {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    msg.name = joint_name_;
    auto radians = mycobot_.getRadians();
    if (radians.size() == num_joints_)
      joint_states_ = radians;

    msg.position = joint_states_;
    msg.velocity = joint_velocity_;
    msg.effort = joint_effort_;

    mycobot_joint_states_pub_.publish(msg);
  }

  void MyCobotController::setAnglesCallback(mycobot_msgs::MyCobotAngles msg)
  {
    auto radians = std::vector<double>({ msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6 }); 
    mycobot_.sendRadians(radians, speed_);
  }

  bool MyCobotController::getAngleServerCallback(mycobot_msgs::GetAngles::Request &req, mycobot_msgs::GetAngles::Response &res)
  {
    res.joint1 = joint_states_[0];
    res.joint2 = joint_states_[1];
    res.joint3 = joint_states_[2];
    res.joint4 = joint_states_[3];
    res.joint5 = joint_states_[4];
    res.joint6 = joint_states_[5];

    return true;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mycobot_controller");
  ros::NodeHandle node_handle("");

  auto mc_controller = std::make_shared<mycobot_controller::MyCobotController>();

  // update
  ros::Timer publish_timer = node_handle.createTimer(
    ros::Duration(0.02), 
    &mycobot_controller::MyCobotController::publishCallback, 
    mc_controller.get()
  );
  ros::Rate rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}