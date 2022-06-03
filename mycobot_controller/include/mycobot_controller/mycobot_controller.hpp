#pragma once
#include <string>

#include "mycobot_libs/mycobot.h"
#include "sensor_msgs/JointState.h"
#include "mycobot_msgs/MyCobotAngles.h"
#include "mycobot_msgs/GetAngles.h"
#include "ros/ros.h"


namespace mycobot_controller {

  class MyCobotController
  {
  public:
    MyCobotController();
    ~MyCobotController() {}
    // adhoc joint state publisher
    void publishCallback(const ros::TimerEvent&);


  private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    mycobot_libs::MyCobot mycobot_;
    int speed_;
    std::vector<std::string> joint_name_;
    std::vector<double> joint_states_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    size_t num_joints_;

    // adhoc joint state publisher
    ros::Publisher mycobot_joint_states_pub_;
    
    ros::ServiceServer mycobot_get_angle_server_;
    
    ros::Subscriber mycobot_set_angles_sub_;

    void initPublisher();
    void initServer();
    void initSubscriber();

    void publishJointStates();

    void setAnglesCallback(mycobot_msgs::MyCobotAngles msg);

    bool getAngleServerCallback(
      mycobot_msgs::GetAngles::Request &req,
      mycobot_msgs::GetAngles::Response &res
    );
  };
}