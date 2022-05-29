#include <cmath>

# include "mycobot_libs/mycobot.h"

using mycobot_libs::MyCobot;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_mycobot");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  
  std::string usb_port = "/dev/ttyUSB0";
  XmlRpc::XmlRpcValue params;
  int baud_rate = 115200, timeout = 10, buffer_size = 200, speed = 40;
  std::vector<double> radians(6, 0);
  
  pnh.getParam("test/", params);

  // usb port
  if (params.hasMember("usb_port"))
  {
    auto param = params["usb_port"];
    ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeString);
    usb_port = std::string(param);
  }
  
  // baud rate
  if (params.hasMember("baud_rate"))
  {
    auto param = params["baud_rate"];
    ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    baud_rate = int(param);
  }

  // timeout
  if (params.hasMember("timeout"))
  {
    auto param = params["timeout"];
    ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    timeout = int(param);
  }

  // buffer_size
  if (params.hasMember("buffer_size"))
  {
    auto param = params["buffer_size"];
    ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
    buffer_size = int(param);
  }
  
  // destination
  if (params.hasMember("destination"))
  {
    auto param_d = params["destination"];    
    if (param_d.hasMember("radians"))
    {
      auto param = param_d["radians"];
      ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeArray && param.size() == 6);
      for (size_t i = 0; i < 6; i++)
      {
        radians[i] = double(param[i]);
      }      
    }

    if (param_d.hasMember("speed"))
    {
      auto param = param_d["speed"];
      ROS_ASSERT(param.getType() == XmlRpc::XmlRpcValue::TypeInt);
      speed = int(param);
    }
  }

  ROS_INFO("usb_port: %s", usb_port.c_str());
  ROS_INFO("baud_rate: %d", baud_rate);
  ROS_INFO("timeout: %d", timeout);
  ROS_INFO("buffer_size: %d", buffer_size);
  ROS_INFO(
    "radians: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    radians[0], radians[1], radians[2], radians[3], radians[4], radians[5]
  );
  ROS_INFO("speed: %d", speed);

  // nonnegative assertion
  ROS_ASSERT(baud_rate > 0 && timeout > 0 && buffer_size > 4);
  // angle assersion
  auto max_radian = *std::max_element(radians.begin(), radians.end());
  auto min_radian = *std::min_element(radians.begin(), radians.end());
  ROS_ASSERT(max_radian < M_PI && min_radian > -M_PI);

  MyCobot mc(usb_port, baud_rate, timeout, buffer_size);

  ROS_INFO("get radians start");
  auto current_radians = mc.getRadians();

  ROS_INFO(
    "get radians: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    current_radians[0], current_radians[1], current_radians[2], 
    current_radians[3], current_radians[4], current_radians[5]
  );

  ROS_INFO("send radians start");
  mc.sendRadians(radians, speed);
  
  return 0;
}