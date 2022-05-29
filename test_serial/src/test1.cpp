#include <string>

typedef std::string STRING;

#include "ros/ros.h"
#include "serial/serial.h"

class test1
{
private:
  serial::Serial serial_;
public:
  test1(STRING usb_port, uint32_t baud_rate);
  ~test1();
  void status();
};

test1::test1(STRING usb_port, uint32_t baud_rate)
{
  serial_.setBaudrate(baud_rate);
  serial_.setPort(usb_port);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(timeout);
  serial_.open();
}

test1::~test1()
{
}

void test1::status()
{
  ROS_INFO("In the class: Is the serial port open?");
  if(serial_.isOpen())
    ROS_INFO("Yes.");
  else
    ROS_INFO("No.");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_serial");

  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  uint32_t baud_rate = 115200;
  serial::Serial my_serial(usb_port, baud_rate, serial::Timeout::simpleTimeout(1000));

  ROS_INFO("Is the serial port open?");
  if(my_serial.isOpen())
    ROS_INFO("Yes.");
  else
    ROS_INFO("No.");

  test1 serial_test(usb_port, baud_rate);
  serial_test.status();

  return 0;
}
