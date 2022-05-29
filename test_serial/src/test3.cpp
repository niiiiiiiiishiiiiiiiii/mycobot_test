#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include "ros/ros.h"
#include "serial/serial.h"

typedef uint8_t MessageUnit;

typedef std::vector<MessageUnit> Message;

typedef double JointValue;

typedef std::string STRING;
typedef uint8_t MessageUnit;
typedef int16_t DataUnit;

typedef struct _MessageUnitPair
{
  MessageUnit upper;
  MessageUnit lower;
} DataPair;
// typedef std::pair<MessageUnit, MessageUnit> MessageUnitPair;

typedef std::vector<MessageUnit> Message;
typedef std::vector<DataUnit> Data;

// typedef struct _Point
// {
//   double position;
//   double velocity;
//   double acceleration;
//   double effort;                              //Torque
// } Point, ActuatorValue, JointValue, ToolValue;

struct ProtocolCode
{
  constexpr static MessageUnit HEADER = 0xFE;
  constexpr static MessageUnit FOOTER = 0xFA;
  constexpr static MessageUnit GET_ANGLES = 0x20;
  constexpr static MessageUnit SEND_ANGLES = 0x22;
};

constexpr MessageUnit ProtocolCode::HEADER;
constexpr MessageUnit ProtocolCode::FOOTER;
constexpr MessageUnit ProtocolCode::GET_ANGLES;
constexpr MessageUnit ProtocolCode::SEND_ANGLES;

class test3
{
private:
  serial::Serial serial_;
  uint32_t buffer_size_;
  Data sendMessage(const MessageUnit &genre);
  void sendMessage(const MessageUnit &genre, const Data &data, const MessageUnit &speed);
  void write(const Message &message, const bool &is_read = true);
  Message read(const MessageUnit &genre);

  static DataPair getDataPair(const DataUnit &d)
  {
    return { static_cast<MessageUnit>(d >> 8), static_cast<MessageUnit>(d & 0x00FF) };
  }
  static DataUnit getDataUnit(const MessageUnit &m1, const MessageUnit &m2)
  {
    return (((DataUnit)m1) << 8) | ((DataUnit)m2);
  }
  static Message getMessageFromData(const Data &data);
  static Data getDataFromMessage(const Message &message);

public:
  test3( 
    STRING usb_port = "/dev/ttyUSB0",
    uint32_t baud_rate = 115200,
    uint32_t timeout = 10,
    uint32_t buffer_size = 200
  );
  ~test3();
  void status();
  void send_radians(const std::vector<JointValue> &radians, int speed);
  std::vector<JointValue> get_radians();
};

Message test3::getMessageFromData(const Data &data)
{
  auto data_size = data.size();
  Message message(data_size * 2);
  std::vector<DataPair> data_pair(data_size);

  // decompose data(2byte) into message pair (1byte * 2)
  std::transform(
    data.begin(), data.end(), data_pair.begin(), getDataPair
  );

  // flatten data_pair
  size_t i = 0;
  std::for_each(
    data_pair.begin(), data_pair.end(),
    [&message, &i](const DataPair &dp) { message[i] = dp.upper; message[i + 1] = dp.lower; i += 2; }
  );
  // auto message  = std::accumulate(
  //   data.begin(), data.end(), 
  //   [](auto &x, auto &y) { x.insert(x.end(), y.begin(), y.end()); return x; }
  // );

  return message;
}

Data test3::getDataFromMessage(const Message &message)
{
  auto data_size = message.size() / 2;
  Data data(data_size);

  for (size_t i = 0; i < data_size; i++)
  {
    data[i] = getDataUnit(message[2 * i], message[2 * i + 1]);
  }

  return data;
}

test3::test3(STRING usb_port, uint32_t baud_rate, uint32_t timeout, uint32_t buffer_size)
  : buffer_size_(buffer_size)
{
  serial_.setBaudrate(baud_rate);
  serial_.setPort(usb_port);
  serial::Timeout timeout_ = serial::Timeout::simpleTimeout(timeout);
  serial_.setTimeout(timeout_);
  serial_.open();
}

test3::~test3()
{
}

void test3::status()
{
  ROS_INFO("In the class: Is the serial port open?");
  if(serial_.isOpen())
    ROS_INFO("Yes.");
  else
    ROS_INFO("No.");
}


std::vector<JointValue> test3::get_radians()
{
  auto angles = sendMessage(ProtocolCode::GET_ANGLES);

  ROS_INFO("read angles:");
  std::for_each(
    angles.begin(), angles.end(),
    [](const DataUnit &m) { std::cout << m << ", "; }
  );
  std::cout << std::endl;

  // calculate
  auto radians = std::vector<JointValue>(angles.size());
  std::transform(
    angles.begin(), angles.end(), 
    radians.begin(), 
    [](const DataUnit &d) { return d * M_PI / 180. / 100.; } 
  );

  return radians;
}

void test3::send_radians(const std::vector<JointValue> &radians, int speed)
{
  // speed assersion
  assert(speed >= 0 && speed <= 100);
  
  auto radians_size = radians.size();

  auto data = Data(radians_size);
  std::transform(
    radians.begin(), radians.end(), 
    data.begin(), 
    [](const JointValue &v) { return static_cast<DataUnit>(round(v * 180. * 100 / M_PI)); }
  );
  // it is extremely bizarre that speed should be sent in 1 byte (unlike other data)
  // data[radians_size] = static_cast<DataUnit>(speed);

  ROS_INFO("created data:");
  std::for_each(
    data.begin(), data.end(),
    [](const DataUnit &m) { std::cout << static_cast<int>(m) << ", "; }
  );
  std::cout << std::endl;

  sendMessage(ProtocolCode::SEND_ANGLES, data, static_cast<DataUnit>(speed));
}

Data test3::sendMessage(const MessageUnit &genre)
{
  Message message({
    ProtocolCode::HEADER, ProtocolCode::HEADER, 
    2, genre,
    ProtocolCode::FOOTER
  });

  ROS_INFO("created message:");
  std::for_each(
    message.begin(), message.end(),
    [](const MessageUnit &m) { std::cout << static_cast<int>(m) << ", "; }
  );
  std::cout << std::endl;

  write(message, true);
  auto dataMessage = read(genre);

  return getDataFromMessage(dataMessage);
}

void test3::sendMessage(const MessageUnit &genre, const Data &data, const MessageUnit &speed)
{
  auto data_size = data.size();

  // message size assersion
  assert((data_size * 2 + 1) + 2 <= 0xFF);

  Message message(
    {
      ProtocolCode::HEADER, ProtocolCode::HEADER, 
      static_cast<MessageUnit>((data_size * 2 + 1) + 2), genre
    }
  );
  auto data_message = getMessageFromData(data);
  message.insert(message.end(), data_message.begin(), data_message.end());
  message.push_back(speed);
  message.push_back(ProtocolCode::FOOTER);
  
  ROS_INFO("created message:");
  std::cout << std::hex;
  std::for_each(
    message.begin(), message.end(),
    [](const MessageUnit &m) { std::cout << std::setw(2) << std::setfill('0') << static_cast<int>(m) << ", "; }
  );
  std::cout << std::dec << std::endl;

  write(message, false);
}


void test3::write(const Message &message, const bool &is_read)
{
  serial_.write(message);
  
  if (is_read)
    serial_.flush(); // get_
  else
    serial_.flushInput(); // send_
}

Message test3::read(const MessageUnit &genre)
{
  // compile fails if Message result();
  auto message = Message();
  size_t data_size, idx_data_start;
  MessageUnit  buffer0, buffer1, buffer2, buffer3;
  MessageUnit buffer[buffer_size_ + 3] = {}; // = std::make_unique<MessageUnit[]>(10);

  size_t i = 0, j = 0;
  ROS_INFO("result message:");
  while (serial_.read(buffer + 3, buffer_size_) > 0)
  {
    std::for_each(
      buffer, buffer + buffer_size_,
      [](const MessageUnit &m) { std::cout << static_cast<int>(m) << ", "; }
    );

    for (i = 0; i < buffer_size_; i++)
    {
      buffer0 = buffer[i];
      buffer1 = buffer[i + 1];
      buffer2 = buffer[i + 2];
      buffer3 = buffer[i + 3];
      if (buffer0 == ProtocolCode::HEADER && buffer1 == ProtocolCode::HEADER && buffer3 == genre)
      {
        data_size = buffer2 - 2;
        idx_data_start = i + 4;
        break;
      }
    }

    if (i == buffer_size_)
    {
      // couldn't find desired message
      buffer[0] = buffer0;
      buffer[1] = buffer1;
      buffer[2] = buffer2;
    }
    else if (i == buffer_size_ - 1)
    {
      // could find but all data lie in the next buffers
      serial_.read(message, data_size);
    }
    else
    {
      if (idx_data_start + data_size > buffer_size_ + 3)
      {
        // data exceeds the buffer
        message.insert(message.end(), buffer + idx_data_start, buffer + buffer_size_ + 3);
        // read residual messages
        auto residual_message = Message(idx_data_start + data_size - buffer_size_ - 3);
        serial_.read(residual_message, idx_data_start + data_size - buffer_size_ - 3);
        message.insert(message.end(), residual_message.begin(), residual_message.end());
      }
      else
      {
        message.insert(message.end(), buffer + idx_data_start, buffer + idx_data_start + data_size);
        break;
      }
    }

    ++j;
    if (j > 1000) break;
  }
  std::cout << std::endl;
  ROS_INFO("read time: %ld, and read message:", j);
  std::for_each(
    message.begin(), message.end(),
    [](const MessageUnit &m) { std::cout << static_cast<int>(m) << ", "; }
  );
  std::cout << std::endl;

  return message;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_serial_3");

  ros::NodeHandle nh("");
  
  STRING usb_port;
  int baud_rate, timeout, buffer_size;
  
  if (!nh.getParam("usb_port", usb_port))
    usb_port =  "/dev/ttyUSB0";
  if (!nh.getParam("baud_rate", baud_rate))
    baud_rate = 115200;
  if (!nh.getParam("timeout", timeout))
    timeout = 10;
  if (!nh.getParam("buffer_size", buffer_size))
    buffer_size = 200;

  // nonnegative assertion
  assert(baud_rate > 0 && timeout > 0 && buffer_size > 4);

  test3 t(usb_port, baud_rate, timeout);

  t.status();

  ROS_INFO("get_radians start\n----------");
  auto radians = t.get_radians();

  ROS_INFO("get radians:");
  std::cout << std::hex;
  std::for_each(
    radians.begin(), radians.end(),
    [](const JointValue &m) { std::cout << m << ", "; }
  );
  std::cout << std::dec << std::endl << std::endl;

  ROS_INFO("send_radians start\n----------");
  // auto target = std::vector<JointValue>({1, 1, 1, 1, 1, 1});
  t.send_radians({0, 0, 0, 0, 0, 0}, 40);

  return 0;
}