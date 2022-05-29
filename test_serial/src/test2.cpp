#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#define UPPER_UNIT(d) static_cast<MessageUnit>((d) >> 8)
#define LOWER_UNIT(d) static_cast<MessageUnit>((d) & 0x00FF)
#define GET_DATAPAIR(d) DataPair{UPPER_UNIT(d), LOWER_UNIT(d)}
#define GET_DATA(m1, m2) ((m1) << 8 | (m2))

typedef uint8_t MessageUnit;
typedef uint16_t DataUnit;

typedef struct _MessageUnitPair
{
  MessageUnit upper;
  MessageUnit lower;
} DataPair;

typedef std::vector<MessageUnit> Message;
typedef std::vector<DataUnit> Data;

typedef double JointValue;

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

std::vector<JointValue> get_radians();

void send_radians(const std::vector<JointValue> &radians, int speed);

std::vector<JointValue> sendMessage(MessageUnit genre);
void sendMessage(MessageUnit genre, const Data &data);
// std::vector<JointValue> sendMessage(MessageUnit genre, Message message, std::vector<JointValue> &result);

Message getMessageFromData(const Data &data);

std::vector<JointValue> get_radians()
{
  auto angles = sendMessage(ProtocolCode::GET_ANGLES);

  // calculate
  auto radians = std::vector<JointValue>(angles.size());
  std::transform(
    angles.begin(), angles.end(), 
    std::back_inserter(radians), 
    [](const JointValue &v) { return v * M_PI / 180.; } 
  );

  return angles;
}

void send_radians(const std::vector<JointValue> &radians, int speed)
{
  // speed assersion
  assert(speed >= 0 && speed <= 100);
  
  auto radians_size = radians.size();

  auto data = Data(radians_size + 1);
  std::transform(
    radians.begin(), radians.end(), 
    data.begin(), 
    [](const JointValue &v) { return static_cast<DataUnit>(round(v * 180. / M_PI)); }
  );
  data[radians_size] = static_cast<DataUnit>(speed);

  sendMessage(ProtocolCode::SEND_ANGLES, data);
}

std::vector<JointValue> sendMessage(MessageUnit genre)
{
  auto command = Message({
    ProtocolCode::HEADER, ProtocolCode::HEADER, 
    genre, 2, 
    ProtocolCode::FOOTER
  });

  ROS_INFO("created command:");
  std::for_each(
    command.begin(), command.end(),
    [](const MessageUnit &m) { std::cout << static_cast<int>(m) << ", "; }
  );
  std::cout << std::endl;

  return std::vector<JointValue>({1});
}

void sendMessage(MessageUnit genre, const Data &data)
{
  auto data_size = data.size();

  // message size assersion
  assert(data_size *2 + 2 <= 0xFF);

  auto command = Message(
    {
      ProtocolCode::HEADER, ProtocolCode::HEADER, 
      genre, static_cast<MessageUnit>(data_size * 2 + 2)
    }
  );
  auto data_message = getMessageFromData(data);
  command.insert(command.end(), data_message.begin(), data_message.end());
  command.push_back(ProtocolCode::FOOTER);
  
  ROS_INFO("created command:");
  std::for_each(
    command.begin(), command.end(),
    [](const MessageUnit &m) { std::cout << static_cast<int>(m) << ", "; }
  );
  std::cout << std::endl;
}

Message getMessageFromData(const Data &data)
{
  auto data_size = data.size();
  Message message(data_size * 2);
  std::vector<DataPair> data_pair(data_size);

  ROS_INFO("1");

  // decompose data(2byte) into message pair (1byte * 2)
  std::transform(
    data.begin(), data.end(), data_pair.begin(), 
    [](const DataUnit &data_unit) { return DataPair{UPPER_UNIT(data_unit), LOWER_UNIT(data_unit)}; }
  );

  ROS_INFO("2");

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

  ROS_INFO("3");

  return message;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_serial_2");

  ros::NodeHandle node_handle("");

  ROS_INFO("get radians!");
  get_radians();
  double r = 1.;
  ROS_INFO(
    "send radians (%f, %f, %f, %f, %f, %f)!", 
    r * 180. / M_PI, r * 180. / M_PI, r * 180. / M_PI, r * 180. / M_PI, r * 180. / M_PI, r * 180. / M_PI);
  send_radians({1., 1., 1., 1., 1., 1.}, 10);

  return 0;
}