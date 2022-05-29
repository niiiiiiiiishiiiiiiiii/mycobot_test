#include <algorithm>
#include <cmath>

#include "mycobot_libs/mycobot.h"

namespace mycobot_libs
{
  MyCobot::MyCobot(std::string usb_port, uint32_t baud_rate, uint32_t timeout, uint32_t buffer_size)
    : buffer_size_(buffer_size)
  {
    serial_.setBaudrate(baud_rate);
    serial_.setPort(usb_port);
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(timeout);
    serial_.setTimeout(timeout_);
    serial_.open();
  }

  MyCobot::~MyCobot()
  {

  }

  bool MyCobot::init(std::string usb_port, uint32_t baud_rate, uint32_t timeout, uint32_t buffer_size)
  {
    buffer_size_ = buffer_size;
    serial_.setBaudrate(baud_rate);
    serial_.setPort(usb_port);
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(timeout);
    serial_.setTimeout(timeout_);
    serial_.open();

    return getSerialStatus();
  }

  bool MyCobot::getSerialStatus()
  {
    return serial_.isOpen();
  }

  std::vector<AngleValue> MyCobot::getAngles()
  {
    auto angles = sendMessage(protocol_code::GET_ANGLES);

    // calculate
    auto degrees = std::vector<AngleValue>(angles.size());
    std::transform(
      angles.begin(), angles.end(), 
      degrees.begin(), 
      [](const DataUnit &d) { return static_cast<AngleValue>(d) / 100.; } 
    );
    ROS_INFO("Get[degree]: %d -> %f", angles[2], degrees[2]);

    return degrees;
  }

  // std::vector<AngleValue> MyCobot::getRadians()
  // {
  //   auto degrees = getAngles();

  //   // transform (degree -> radian)
  //   auto radians = std::vector<AngleValue>(degrees.size());
  //   for (size_t i = 0; i < degrees.size(); i++)
  //   {
  //     radians[i] = degrees[i] * M_PI / 180.;
  //   }
    
  //   // std::transform(
  //   //   degrees.begin(), degrees.end(), 
  //   //   radians.begin(), 
  //   //   [](const AngleValue &d) { return d * M_PI / 180.; } 
  //   // );
  //   ROS_INFO("Get [rad]: %f -> %f", degrees[2], radians[2]);

  //   return radians;
  // }

  std::vector<AngleValue> MyCobot::getRadians()
  {
    auto angles = sendMessage(protocol_code::GET_ANGLES);

    // calculate
    auto radians = std::vector<AngleValue>(angles.size());
    std::transform(
      angles.begin(), angles.end(), 
      radians.begin(), 
      [](const DataUnit &d) { return static_cast<AngleValue>(d) / 100. * M_PI / 180.; } 
    );
    // ROS_INFO("Get: (%d, %d, %d, %d, %d, %d) -> (%f, %f, %f, %f, %f, %f)", 
    //   angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], 
    //   radians[0], radians[1], radians[2], radians[3], radians[4], radians[5]
    // );

    return radians;
  }

  void MyCobot::sendAngles(std::vector<AngleValue> degrees, int speed)
  {
    // angle assersion
    auto max_degree = *std::max_element(degrees.begin(), degrees.end());
    auto min_degree = *std::min_element(degrees.begin(), degrees.end());
    ROS_ASSERT(max_degree < 180 && min_degree > -180);
    // speed assersion
    ROS_ASSERT(speed >= 0 && speed <= 100);
    
    auto degrees_size = degrees.size();

    auto data = Data(degrees_size);
    std::transform(
      degrees.begin(), degrees.end(), 
      data.begin(), 
      [](const AngleValue &v) { return static_cast<DataUnit>(round(v * 100)); }
    );
    // it is extremely bizarre that speed should be sent in 1 byte (unlike other data)
    // data[degrees_size] = static_cast<DataUnit>(speed);


    // ROS_INFO("Send: %f -> %d", degrees[2], data[2]);

    sendMessage(protocol_code::SEND_ANGLES, data, static_cast<DataUnit>(speed));
  }

  void MyCobot::sendRadians(std::vector<AngleValue> radians, int speed)
  {
    std::vector<AngleValue> degrees(radians.size());

    // transform (degree -> radian)
    std::transform(
      radians.begin(), radians.end(), 
      degrees.begin(), 
      [](const AngleValue &d) { return d * 180. / M_PI; }
    );
    // ROS_INFO("Send [rad]: %f -> %f", radians[2], degrees[2]);
    
    sendAngles(degrees, speed);
  }

  Data MyCobot::sendMessage(const MessageUnit &genre)
  {
    Message message({
      protocol_code::HEADER, protocol_code::HEADER, 
      2, genre,
      protocol_code::FOOTER
    });

    write(message, true);
    auto dataMessage = read(genre);

    return getDataFromMessage(dataMessage);
  }

  void MyCobot::sendMessage(const MessageUnit &genre, const Data &data, const MessageUnit &speed)
  {
    auto data_size = data.size();

    // message size assersion
    ROS_ASSERT((data_size * 2 + 1) + 2 <= 0xFF);

    Message message(
      {
        protocol_code::HEADER, protocol_code::HEADER, 
        static_cast<MessageUnit>((data_size * 2 + 1) + 2), genre
      }
    );
    auto data_message = getMessageFromData(data);
    message.insert(message.end(), data_message.begin(), data_message.end());
    message.push_back(speed);
    message.push_back(protocol_code::FOOTER);

    write(message, false);
  }


  void MyCobot::write(const Message &message, const bool &is_read)
  {
    serial_.write(message);
  
    // Output : PC -> MyCobot
    // Input  : MyCobot -> PC
    if (is_read)
      serial_.flush(); // get_
    else
      // Only send message
      serial_.flushOutput(); // send_
  }

  Message MyCobot::read(const MessageUnit &genre)
  {
    // compile fails if Message result();
    auto message = Message();
    size_t data_size, idx_data_start;
    MessageUnit  buffer0, buffer1, buffer2, buffer3;
    MessageUnit buffer[buffer_size_ + 3] = {0}; // = std::make_unique<MessageUnit[]>(10);

    size_t i = 0;
    while (serial_.read(buffer + 3, buffer_size_) > 0)
    {
      for (i = 0; i < buffer_size_; i++)
      {
        buffer0 = buffer[i];
        buffer1 = buffer[i + 1];
        buffer2 = buffer[i + 2];
        buffer3 = buffer[i + 3];
        if (buffer0 == protocol_code::HEADER && buffer1 == protocol_code::HEADER && buffer3 == genre)
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
    }

    return message;
  }
}
