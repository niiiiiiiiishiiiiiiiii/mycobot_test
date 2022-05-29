#include <string>
#include <vector>

#include "ros/ros.h"
#include "serial/serial.h"

namespace mycobot_libs
{
  typedef uint8_t MessageUnit;
  typedef int16_t DataUnit;
  typedef double AngleValue;

  typedef std::pair<MessageUnit, MessageUnit> MessageUnitPair;
  typedef std::vector<MessageUnit> Message;
  typedef std::vector<DataUnit> Data;

  namespace protocol_code
  {
    const MessageUnit HEADER = 0xFE;
    const MessageUnit FOOTER = 0xFA;
    const MessageUnit GET_ANGLES = 0x20;
    const MessageUnit SEND_ANGLES = 0x22;
  }

  class MyCobot
  {
  private:
    serial::Serial serial_;
    uint32_t buffer_size_;
    Data sendMessage(const MessageUnit &genre);
    void sendMessage(const MessageUnit &genre, const Data &data, const MessageUnit &speed);
    void write(const Message &message, const bool &is_read = true);
    Message read(const MessageUnit &genre);

    static MessageUnitPair getMessageUnitPair(const DataUnit &d);
    static DataUnit getDataUnit(const MessageUnit &m1, const MessageUnit &m2);
    static Message getMessageFromData(const Data &data);
    static Data getDataFromMessage(const Message &message);

  public:
    MyCobot() {}
    MyCobot(
      std::string usb_port,
      uint32_t baud_rate,
      uint32_t timeout,
      uint32_t buffer_size
    );
    ~MyCobot();
    bool init(
      std::string usb_port = "/dev/ttyUSB0",
      uint32_t baud_rate = 115200,
      uint32_t timeout = 10,
      uint32_t buffer_size = 200
    );
    bool getSerialStatus();

    // External interface must take the copy as its arguments although there exists a little overhead
    // especially in the case when we use this interface with RobotHW
    void sendAngles(std::vector<AngleValue> angles, int speed);
    void sendRadians(std::vector<AngleValue> radians, int speed);
    
    std::vector<AngleValue> getAngles();
    std::vector<AngleValue> getRadians();
  };
}
