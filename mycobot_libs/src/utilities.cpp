#include "mycobot_libs/mycobot.h"

namespace mycobot_libs
{
  MessageUnitPair MyCobot::getMessageUnitPair(const DataUnit &d)
  {
    return {static_cast<MessageUnit>(d >> 8), static_cast<MessageUnit>(d & 0x00FF)};
  }

  DataUnit MyCobot::getDataUnit(const MessageUnit &m1, const MessageUnit &m2)
  {
    return (((DataUnit)m1) << 8) | ((DataUnit)m2);
  }

  Message MyCobot::getMessageFromData(const Data &data)
  {
    auto data_size = data.size();
    Message message(data_size * 2);

    // flatten data_pair
    for (size_t i = 0; i < data_size; i ++)
    {
      auto message_unit_pair = getMessageUnitPair(data[i]);
      message[2 * i] = message_unit_pair.first;
      message[2 * i + 1] = message_unit_pair.second;
    }

    return message;
  }

  Data MyCobot::getDataFromMessage(const Message &message)
  {
    auto data_size = message.size() / 2;
    Data data(data_size);

    for (size_t i = 0; i < data_size; i ++)
    {
      data[i] = getDataUnit(message[2 * i], message[2 * i + 1]);
    }

    return data;
  }
}
