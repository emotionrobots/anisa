/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup serial_connection Serial Connection
 * @brief Specialized serial port
 * @ingroup communication
 *
 * @{
 */
#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H


#include <vector>
#include <string>
#include <list>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "communication_constants.h"


namespace ComLib
{

  class Message
  {
    public:
      Message(const uint8_t *data, const uint32_t type)
      {
        memcpy(this->data, data, CommunicationConstants::Command::SIZE_TOTAL);
        this->type = static_cast<uint8_t>(type);
      }

      uint8_t *getData()
      {
        return data;
      }

      uint8_t getType()
      {
        return type;
      }

    private:
      uint8_t data[CommunicationConstants::Command::SIZE_TOTAL];
      uint8_t type;
  };


//! Specialized implementation of serial port
/*!
 * This class implements some specific functionality for the communication into the
 * base of serial port
 */
class SerialConnection
{    

public:
    SerialConnection();
    ~SerialConnection();

    bool openPort(std::string &portName);
    void closePort();

    std::vector<std::string> availableDevices();
    ssize_t sendData(uint8_t *data);
    bool sendCommand(uint8_t *data, uint8_t expectedType, bool blocking);
    void addGeneralAnswerType(const uint8_t type);

    //signals:
    boost::signals2::signal< void (const std::vector<uint8_t>&, const uint8_t)>  sigReceivedData;

    int readRxData(int size); //slot

    std::vector<uint8_t> rxArray;
    bool processData(std::vector<uint8_t> array);

  private:
    ssize_t sendCommandInternal(uint8_t *data, uint8_t expectedType_);
    uint32_t calculateChecksum(const uint8_t *data, const uint32_t size);

    int getExpextedSize(const std::vector<uint8_t> &array);
    bool checksumIsCorrect(const std::vector<uint8_t> &array, const unsigned int expectedSize);

    uint8_t getType(const std::vector<uint8_t> &array);
    void setBlocking (int should_block);
    int setInterfaceAttribs (int speed);

    boost::asio::io_service io_service;
    boost::asio::serial_port *serialPort;


    std::vector<std::string> deviceListString;   
    int expectedSize;    
    int fileDescription;

    bool waitForSpecificDataType;
    uint8_t expectedType;
    std::queue<Message> queue;
    std::list<uint8_t> generalAnswerTypes;

};
}

#endif // SERIAL_CONNECTION_H

/** @} */
