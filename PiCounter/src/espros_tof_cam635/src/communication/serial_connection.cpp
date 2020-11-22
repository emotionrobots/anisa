#include <ros/ros.h>

#include "serial_connection.h"
#include "crc_calc.h"
#include "util.h"

//#include <stropts.h>
#include <termios.h>

#define termios asmtermios
#define winsize asmwinsize
#define termio asmtermio

#include <asm/termios.h>
#undef  termios
#undef  winsize
#undef  termio
#include <termios.h>

typedef boost::asio::serial_port_base asio_serial;
using namespace std;

namespace ComLib
{

SerialConnection::SerialConnection()
{  
  expectedSize = 0;
  fileDescription = 0;  
}

SerialConnection::~SerialConnection()
{
  deviceListString.clear();  
  rxArray.clear();
}

void SerialConnection::addGeneralAnswerType(const uint8_t type)
{
  generalAnswerTypes.push_back(type);
}

/**
 * @brief List the available devices
 *
 * This function is used to list the available devices. The index of a device in the list can later be
 * used to select the port to open. The strings can for example be directly put into a comboBox in the GUI and
 * the index is directly given by the comboBox.
 *
 * @return List of strings containing the names of the available devices
 */
vector<string> SerialConnection::availableDevices()
{
  deviceListString.clear();
  string str = "/dev/ttyACM0";
  deviceListString.push_back(str);
  ROS_INFO_STREAM("SerialConnection::availableDevices: "<< str);
  return deviceListString;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool SerialConnection::openPort(std::string &portName)
{  
    if(fileDescription > 0) closePort();

    fileDescription = open(portName.data(), O_RDWR | O_NOCTTY | O_SYNC);

    if(fileDescription <=0){
        fileDescription = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

        if(fileDescription <=0){          
            fileDescription = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_SYNC);

            if(fileDescription <=0){
                fileDescription = open("/dev/ttyACM2", O_RDWR | O_NOCTTY | O_SYNC);

                if(fileDescription <=0){
                    ROS_ERROR ("Error %d opening %s: %s", errno, "/dev/ttyACM2", strerror (errno));
                    return false;
                }
            }
        }
    }

    ROS_INFO ("Open serial port: /dev/ttyACM0: %d", fileDescription);

    setInterfaceAttribs(B4000000);  // set speed to 10000000 bps, 8n1 (no parity)    
    rxArray.clear();
    return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closePort()
{      
    fileDescription = close(fileDescription);
    ROS_INFO("SerialConnection::closePort: %d\n", fileDescription);
}

/**
 * @brief Check, if the checksum is correct
 *
 * This function checks the CRC. It extracts the CRC from the received data and calculates the CRC of the received
 * data and then compares them.
 *
 * @param array Pointer to the received data
 * @param expectedSize Expected size of the received data
 * @return Checksum correct or not
 */
bool SerialConnection::checksumIsCorrect(const vector<uint8_t> &array, const unsigned int expectedSize)
{
  //The received CRC is the one in the data
  uint32_t receivedCrc = Util::getUint32LittleEndian(array, (CommunicationConstants::Data::SIZE_HEADER + expectedSize));

  //The wanted CRC is the one calculated out of the payload    
  uint32_t wantedCrc = calculateChecksum((uint8_t *)array.data(), (CommunicationConstants::Data::SIZE_HEADER + expectedSize));

  if (receivedCrc == wantedCrc){
    return true;
  }

  ROS_ERROR("SerialConnection::checksumIsCorrect ERROR!!!");
  return false;
}

/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
ssize_t SerialConnection::sendData(uint8_t *data)
{
    if(fileDescription <=0 ){
        ROS_ERROR("Error SerialConnection::sendData fileDescription =0 \n");
        return 0;
    }

    //Add the start buffer at the beginning
    data[0] = CommunicationConstants::Command::START_MARK;

    //Calculate the CRC    
     uint32_t crc = calculateChecksum(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)));

    //Add it to the buffer123
    Util::setUint32LittleEndian(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)), crc);

    //This is just to print out the data
    std::string str= "SEND DATA: ";
    char buf[4];
    for(int i=0; i<14; i++){
        sprintf(buf, "%x ", data[i]);
        str.append(buf);
    }

    ROS_DEBUG_STREAM(str);

    return write(fileDescription, (uint8_t *)(data), CommunicationConstants::Command::SIZE_TOTAL);
}


/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int SerialConnection::getExpextedSize(const std::vector<uint8_t> &array)
{
  int expectedSize = Util::getUint16LittleEndian(array, CommunicationConstants::Data::INDEX_LENGTH);
  return expectedSize;
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t SerialConnection::getType(const vector<uint8_t> &array)
{
    return array.at(CommunicationConstants::Data::INDEX_TYPE);
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int SerialConnection::readRxData(int size)
{    
  uint8_t buf[50000] = {0};

  int n = read(fileDescription, buf, size);

  if(n == -1){
    ROS_ERROR("Error on  SerialConnection::readRxData");
    return -1;
  }
  rxArray.insert(std::end(rxArray), buf, buf + n); //Append the new data to the rxArray buffer
  processData(rxArray);
  return n;
}

/**
 * @brief Process the received data
 *
 * @param array Pointer to the received byte array
 */
bool SerialConnection::processData(std::vector<uint8_t> array)
{        
    if (array.size() == 0){
        ROS_ERROR("ERROR SerialConnection::processData array.size() = 0");
        return false;
    }

    //Check for the marking byte
    if(array.at(0) != CommunicationConstants::Data::START_MARK){
        array.erase(array.begin(), array.begin() + 1 );
        return true;
    }

    //Cancel here if no marking bytes

    //Get the expected size. Cancel here if not enough bytes received
    if (array.size() < (static_cast<int>(CommunicationConstants::Data::SIZE_HEADER)))
        return true;

    //Get the expexted size
    expectedSize = getExpextedSize(array);

    //Cancel here if not enough bytes received
    if (array.size() < (static_cast<int>(expectedSize + CommunicationConstants::Data::SIZE_OVERHEAD)))
        return true;

    //Check if the end marking is present. Only use the data if this is the case.
    if (checksumIsCorrect(array, expectedSize))
    {
        uint8_t type = getType(array);
        vector<uint8_t> dataArray = array;
        dataArray.erase(dataArray.begin(), dataArray.begin() + CommunicationConstants::Data::SIZE_HEADER);

        //Remove checksum at the end
        dataArray.erase(dataArray.begin() + expectedSize,  dataArray.begin() + expectedSize + CommunicationConstants::Data::SIZE_CHECKSUM);
        dataArray.erase(dataArray.begin() + expectedSize, dataArray.end());

        //Remove the remaining: thats the checksum
        array.erase(array.begin(), array.begin() + (expectedSize + CommunicationConstants::Data::SIZE_CHECKSUM) );

        sigReceivedData(dataArray, type);

        //If we wait for a specific type and this is now received, handle it
        /*if (waitForSpecificDataType)
        {
          if ((type == expectedType) || ( find(generalAnswerTypes.begin(), generalAnswerTypes.end(), type) != generalAnswerTypes.end() )) {
            sigReceivedData(dataArray, type);
            //timeoutTimer->stop(); //TODO...
            //If there is more data to send in the queue, do it now
            if(queue.empty() == false){
              Message newMessage = queue.front(); //   dequeue();
              queue.pop();
              sendCommandInternal(newMessage.getData(), newMessage.getType());
            } else {
              waitForSpecificDataType = false;
            }
          }
          //It's not the expected type, but there is data, so the communication is doing something. Restart the timeout timer.
          else{
            //timeoutTimer->start(TIMEOUT);
          }
        }else{
          sigReceivedData(dataArray, type);
        }*/

    }else{
        ROS_WARN("corrupted data %s", array.data());
        array.erase(array.begin(), array.begin() + 1);
    }

  array.clear();

  return false;
}

int SerialConnection::setInterfaceAttribs(int speed)
{
  tcflush(fileDescription, TCIOFLUSH);

  struct termios tty;
  memset (&tty, 0, sizeof tty);

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 1;            // read block
  tty.c_cc[VTIME] = 30;           // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  //tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fileDescription, TCSANOW, &tty) != 0){
      ROS_ERROR("Error %d from tcsetattr", errno);
      return -1;
  }

  return 0;
}


void SerialConnection::setBlocking (int should_block){

 struct termios tty;
  memset (&tty, 0, sizeof tty);

  if (tcgetattr (fileDescription, &tty) != 0){
      ROS_ERROR("Error %d from tggetattr", errno);
      return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 30; // 0.5 seconds read timeout

  if (tcsetattr (fileDescription, TCSANOW, &tty) != 0)
      ROS_ERROR ("Error %d setting term attributes", errno);
}


/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
bool SerialConnection::sendCommand(uint8_t *data, uint8_t expectedType, bool blocking)
{
  if (fileDescription <= 0)
  {
    return false;
  }

  //If a command is still ongoing, just put the data into the queue to send it later
  if(waitForSpecificDataType)
  {
    Message newMessage(data, expectedType);
    queue.emplace(newMessage);
    return false;
  }

  sendCommandInternal(data, expectedType);

  //If a blocking command is desired, wait until the answer is received
  if(blocking)
  {
    //Process the events. Otherwise everything will block
    while(waitForSpecificDataType)
    {
      //QCoreApplication::processEvents();
    }

    //This flag is set by the timeout timer signal
    /*if (timeout) //TODO...
    {
      return false;
    }*/
  }

  return true;
}

ssize_t SerialConnection::sendCommandInternal(uint8_t *data, uint8_t expectedType_)
{
  this->expectedType = expectedType_;
  this->waitForSpecificDataType = true;
  //timeoutTimer->start(TIMEOUT);
  //timeout = false;

  //Add the start buffer at the beginning
  data[0] = CommunicationConstants::Command::START_MARK;

  //Calculate the CRC
  uint32_t crc = calculateChecksum(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)));

  //Add it to the buffer
  Util::setUint32LittleEndian(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)), crc);

  return write(fileDescription, (uint8_t *)(data), CommunicationConstants::Command::SIZE_TOTAL);
}

uint32_t SerialConnection::calculateChecksum(const uint8_t *data, const uint32_t size)
{
  return CrcCalc::calcCrc32_32(data, size);
}

} //end namespace ComLib
