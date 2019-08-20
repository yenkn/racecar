/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file SerialPortThreaded.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SerialPortThreaded class implementation
 *
 ***********************************************/
#include <racecar_core/serial_port/serial_port_threaded.h>

#include <sys/select.h>

#include <ros/time.h>

SerialPortThreaded::SerialPortThreaded() :
    m_port(""),
    m_settingsApplied(false),
    m_alive(false) {}

SerialPortThreaded::SerialPortThreaded(ros::NodeHandle &nh,
                                       const std::string &prefix,
                                       const std::string &port,
                                       const bool queueData) :
    SerialCommon(prefix, port),
    m_port(port),
    m_settingsApplied(false),
    m_alive(false) {
  init(nh, prefix, port, queueData);
}

SerialPortThreaded::~SerialPortThreaded() {
  /*
  ** std::cout is used here instead of ROS_INFO because
  ** when the destructor is called, ros::shutdown has already been executed
  ** and roscore is not running. So ROS_INFO will not print any output.
  ** Additionally, in the launch file the output param must be set as "screen"
  ** to be able to view the std::cout outputs.
  */

  //clearDataCallback();

  //if the serial thread isnt dead yet, wait for it to close
  if (m_alive) {
    m_alive = false;
    std::cout << "Joining " << m_port.c_str() << std::endl;
    m_runThread->join();
    std::cout << "Joined " << m_port.c_str() << std::endl;
  }
  //std::cout << "Shutting down " << m_port.c_str() << " " << close(fileDescriptor()) << std::endl;
}

void SerialPortThreaded::init(ros::NodeHandle &nh,
                              const std::string &prefix,
                              const std::string &port,
                              const bool queueData) {
  std::string parity;
  int baud, stopBits, dataBits;
  bool hardwareFlow, softwareFlow;
  m_port = port;


  SerialCommon::init(prefix, m_port);

  //get current node name to allow access to the serial parameters
  //specific to this node
  //std::string nName = nodeName;//+((portHandle.length()>0)?"/"+portHandle:"");
  //std::cout << portName+((portHandle.empty())?"":"/"+portHandle) << std::endl;

  if (!nh.getParam(prefix + "/serialBaud", baud) ||
      !nh.getParam(prefix + "/serialParity", parity) ||
      !nh.getParam(prefix + "/serialStopBits", stopBits) ||
      !nh.getParam(prefix + "/serialDataBits", dataBits) ||
      !nh.getParam(prefix + "/serialHardwareFlow", hardwareFlow) ||
      !nh.getParam(prefix + "/serialSoftwareFlow", softwareFlow)) {
    ROS_ERROR("Could not get all SerialPortThreaded parameters for %s", prefix.c_str());
  }

  m_settingsApplied = this->connect(m_port,
                                    baud,
                                    parity,
                                    stopBits,
                                    dataBits,
                                    hardwareFlow,
                                    softwareFlow);

  if (queueData && m_settingsApplied) {
    //start worker in separate thread
    m_alive = true;
    m_runThread = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&SerialPortThreaded::run, this)));
  }
}

void SerialPortThreaded::run() {
  fd_set rfds;
  struct timeval tv;
  int retval;
  char data[512];
  ssize_t received;

  if (!connected()) {
    std::cout << "Not connected, cannot start reading" << std::endl;
    return;
  }

  while (m_alive) {
    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(fileDescriptor(), &rfds);

    /* Wait up to one seconds. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    retval = select(fileDescriptor() + 1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (retval == -1) {
      ROS_ERROR("select() error");
    } else if (retval) {
      /* FD_ISSET(0, &rfds) will be true. */
      if ((received = read(fileDescriptor(), &data, 512)) >= 0) {
        m_dataMutex.lock();
        m_data.append(data, received);

        m_dataMutex.unlock();
        //callback triggered within same thread
        if (m_dataCallback) {
          try {
            m_dataCallback();
          } catch (boost::bad_function_call &) {
            //catch this exception for cleaner shutdown
            std::cout << "Caught bad function call in SerialPortThreaded (probably during shutdown)" << std::endl;
          }
        }
        //condition can notify (wake) other threads waiting for data
//        m_waitCond.notify_all();
      }
    } else {
      ROS_ERROR("No data within previous second");
    }
  }

  //since ros is shutdown and ROS diag messages wouldnt go anywhere
  std::cout << "SerialPortThreaded Done Running " << fileDescriptor() << std::endl;
}

void SerialPortThreaded::lock() {
  m_dataMutex.lock();
}

bool SerialPortThreaded::tryLock() {
  return m_dataMutex.try_lock();
}

void SerialPortThreaded::unlock() {
  m_dataMutex.unlock();
}

void SerialPortThreaded::registerDataCallback(DataCallback callback) {
  m_dataCallback = callback;
}

void SerialPortThreaded::clearDataCallback() {
  m_dataCallback = NULL;
}

//void SerialPortThreaded::waitForData()
//{
//  boost::unique_lock<boost::mutex> lock(m_waitMutex);
//  m_waitCond.wait(lock);
//}

//bool SerialPortThreaded::waitForData(const long& timeMS)
//{
//  boost::unique_lock<boost::mutex> lock(m_waitMutex);
//  return m_waitCond.timed_wait(lock, boost::posix_time::milliseconds(timeMS));
//}

int SerialPortThreaded::writePort(const std::string data) {
  if (connected()) {
    boost::unique_lock<boost::mutex> lock(m_writeMutex);
    return SerialCommon::writePort(data);
  }
  return -1;
}

int SerialPortThreaded::writePort(const unsigned char *data, unsigned int length) {
  if (connected()) {
    boost::unique_lock<boost::mutex> lock(m_writeMutex);
    return SerialCommon::writePort(data, length);
  }
  return -1;
}

int SerialPortThreaded::writePortTry(const std::string data) {
  if (connected()) {
    boost::unique_lock<boost::mutex> lock(m_writeMutex, boost::try_to_lock);
    if (lock) {
      return SerialCommon::writePort(data);
    }
  }
  return -1;
}

int SerialPortThreaded::writePortTry(const unsigned char *data, unsigned int length) {
  if (connected()) {
    boost::unique_lock<boost::mutex> lock(m_writeMutex, boost::try_to_lock);
    if (lock) {
      return SerialCommon::writePort(data, length);
    }
  }
  return -1;
}
