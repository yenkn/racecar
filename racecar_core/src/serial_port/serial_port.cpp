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
 * @file SerialSensorInterface.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SerialSensorInterface class implementation
 *
 ***********************************************/
#include <racecar_core/serial_port/serial_port.h>

#include <ros/time.h>

SerialPort::SerialPort() :
    m_port(""),
    m_settingsApplied(false) {}

SerialPort::SerialPort(ros::NodeHandle &nh,
                       const std::string prefix,
                       const std::string port,
                       const bool queueData) :
    SerialCommon(prefix, port),
    m_port(port),
    m_settingsApplied(false),
    m_queueData(queueData),
    m_mostRecentData(ros::Time::now()) {
  init(nh, prefix, port, queueData);
}


SerialPort::~SerialPort() {
  /*
  ** std::cout is used here instead of ROS_INFO because
  ** when the destructor is called, ros::shutdown has already been executed
  ** and roscore is not running. So ROS_INFO will not print any output.
  ** Additionally, in the launch file the output param must be set as "screen"
  ** to be able to view the std::cout outputs.
  */
  //std::cout << "Shutting down " << m_port.c_str() << " " << close(fileDescriptor()) << std::endl;
}

void SerialPort::init(ros::NodeHandle &nh,
                      const std::string prefix,
                      const std::string port,
                      const bool queueData) {
  std::string parity;
  int baud, stopBits, dataBits;
  bool hardwareFlow, softwareFlow;
  m_port = port;
  m_queueData = queueData;

  SerialCommon::init(prefix, m_port);

  //get current node name to allow access to the serial parameters
  //specific to this node
  nh.param(prefix + "/serialBaud", baud, 38400);
  nh.param<std::string>(prefix + "/serialParity", parity, "none");
  nh.param(prefix + "/serialStopBits", stopBits, 1);
  nh.param(prefix + "/serialDataBits", dataBits, 8);
  nh.param(prefix + "/serialHardwareFlow", hardwareFlow, false);
  nh.param(prefix + "/serialSoftwareFlow", softwareFlow, false);

  m_settingsApplied = connect(m_port,
                              baud,
                              parity,
                              stopBits,
                              dataBits,
                              hardwareFlow,
                              softwareFlow);

  if (m_queueData && m_settingsApplied) {

    //set timer to be able to pull 25% more data off serial port than possible
    m_serialTimer = nh.createTimer(ros::Duration(1.0 / ((baud / 100) * 1.25)),
                                   &SerialPort::pollSerial, this);
  }
}


std::string SerialPort::readPort() {
  char data[100];
  //get up to 100 bytes from the com port, add it to the data buffer
  ssize_t received = 0;
  if (connected()) {
    if ((received = read(fileDescriptor(), &data, 100)) >= 0) {
      //std::cout<<"true";
      data[received] = '\0';
      if ((int) data[received - 1] == 10) {
        data[received - 1] = '\0';
        received--;
      }
      m_data.append(std::string(data));
      m_mostRecentData = ros::Time::now();
      return std::string(data);
    }
  }
  return std::string();
}

void SerialPort::pollSerial(const ros::TimerEvent & /*time*/) {
  char data[100];
  //get up to 100 bytes from the com port, add it to the data buffer
  ssize_t received = 0;
  //ROS_INFO("Polling");
  if (connected()) {
    if ((received = read(fileDescriptor(), &data, 100)) >= 0) {
      //ROS_INFO("RECIEVED");
      //data[received]='\0';
      m_data.append(data, received);
      m_mostRecentData = ros::Time::now();
    }
  }
}
