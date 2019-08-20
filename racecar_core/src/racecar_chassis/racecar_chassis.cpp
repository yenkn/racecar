//
// Created by yenkn on 19-3-12.
//

#include "racecar_chassis.h"
#include <ros/ros.h>
#include <racecar_core/utils/std.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <fcntl.h>

#define LOWORD(X) ((u_char)((X) & 0xff))
#define HIWORD(X) ((u_char)(((X) & 0xff00) >> 8))

namespace racecar_core {

RacecarChassis::RacecarChassis() {
  nodeHandle_ = ros::NodeHandle("~");
  emergencySub_ = nodeHandle_.subscribe("/emergency_mode", 1, &RacecarChassis::emergencyModeCallback, this);
  loadCommandPriorities();

  nodeHandle_.param("racecarProperties/max_speed", maxSpeed_, 10.0);
  nodeHandle_.param("racecarProperties/max_steering", maxSteering_, 0.7);

  nodeName_ = ros::this_node::getName();

  std::string port;
  nodeHandle_.param<std::string>("port", port, "/dev/car");

  nodeHandle_.param("encoderFactor", encoderFactor_, 2.95 / 100 * M_PI / (512 * 0.005));
  nodeHandle_.param("steerMiddle", steerMiddle_, 1500.0);

  std::cout << "factor: " << encoderFactor_ << std::endl;

  initSerial(port);

  double commandRate, commandMaxAge;
  nodeHandle_.param("commandRate", commandRate, 30.0);
  nodeHandle_.param("commandMaxAge", commandMaxAge, 2.0);
  nodeHandle_.param("speedPublishRate", speedPublishRate_, 30);

  speedPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/speed", 1);
  ekfPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/speed_ekf", 1);

  chassisCommandMaxAge_ = ros::Duration(commandMaxAge);

  chassisControlTimer_ = nodeHandle_.createTimer(ros::Rate(commandRate),
                                                 &RacecarChassis::selectChassisCommand, this);

  speedTimer_ = nodeHandle_.createTimer(ros::Rate(speedPublishRate_), &RacecarChassis::publishCurrentSpeed, this);
}

void RacecarChassis::initSerial(const std::string &port) {
  carPort_ = new boost::asio::serial_port(ioService_);
  try {
    carPort_->open(port);
  }
  catch (boost::system::system_error &error) {
    ROS_ERROR("Failed to open port with error %s", error.what());
    return;
  }

  if (!carPort_->is_open()) {
    ROS_ERROR("failed to open serial port");
    return;
  }

  typedef boost::asio::serial_port_base sb;

  sb::baud_rate baud_option(38400);
  sb::flow_control flow_control(sb::flow_control::none);
  sb::parity parity(sb::parity::none);
  sb::stop_bits stop_bits(sb::stop_bits::one);

  carPort_->set_option(baud_option);
  carPort_->set_option(flow_control);
  carPort_->set_option(parity);
  carPort_->set_option(stop_bits);

  const char *path = port.c_str();
  carFd_ = open(path, O_RDWR);
  if (carFd_ < 0) {
    ROS_ERROR("Port Error!: %s", path);
    return;
  }

  ROS_INFO("port %s opened", path);
}

RacecarChassis::~RacecarChassis() {
  close(carFd_);
  delete carPort_;
}

void RacecarChassis::loadCommandPriorities() {
  //read in chassisCommandPriorities from the parameter server that were loaded by the launch file
  XmlRpc::XmlRpcValue v;
  nodeHandle_.param("chassisCommandPriorities", v, v);

  for (auto &mapIt : v) {
    if (mapIt.second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      //add entry in priority queue and command map
      std::string sender = mapIt.first;
      int priority = static_cast<int>(mapIt.second);
      chassisCommandPriorities_.push_back({sender, priority});
      chassisCommands_[mapIt.first] = racecar_msgs::ChassisCommand();

    } else {
      ROS_INFO("%s XmlRpc chassis command priorities formatted incorrectly", nodeName_.c_str());
    }
  }

  //sort the loaded commanders according to their priority
  std::sort(chassisCommandPriorities_.begin(), chassisCommandPriorities_.end(),
            [](priorityEntry a, priorityEntry b) { return a.priority > b.priority; });

  for (auto &vecIt : chassisCommandPriorities_) {
    std::string topic = "/" + vecIt.id + "/chassis_command";
    ros::Subscriber sub = nodeHandle_.subscribe(topic, 1, &RacecarChassis::chassisCommandCallback, this);
    chassisCommandSub_[vecIt.id] = sub;

    ROS_INFO("%s loaded commander %s with priority %d", nodeName_.c_str(), vecIt.id.c_str(), vecIt.priority);
  }
  ROS_INFO("%s loaded %d priorities", nodeName_.c_str(), (int) chassisCommandPriorities_.size());
}

void RacecarChassis::chassisCommandCallback(const racecar_msgs::ChassisCommandConstPtr &msg) {
  std::map<std::string, racecar_msgs::ChassisCommand>::iterator mapIt;
  if ((mapIt = chassisCommands_.find(msg->sender)) == chassisCommands_.end()) {
    ROS_INFO("%s: Unknown controller %s attempting to control chassis", nodeName_.c_str(), msg->sender.c_str());
  }
  {
    mapIt->second = *msg;
  }
}

void RacecarChassis::selectChassisCommand(const ros::TimerEvent &) {
  if (emergencyMode_ > 0) {
    sendCommandToChassis(0.0, 0.0);
    return;
  }
  ros::Time currentTime = ros::Time::now();

  bool foundThrottle = false, foundSteering = false;

  //find highest priority (lowest valuemostRecentRc_) command message for each actuator across all valid actuator commands
  for (auto &vecIt : chassisCommandPriorities_) {
    if (currentTime - chassisCommands_[vecIt.id].header.stamp < chassisCommandMaxAge_) {
      //valid throttle commands are on [-1,1], only set throttle value if runstop is enabled
      if (
          chassisCommands_[vecIt.id].throttle <= 1.0 &&
          chassisCommands_[vecIt.id].throttle >= -1.0 && !foundThrottle) {
        command_.throttle = chassisCommands_[vecIt.id].throttle;
        foundThrottle = true;
      }

      //valid steeringBrake commands are on [-1,1]
      if (chassisCommands_[vecIt.id].steering <= 1.0 &&
          chassisCommands_[vecIt.id].steering >= -1.0 && !foundSteering) {
        command_.steering = chassisCommands_[vecIt.id].steering;
        foundSteering = true;
      }
    }
  }

  //send actuator commands down to chassis, sets to calibrated neutral if no valid commander
  sendCommandToChassis(command_.throttle, command_.steering);
}

void RacecarChassis::publishCurrentSpeed(const ros::TimerEvent &) {
  geometry_msgs::Twist msg;
  msg.linear.x = currentSpeed_;
  speedPublisher_.publish(msg);

  geometry_msgs::TwistWithCovarianceStamped tmsg;
  tmsg.header.frame_id = "base_footprint";
  tmsg.header.stamp = ros::Time::now();
  tmsg.twist.twist = msg;
  tmsg.twist.covariance = {
      0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ekfPublisher_.publish(tmsg);
}

void RacecarChassis::sendCommandToChassis(double throttle, double steering) {
  auto angle = static_cast<int16_t>(steerMiddle_ - steering * 1000.0); // 1500 middle; < 1500 to right; > 1500 to left;
  auto speed = static_cast<int16_t>(1500.0 + throttle * 1000.0);

  if (steering != lastSteering_) printf("new steering: %.2f, %d, %.6f\n", steering, angle, ros::Time::now().toSec());
  lastSteering_ = steering;

  u_char data[7];
  data[0] = 0xAA; // frame header
  data[1] = LOWORD(speed);
  data[2] = HIWORD(speed);
  data[3] = LOWORD(angle);
  data[4] = HIWORD(angle);
  data[5] = data[1] + data[2] + data[3] + data[4];
  data[6] = 0x55; // frame footer

//  printf("output [%d, %d]: ", speed, angle);
//  for (int i = 0; i < 7; i++)
//  {
//    printf("%02X ", data[i]);
//  }
//  putchar('\n');
  write(carFd_, data, 7);
}

void RacecarChassis::emergencyModeCallback(const std_msgs::Int32ConstPtr &mode) {
  ROS_INFO("EMERGENCY MODE [%d]", mode->data);
  emergencyMode_ = mode->data;
}

void RacecarChassis::spin() {
  ros::Rate r(speedPublishRate_);

  while (nodeHandle_.ok()) {
    ssize_t recived = 0;
    unsigned char buffer[4] = {0};
    tcflush(carFd_, TCIFLUSH);
    if ((recived = read(carFd_, buffer, 4))) {
      if (recived != 4) continue;
      if (buffer[0] != 0xBB || buffer[3] != 0x66) continue;
      auto *cnt = (int16_t *) (buffer + 1);
      double distance = encoderFactor_ * (double) (*cnt);
      currentSpeed_ = distance;
    }
    ros::spinOnce();
  }
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "racecar_chassis");
  racecar_core::RacecarChassis chassis;
  chassis.spin();
  return 0;
}
