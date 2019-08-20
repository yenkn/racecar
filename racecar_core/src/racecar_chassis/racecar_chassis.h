//
// Created by yenkn on 19-3-12.
//

#ifndef PROJECT_RACECAR_CHASSIS_H
#define PROJECT_RACECAR_CHASSIS_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

#include <racecar_msgs/ChassisCommand.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

namespace racecar_core {

class RacecarChassis {
public:
  RacecarChassis();
  ~RacecarChassis();
  void spin();

protected:
  void loadCommandPriorities();
  void initSerial(const std::string &port);

private:
  /*
   * @struct priorityEntry
   * @brief Entry for each chassis commander loaded from the commanders file. The highest priority is 0.
   *
   * @note the id must be the same as the sender in received chassisCommand messages
   */
  struct priorityEntry {
      std::string id; ///< Unique identifying string for a program that will control the chassis
      int priority; ///< Priority of the commander, 0 is highest priotity
  };

  std::string nodeName_;
  ros::NodeHandle nodeHandle_;
  ros::Publisher speedPublisher_, ekfPublisher_;
  int speedPublishRate_;
  double maxSpeed_;
  double maxSteering_;
  double encoderFactor_, steerMiddle_;

  double currentSpeed_ = 0.0;
  double lastSteering_ = 0.0;

  boost::asio::io_service ioService_;
  boost::asio::serial_port *carPort_;
  int carFd_;

  std::vector<priorityEntry> chassisCommandPriorities_;
  std::map<std::string, racecar_msgs::ChassisCommand> chassisCommands_;
  std::map<std::string, ros::Subscriber> chassisCommandSub_;
  ros::Subscriber emergencySub_;
  ros::Timer chassisControlTimer_, speedTimer_;
  ros::Duration chassisCommandMaxAge_;
  int emergencyMode_ = 0;

  racecar_msgs::ChassisCommand command_;

  void emergencyModeCallback(const std_msgs::Int32ConstPtr &);
  void chassisCommandCallback(const racecar_msgs::ChassisCommandConstPtr &);
  void velCallback(const geometry_msgs::TwistConstPtr &);
  void selectChassisCommand(const ros::TimerEvent &);
  void publishCurrentSpeed(const ros::TimerEvent &);
  void sendCommandToChassis(double throttle, double steering);
};

}

#endif //PROJECT_RACECAR_CHASSIS_H
