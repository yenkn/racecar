//
// Created by yenkn on 19-3-13.
//

#ifndef PROJECT_SPEEDCONTROLLER_H
#define PROJECT_SPEEDCONTROLLER_H

#include <string>
#include <deque>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <racecar_msgs/ChassisCommand.h>

#include <racecar_core/math_common/table_interpolation.h>
#include <dynamic_reconfigure/server.h>
#include <racecar_control/speed_controllerConfig.h>

namespace racecar_control {

class SpeedController {
public:
  SpeedController();

  void spin();

protected:
  void sendCommand(double throttle, double steering);

private:
  ros::NodeHandle nodeHandle_;
  ros::Timer controlTimer_, dispTimer_;
  ros::Publisher commandPub_, interPub_, inter1Pub_;
  ros::Subscriber odomSub_;
  ros::Subscriber speedSub_;
  ros::Subscriber cmdVelSub_;

  double maxSpeed_;
  double maxSteering_;
  double setSpeed_ = 0.0, setSteering_ = 0.0, lastSetSpeed_ = 0.0;
  double currentSpeed_ = 0.0;
  double integralError_ = 0.0, adjustPercent_ = 1.0;
  double previousError_ = 0.0;
  double lastPreviousError_ = 0.0;
  double throttle_ = 0.0;
  bool openLoop_ = false;

  char isStarting_ = 0;

  std::deque<double> filterBuffer_;
  std::vector<double> filter_;
  int filterInterval_;
  int filterBufferSize_;

  double maxStep_, stepValue_;
  int stepInterval_;
  std::deque<double> stepBuffer_;

  std::string odomTopic_;
  std::string subTopic_;
  double speedKP_;
  double speedKD_;
  double speedKI_, speedILimit_;
  double speedKInterp_;
  double throttleMax_;
  int controlRate_;

  dynamic_reconfigure::Server<racecar_control::speed_controllerConfig> dsrv_;
  TableInterpolation interpolator_, steerInterpolator_;

  double speedFiltered();
  double speedSteped(double speed);

  void timerCallback(const ros::TimerEvent &);
  void displayCallback(const ros::TimerEvent &);
  void speedCallback(const geometry_msgs::TwistConstPtr &);
  void odomCallback(const nav_msgs::OdometryConstPtr &);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &);
  void paramCallback(racecar_control::speed_controllerConfig &config, uint32_t level);
  void loadThrottleMapping();
  void loadSteerMapping();

};

}

#endif //PROJECT_SPEEDCONTROLLER_H
