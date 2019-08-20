//
// Created by yenkn on 19-3-13.
//

#include "speed_controller.h"

#include <map>
#include <std_msgs/Float32.h>

namespace racecar_control {

SpeedController::SpeedController(): interpolator_(InterpolateMode::Linear, BoundaryType::Clamp), steerInterpolator_(InterpolateMode::Linear, BoundaryType::Clamp) {
  nodeHandle_ = ros::NodeHandle("~");

  nodeHandle_.param<std::string>("odom", odomTopic_, "/odometry/filtered");
  nodeHandle_.param<std::string>("commander", subTopic_, "/cmd_vel");
  nodeHandle_.param<double>("throttle_max", throttleMax_, 1.0);
  nodeHandle_.param("control_rate", controlRate_, 100);

  int displayRate = 0;
  nodeHandle_.param("display_rate", displayRate, 100);

  nodeHandle_.param("racecar_properties/max_speed", maxSpeed_, 10.0);
  nodeHandle_.param("racecar_properties/max_steering", maxSteering_, 0.7);

  // dsrv_.setCallback(std::bind(&SpeedController::paramCallback, this, std::placeholders::_1, std::placeholders::_2));

  if (!nodeHandle_.getParam("KP", speedKP_) ||
      !nodeHandle_.getParam("KD", speedKD_) ||
      !nodeHandle_.getParam("KI", speedKI_) || !nodeHandle_.getParam("KILimit", speedILimit_) ||
    !nodeHandle_.getParam("KInterp", speedKInterp_) || !nodeHandle_.getParam("AdjustPercent", adjustPercent_)) {
    ROS_ERROR("Could not get all PID params");
  }

  nodeHandle_.param("openLoop", openLoop_, false);

  nodeHandle_.param("filter", filter_, { 1.0 });
  nodeHandle_.param("filter_interval", filterInterval_, 100);
  filterBufferSize_ = ((int)filter_.size()-1) * filterInterval_ + 1;

  nodeHandle_.param("max_step", maxStep_, 1.2);
  nodeHandle_.param("step_value", stepValue_, 0.2);
  nodeHandle_.param("step_interval", stepInterval_, 5);

  loadThrottleMapping();
  loadSteerMapping();

  odomSub_ = nodeHandle_.subscribe(odomTopic_, 1, &SpeedController::odomCallback, this);
  cmdVelSub_ = nodeHandle_.subscribe(subTopic_, 1, &SpeedController::cmdVelCallback, this);
  speedSub_ = nodeHandle_.subscribe("/speed", 1, &SpeedController::speedCallback, this);
  commandPub_ = nodeHandle_.advertise<racecar_msgs::ChassisCommand>("/controller/chassis_command", 1);

  interPub_ = nodeHandle_.advertise<std_msgs::Float32>("/inter", 1);
  inter1Pub_ = nodeHandle_.advertise<std_msgs::Float32>("/inter1", 1);

  controlTimer_ = nodeHandle_.createTimer(ros::Rate(controlRate_), &SpeedController::timerCallback, this);
  dispTimer_ = nodeHandle_.createTimer(ros::Rate(displayRate), &SpeedController::displayCallback, this);

  ROS_INFO("PID values: %.6f, %.6f, %.6f", speedKP_, speedKI_, speedKD_);
  ROS_INFO("Filter buffer: %d", filterBufferSize_);
  std::cout << "Filter: ";
  for(auto &s : filter_) {
    std::cout << s << ", ";
  }
  std::cout << std::endl;
}

void SpeedController::spin() {
  ros::spin();
}

void SpeedController::paramCallback(racecar_control::speed_controllerConfig &config, uint32_t level) {
//  speedKP_ = config.speed_p;
//  speedKI_ = config.speed_i;
//  speedKD_ = config.speed_d;
//
//  ROS_INFO("PID recived: %.6f, %.6f, %.6f", speedKP_, speedKI_, speedKD_);
}

void SpeedController::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
  // currentSpeed_ = odom->twist.twist.linear.x;
}

void SpeedController::speedCallback(const geometry_msgs::TwistConstPtr &speed) {
  currentSpeed_ = speed->linear.x;
  if(isStarting_ == 1 && currentSpeed_ >= setSpeed_ * 0.8) {
    isStarting_ = -1;
    ROS_INFO("starting end......");
  }

  if (filterBuffer_.size() == filterBufferSize_) {
    filterBuffer_.pop_back();
  }
  if(stepBuffer_.size() == stepInterval_ && stepInterval_ > 0) {
    stepBuffer_.pop_back();
  }
  filterBuffer_.push_front(currentSpeed_);
  stepBuffer_.push_front(speedFiltered());
}

double SpeedController::speedSteped(double speed) {
  if(stepInterval_ == 0 || stepBuffer_.size() != stepInterval_) return speed;
  double diff = speed - stepBuffer_.back();
  if(fabs(diff) > maxStep_) {
    double sign = (diff > 0) - (diff < 0);
    return stepBuffer_.back() + sign * stepValue_;
  }
  return speed;
}

double SpeedController::speedFiltered() {
  if(filter_.empty() || filterBuffer_.size() != filterBufferSize_) return currentSpeed_;
  double result = 0.0;
  for(int i = 0; i < filter_.size(); i++) {
    result += filter_[i] * filterBuffer_[i * filterInterval_];
  }
  return result;
}

void SpeedController::displayCallback(const ros::TimerEvent &) {
  if (std::abs(setSpeed_) > 0.01) {
    ROS_INFO("set: %.2f, current: %.2f, throttle: %.8f", setSpeed_, currentSpeed_, throttle_);
  }
}

void SpeedController::timerCallback(const ros::TimerEvent &) {
  double dt = ros::Rate(controlRate_).cycleTime().toSec();
  double curSpeed = isStarting_ == 1 ? currentSpeed_ : speedSteped(speedFiltered());
  double error = setSpeed_ - curSpeed;

  if(openLoop_) {
    double interpolateValue = interpolator_.interpolate(setSpeed_);
    throttle_ = interpolateValue * speedKInterp_;
  } else {
    float isInter = 1.0;
//    if(fabs(error) <= adjustPercent_ * fabs(setSpeed_ - lastSetSpeed_) && fabs(setSpeed_) > 0.0) {
//      isInter = 1.0;
//    }

//    throttle_ += speedKP_ * (error - previousError_) +
//                 isInter * speedKI_ * error +
//                 speedKD_ * (error - previousError_ * 2 + lastPreviousError_);

    integralError_ += error;
    integralError_ = std::max(-speedILimit_, std::min(speedILimit_, integralError_));
    throttle_ = speedKP_ * error + isInter * speedKI_ * integralError_ + speedKD_ * (error - previousError_);

//    if(fabs(error) < adjustPercent_) {
//      integralError_ += error;
//      integralError_ = std::max(-speedILimit_, std::min(speedILimit_, integralError_));
//      throttle_ = speedKP_ * error + speedKI_ * integralError_ + speedKD_ * (error - previousError_);
//    } else {
//      throttle_ = speedKP_ * error + speedKD_ * (error - previousError_);
//    }
//

    std_msgs::Float32 msg1;
    msg1.data = speedFiltered();
    inter1Pub_.publish(msg1);
    std_msgs::Float32 msg;
    msg.data = curSpeed;
    interPub_.publish(msg);
  }
  throttle_ = std::max(-throttleMax_, std::min(throttleMax_, throttle_));

  lastPreviousError_ = previousError_;
  previousError_ = error;

  sendCommand(throttle_, setSteering_); // invalid steering
}

void SpeedController::cmdVelCallback(const geometry_msgs::TwistConstPtr &twist) {
  if (std::abs(setSpeed_ - twist->linear.x) >= 0.01) {
    ROS_INFO("SpeedController: new speed setPoint:%.2f", twist->linear.x);


    if(isStarting_ == 0 && setSpeed_ == 0.0 && twist->linear.x > 0.5) {
      isStarting_ = 1;
      ROS_WARN("starting mode");
    }

    lastSetSpeed_ = setSpeed_;
  }
  setSpeed_ = twist->linear.x;
  setSteering_ =  twist->angular.z / maxSteering_; // steerInterpolator_.interpolate(twist->angular.z);
  if(setSteering_ < -0.1) setSteering_ *= 1.5;
  // if(setSteering_ < 0.3 && setSteering_ > 0.1) setSteering_ = 0.3;
}

void SpeedController::sendCommand(double throttle, double steering) {
  racecar_msgs::ChassisCommand command;
  command.header.stamp = ros::Time::now();
  command.sender = "controller";
  command.throttle = std::min(std::max(throttle, -1.0), 1.0);
  command.steering = std::min(std::max(steering, -1.0), 1.0);
  commandPub_.publish(command);
}

void SpeedController::loadThrottleMapping() {
  XmlRpc::XmlRpcValue v;
  nodeHandle_.param("throttle_mapping", v, v);
  std::map<double, double> mapping;
  for(auto &mapIt : v)
  {
    if(mapIt.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::pair<double, double> toAdd(std::pair<double, double>(
          boost::lexical_cast<double>(mapIt.first),
          static_cast<double>(mapIt.second)));
      mapping[toAdd.first] = toAdd.second;
      ROS_INFO("SpeedController added to add mapping %0.2f : %0.2f", toAdd.first, toAdd.second);
    } else
    {
      ROS_ERROR("SpeedController: XmlRpc throttle calibration formatted incorrectly");
    }
  }
  interpolator_.setPoints(mapping);
}

void SpeedController::loadSteerMapping() {
  XmlRpc::XmlRpcValue v;
  nodeHandle_.param("steer_mapping", v, v);
  std::map<double, double> mapping;
  for(auto &mapIt : v)
  {
    if(mapIt.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::pair<double, double> toAdd(std::pair<double, double>(
          boost::lexical_cast<double>(mapIt.first),
          static_cast<double>(mapIt.second)));
      mapping[toAdd.first] = toAdd.second;
      ROS_INFO("Steer added to add mapping %0.2f : %0.2f", toAdd.first, toAdd.second);
    } else
    {
      ROS_ERROR("Steer: XmlRpc steering calibration formatted incorrectly");
    }
  }
  steerInterpolator_.setPoints(mapping);
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "speed_controller");
  racecar_control::SpeedController controller;
  controller.spin();
  return 0;
}