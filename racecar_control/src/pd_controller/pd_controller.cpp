/*
# Copyright 2018 HyphaROS Workshop.
# Latest Modifier: HaoChih, LIN (hypha.ros@gmail.com)
# Original Author: ChanYuan KUO & YoRu LU
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include "pd_controller.h"
#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <racecar_core/utils/tf.h>
#include <racecar_core/utils/math.h>
#include <racecar_core/utils/color.h>

PDController::PDController(): steeringGain_(1.0) {
  //Private parameters handler
  ros::NodeHandle pn("~");

  int controllerFreq;
  //Controller parameter
  pn.param("pid_p", pidP_, 1.0);
  pn.param("pid_i", pidI_, 0.0);
  pn.param("pid_d", pidD_, 0.1);

  pn.param("set_speed", setSpeed_, 1.0);
  pn.param("error_distance", errorDistance_, 1.0);
  pn.param("L", L_, 0.47); // length of car
  pn.param("lfw", lfw_, 0.1675); // distance between front the center of car

  pn.param("controller_freq", controllerFreq, 20);
  pn.param("goal_radius", goalRadius_, 0.5); // goal radius (m)
  pn.param("base_angle", baseAngle_, 0.0); // neutral point of servo (rad)
  pn.param("smooth_accel", smoothAccel_, true); // smooth the acceleration of car
  pn.param("speed_incremental", speedIncremental_, 0.05); // speed incremental value (discrete acceleraton), unit: m/s
  pn.param("error_limit", errorLimit_, 0.1);

  //Publishers and Subscribers
  odomSub_ = nh_.subscribe("/odometry/filtered", 1, &PDController::odomCB, this);
  pathSub_ = nh_.subscribe("/scan_planner/plan", 1, &PDController::pathCB, this);
  goalSub_ = nh_.subscribe("/move_base_simple/goal", 1, &PDController::goalCB, this);
  amclSub_ = nh_.subscribe("/amcl_pose", 5, &PDController::amclCB, this);
  obstacleSub_ = nh_.subscribe("/navigator/obstacle_detector/obstacles", 1, &PDController::obstacleCB, this);
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("/pd_controller/path_marker", 10);
  cmdvelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  errorPub_ = nh_.advertise<std_msgs::Float32>("error", 1);

  dsrv_.setCallback(std::bind(&PDController::paramCallback, this, std::placeholders::_1, std::placeholders::_2));

  //Timer
  timer_ = nh_.createTimer(ros::Rate(controllerFreq), &PDController::controlLoopCB, this); // Duration(0.05) -> 20Hz

  //Init variables
  goalReceived_ = false;
  goalReached_ = false;

  //Show info
  ROS_INFO("[param] P: %f", pidP_);
  ROS_INFO("[param] I: %f", pidI_);
  ROS_INFO("[param] D: %f", pidD_);

  //Visualization Marker Settings
  initMarker();
}


void PDController::initMarker() {
  points_.header.frame_id = lineStrip_.header.frame_id = rangeLines_.header.frame_id = goalCircle_.header.frame_id = "map";
  points_.ns = lineStrip_.ns = rangeLines_.ns = goalCircle_.ns = "Markers";
  points_.action = lineStrip_.action = rangeLines_.action = goalCircle_.action = visualization_msgs::Marker::ADD;
  points_.pose.orientation.w = lineStrip_.pose.orientation.w = rangeLines_.pose.orientation.w = goalCircle_.pose.orientation.w = 1.0;
  points_.id = 0;
  lineStrip_.id = 1;
  goalCircle_.id = 2;
  rangeLines_.id = 3;

  points_.type = visualization_msgs::Marker::POINTS;
  lineStrip_.type = visualization_msgs::Marker::LINE_STRIP;
  goalCircle_.type = visualization_msgs::Marker::CYLINDER;
  rangeLines_.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  points_.scale.x = 0.2;
  points_.scale.y = 0.2;

  rangeLines_.scale.x = rangeLines_.scale.y = 0.05;

  //LINE_STRIP markers use only the x component of scale, for the line width
  lineStrip_.scale.x = 0.1;

  goalCircle_.scale.x = goalRadius_;
  goalCircle_.scale.y = goalRadius_;
  goalCircle_.scale.z = 0.1;

  // Points are green
  points_.color.g = 1.0f;
  points_.color.a = 1.0;

  // Line strip is blue
  lineStrip_.color.b = 1.0;
  lineStrip_.color.a = 1.0;

  //goalCircle_ is yellow
  goalCircle_.color.r = 1.0;
  goalCircle_.color.g = 1.0;
  goalCircle_.color.b = 0.0;
  goalCircle_.color.a = 0.5;
}

void PDController::paramCallback(racecar_control::pd_controllerConfig &config, uint32_t level) {
  setSpeed_ = config.desire_velocity;
  errorDistance_ = config.error_distance;

  if(config.pid_p > 0.0) {
    pidP_ = config.pid_p;
    ROS_INFO("new p params: %.4f", pidP_);
  }

  if(config.pid_d > 0.0) {
    pidD_ = config.pid_d;
    ROS_INFO("new d params: %.4f", pidD_);
  }

  ROS_INFO("new params, v: %.2f, %.2f", setSpeed_, errorDistance_);
}

void PDController::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  odom = *odomMsg;

  utils::getTFTransform(tfListener_, "map", "base_footprint", mapToCar_);
}

void PDController::pathCB(const nav_msgs::Path::ConstPtr &pathMsg) {
  if(pathMsg->poses.empty()) return;
  mapPath_ = *pathMsg;

  carIndex_ = 0;
  waypointIndex_ = 0;

  return;

  pathCurvature_ = std::vector<double>(mapPath_.poses.size(), 0.0);
  for(int i = stepSize_; i < (int)mapPath_.poses.size() - stepSize_; i++) {
    double val = std::min(utils::calculatePointCurvature(mapPath_, i, stepSize_) / 2, 1.0);
    pathCurvature_[i] = val;
  }

  // padding
  for(int i = 0; i < stepSize_; i++) {
    pathCurvature_[i] = pathCurvature_[stepSize_];
    pathCurvature_[mapPath_.poses.size() - i - 1] = pathCurvature_[mapPath_.poses.size() - stepSize_ - 1];
  }

  // conv1d
  pathCurvature_ = utils::maximumFilter(pathCurvature_, 60);

  for(int i = 0; i < pathCurvature_.size(); i++) {
    pathCurvature_[i] = std::min(pathCurvature_[i] * 1.5, 1.0);
  }

  rangeLines_.points.clear();
  rangeLines_.colors.clear();
  for(int i = 0; i < mapPath_.poses.size(); i+=5) {
    rangeLines_.points.push_back(mapPath_.poses[i].pose.position);
    double val = pathCurvature_[i];
    rangeLines_.colors.push_back(utils::Color::fromRGB(val, val, val).toColorRGBA());
  }
  markerPub_.publish(rangeLines_);

}

void PDController::obstacleCB(const racecar_msgs::ObstaclesConstPtr &msg) {
  obstacles = *msg;
}

void PDController::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
  goal_ = goalMsg->pose.position;
  goalReceived_ = true;
  goalReached_ = false;

  startTime_ = ros::Time::now();

  /*Draw Goal on RVIZ*/
  goalCircle_.pose = goalMsg->pose;
  markerPub_.publish(goalCircle_);
}

tf::Vector3 PDController::getNextWaypoint() {
  // calculateWaypointCurvature();
  // practiceLfw_ = Lfw_ * sigmoid(brakeGain_) * 1.5; // 0 to 0.75

  auto carPose = mapToCar_.inverse().getOrigin();
  tf::Vector3 waypoint, waypointInCar, carPoint;
  foundCar_ = foundWaypoint_ = false;

  if (!goalReached_) {
    int startIndex = std::max(carIndex_ - 10, 0);
    double minDistance = FLT_MAX;
    for (int i = startIndex; i < mapPath_.poses.size(); i++) {
      auto mapPose = utils::pointToVector(mapPath_.poses[i].pose.position);
      auto distance = carPose.distance(mapPose);
      if(distance < minDistance) {
        minDistance = distance;
        foundCar_ = true;
        carIndex_ = i;
        carPoint = mapPose;
      }
    }

//    if(carIndex_ + 20 < pathCurvature_.size()) {
//      double val = pathCurvature_[carIndex_ + 20];
//      practiceLfw_ = minLfw_ + (maxLfw_ - minLfw_) * (1 - val);
//    } else {
//      practiceLfw_ = minLfw_;
//    }
    practiceLfw_ = errorDistance_;

    for (int i = carIndex_; i < mapPath_.poses.size(); i++) {
      geometry_msgs::PoseStamped mapPose = mapPath_.poses[i];

      waypoint = utils::pointToVector(mapPose.pose.position);
      waypointInCar = mapToCar_ * waypoint;

      double distance = waypointInCar.length();
      if (distance >= practiceLfw_) {
        waypointIndex_ = i;
        foundWaypoint_ = true;
        break;
      }
    }

  } else if (goalReached_) {
    waypoint = utils::pointToVector(goal_);
    waypointInCar = mapToCar_ * waypoint;
    foundWaypoint_ = false;
    //ROS_INFO("goal REACHED!");
  }

  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points_.points.clear();
  lineStrip_.points.clear();

  if (foundWaypoint_ && !goalReached_) {
    points_.points.push_back(utils::vectorToPoint(carPose));
    points_.points.push_back(utils::vectorToPoint(carPoint));
    points_.points.push_back(utils::vectorToPoint(waypoint));
    lineStrip_.points.push_back(utils::vectorToPoint(carPose));
    lineStrip_.points.push_back(utils::vectorToPoint(waypoint));
  }

  markerPub_.publish(points_);
  markerPub_.publish(lineStrip_);

  return waypointInCar;
}

double PDController::calculateSteering(const tf::Vector3 &waypoint) {
  /*Estimate Steering Angle*/
  double eta = atan2(waypoint.y(), waypoint.x());
  double error = atan((L_ * sin(eta))/(practiceLfw_ / 2 + lfw_ * cos(eta)));
  // double error = eta;
  double dError = error - previousError_;
  if(errorLimit_ > 0.0) {
    dError = std::min(std::max(dError, -errorLimit_), errorLimit_); // limit
    error = previousError_ + dError;
  }
  double steering = pidP_ * error + pidD_ * dError;
  previousError_ = error;

  std_msgs::Float32 msg;
  msg.data = error;
  errorPub_.publish(msg);
  return steering;
}

double PDController::getSteering() {
  return calculateSteering(mapToCar_ * utils::pointToVector(mapPath_.poses[waypointIndex_].pose.position));
}


void PDController::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclMsg) {
  amclPose_ = *amclMsg;
  if (goalReceived_) {
    double car2goal_x = goal_.x - amclMsg->pose.pose.position.x;
    double car2goal_y = goal_.y - amclMsg->pose.pose.position.y;
    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
    if (dist2goal < goalRadius_) {
      goalReached_ = true;
      goalReceived_ = false;
      ROS_INFO("Goal Reached! time: %.2fs", (ros::Time::now() - startTime_).toSec());
    }
  }
}

void PDController::sendStop() {
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = 0;
  cmdVel.angular.z = 0;
  cmdvelPub_.publish(cmdVel);
}

void PDController::controlLoopCB(const ros::TimerEvent &) {

  geometry_msgs::Twist carVel = odom.twist.twist;

  if (goalReceived_) {
    getNextWaypoint();
    if(foundWaypoint_) {
      auto steer = getSteering();
      steering_ = baseAngle_ + steer * steeringGain_;

      /*Estimate Gas Input*/
      if (!goalReached_) {
        velocity_ = smoothAccel_ ? std::min(velocity_ + speedIncremental_, setSpeed_) : setSpeed_;
      }
    }
  }

  if (goalReached_) {
    if (carVel.linear.x > 0.1) {
      velocity_ = -1.0; // run backward to brake
    } else {
      velocity_ = 0.0;
    }
    steering_ = baseAngle_;
  }

  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = velocity_;
  cmdVel.angular.z = steering_;
  cmdvelPub_.publish(cmdVel);
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv) {
  //Initiate ROS
  ros::init(argc, argv, "pd_controller");
  PDController controller;
  ros::spin();
  return 0;
}
