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

#include "pure_pursuit.h"
#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <racecar_core/utils/tf.h>

PurePursuit::PurePursuit(): Lfw_(1.2), Vcmd_(1.5), steeringGain_(0.8) {
  //Private parameters handler
  ros::NodeHandle pn("~");

  //Car parameter
  pn.param("L", L_, 0.47); // length of car
  pn.param("lfw", lfw_, 0.1675); // distance between front the center of car

  int controllerFreq;
  //Controller parameter
  pn.param("controller_freq", controllerFreq, 20);
  pn.param("goal_radius", goalRadius_, 0.5); // goal radius (m)
  pn.param("base_angle", baseAngle_, 0.0); // neutral point of servo (rad)
  pn.param("debug_mode", debugMode_, false); // debug mode
  pn.param("smooth_accel", smoothAccel_, true); // smooth the acceleration of car
  pn.param("speed_incremental", speedIncremental_, 0.05); // speed incremental value (discrete acceleraton), unit: m/s
  pn.param("predict_time", predictTime_, 0.0); // predict time
  pn.param("min_velocity", minVelocity_, 0.5); // minimal velocity

  dsrv_.setCallback(std::bind(&PurePursuit::paramCallback, this, std::placeholders::_1, std::placeholders::_2));

  //Publishers and Subscribers
  odomSub_ = nh_.subscribe("/pure_pursuit/odom", 1, &PurePursuit::odomCB, this);
  pathSub_ = nh_.subscribe("/pure_pursuit/global_plan", 1, &PurePursuit::pathCB, this);
  goalSub_ = nh_.subscribe("/move_base_simple/goal", 1, &PurePursuit::goalCB, this);
  amclSub_ = nh_.subscribe("/pure_pursuit/pose", 5, &PurePursuit::amclCB, this);
  obstacleSub_ = nh_.subscribe("/navigator/obstacle_detector/obstacles", 1, &PurePursuit::obstacleCB, this);
  collisionSub_ = nh_.subscribe("/navigator/collision", 1, &PurePursuit::collisionCB, this);
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("/pure_pursuit/path_marker", 10);
  cmdvelPub_ = nh_.advertise<geometry_msgs::Twist>("/pure_pursuit/cmd_vel", 1);
  curvaturePub_ = nh_.advertise<std_msgs::Float32>("/pure_pursuit/yaw", 1);

  //Timer
  timer_ = nh_.createTimer(ros::Rate(controllerFreq), &PurePursuit::controlLoopCB, this); // Duration(0.05) -> 20Hz

  //Init variables
  goalReceived_ = false;
  goalReached_ = false;

  //Show info
  ROS_INFO("[param] base_angle: %f", baseAngle_);

  //Visualization Marker Settings
  initMarker();
}


void PurePursuit::initMarker() {
  points_.header.frame_id = lineStrip_.header.frame_id = goalCircle_.header.frame_id = "map";
  points_.ns = lineStrip_.ns = goalCircle_.ns = "Markers";
  points_.action = lineStrip_.action = goalCircle_.action = visualization_msgs::Marker::ADD;
  points_.pose.orientation.w = lineStrip_.pose.orientation.w = goalCircle_.pose.orientation.w = 1.0;
  points_.id = 0;
  lineStrip_.id = 1;
  goalCircle_.id = 2;

  points_.type = visualization_msgs::Marker::POINTS;
  lineStrip_.type = visualization_msgs::Marker::LINE_STRIP;
  goalCircle_.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points_.scale.x = 0.2;
  points_.scale.y = 0.2;

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

void PurePursuit::paramCallback(racecar_control::pure_pursuitConfig &config, uint32_t level) {
  Vcmd_ = config.desire_velocity;
  steeringGain_ = config.steering_gain;
  Lfw_ = config.forward_distance;
  ROS_INFO("get param: v: %.2f, gain: %.2f, Lfw: %.2f", Vcmd_, steeringGain_, Lfw_);
}

void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  odom = *odomMsg;

  utils::getTFTransform(tfListener_, "map", "base_footprint", mapToCar_);
}

void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr &pathMsg) {
  mapPath_ = *pathMsg;
  carIndex_ = 0;
  waypointIndex_ = 0;
}

void PurePursuit::obstacleCB(const racecar_msgs::ObstaclesConstPtr &msg) {
  obstacles = *msg;
}

// https://www.desmos.com/calculator/unx47uf1pr
inline double sigmoid_brake(double x, double brake_distance, double w = 4.0) {
  return -1/(1+exp(w - 2*w*x/brake_distance)) + 1;
}

void PurePursuit::collisionCB(const geometry_msgs::PointStampedConstPtr &pt) {
  tf::Point tfPoint;
  tf::pointMsgToTF(pt->point, tfPoint);
  auto collision = mapToCar_ * tfPoint;
  double collisionDistance = collision.length();
  if(collisionDistance == 0) return;
  brakeGain_= sigmoid_brake(collisionDistance, 5);
  // std::cout << collisionDistance << ", " << brakeGain_ << ", " << velocity << std::endl;
}

tf::Vector3 PurePursuit::predictedCarPos(double dt) {
  auto carPose = amclPose_.pose.pose.position;
  double yaw = tf::getYaw(amclPose_.pose.pose.orientation);
  double v = odom.twist.twist.linear.x;

  return {
    carPose.x + v * cos(yaw) * dt,
    carPose.y + v * sin(yaw) * dt,
    carPose.z + v * steering_ * dt / lfw_, // phi
  };
}
/*
// * Finds the center of the circle passing through the points p1, p2 and p3.
// * (NaN, NaN) will be returned if the points are colinear.
tf::Vector3 CircleCenterFrom3Points (tf::Vector3 p1, tf::Vector3 p2, tf::Vector3 p3) {
  double temp = p2.length2();
  double bc = (p1.length2() - temp)/2.0f;
  double cd = (temp - p3.length2())/2.0f;
  double det = (p1.x()-p2.x())*(p2.y()-p3.y())-(p2.x()-p3.x())*(p1.y()-p2.y());
//  if (fabs(det) < 1.0e-6) {
//    return { NAN, NAN, 0 };
//  }
  det = 1/det;
  return { (bc*(p2.y()-p3.y())-cd*(p1.y()-p2.y()))*det, ((p1.x()-p2.x())*cd-(p2.x()-p3.x())*bc)*det, 0 };
}

void PurePursuit::calculateWaypointCurvature() {
  if(!foundCar_ || !foundWaypoint_) return;

  tf::StampedTransform mapToCar;
  if(!utils::getTFTransform(tfListener_, "map", "base_footprint", mapToCar)) {
    return;
  }

  int end = std::min(carIndex_ + 50, (int)mapPath_.poses.size()-1);
  std::vector<tf::Vector3> path;
  auto step_size = (int)floor(double(end - carIndex_) / 10);
  if(step_size < 1) return;
  for(int i = carIndex_; i < end; i++) {
    if(i%step_size == 0) {
      auto pt = mapPath_.poses[i].pose.position;
      path.push_back(mapToCar * tf::Vector3(pt.x, pt.y, 0));
    }
  }
  if(path.size() < 2) return;
  path_curvatures.resize(path.size()-2);
  std::fill(path_curvatures.begin(), path_curvatures.end(), 0);
  for(int i = 1; i < path.size()-1; i++) {
    auto cc = CircleCenterFrom3Points(path[i-1], path[i], path[i+1]);
    auto radius = (cc - path[i]).length();
    radius = std::max(radius, 0.0001);
    auto curvature = 1 / radius;
    path_curvatures[i-1] = curvature;
  }
  double mean = std::accumulate(path_curvatures.begin(), path_curvatures.end(), 0.0) / path_curvatures.size();
  double total = 0.0;
  for(auto c : path_curvatures) {
    total += pow(c-mean, 2);
  }
  double deviation = total / (path_curvatures.size()-1);
  auto pMax = std::max_element(path_curvatures.begin(), path_curvatures.end());
  auto maxIndex = std::distance(path_curvatures.begin(), pMax);
  brakeGain_ = (10 - maxIndex) * *pMax + deviation * 5 + mean;

  // std::cout << "Mean: " << mean << ", Deviation: " << deviation << ", Max[" << maxIndex << "]: " << *pMax << " = " << brakeGain_ << std::endl;

//  std_msgs::Float32 msg;
//  msg.data = (float)brakeGain_;
//  curvaturePub_.publish(msg);
}
*/

void PurePursuit::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
  goal_ = goalMsg->pose.position;
  goalReceived_ = true;
  goalReached_ = false;

  /*Draw Goal on RVIZ*/
  goalCircle_.pose = goalMsg->pose;
  markerPub_.publish(goalCircle_);
}

bool PurePursuit::getNearestForwardObstacle(racecar_msgs::Obstacle &obstacle) {
  practiceLfw_ = Lfw_;
//  bool foundObstacle = false;
//  for(auto &item: obstacles.obstacles) {
//    auto obstacleInCar = mapToCar_ * utils::pointToVector(item.mean);
//    if(obstacleInCar.x() > 0) {
//      auto len = obstacleInCar.length();
//      if(len < practiceLfw_) {
//        practiceLfw_ = (0.6*len + 0.4 * Lfw_);
//        obstacle = item;
//        foundObstacle = true;
//      }
//    }
//  }
  return false;
}

tf::Vector3 PurePursuit::getNextWaypoint() {
  racecar_msgs::Obstacle obstacle;
  getNearestForwardObstacle(obstacle);
  // calculateWaypointCurvature();
  // practiceLfw_ = Lfw_ * sigmoid(brakeGain_) * 1.5; // 0 to 0.75

  auto carPose = predictedCarPos(predictTime_);
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

    for (int i = carIndex_; i < mapPath_.poses.size(); i++) {
      geometry_msgs::PoseStamped mapPose = mapPath_.poses[i];

      waypoint = utils::pointToVector(mapPose.pose.position);
      waypointInCar = mapToCar_ * waypoint;

      if (waypoint.distance(carPose) >= practiceLfw_) {
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

  if(foundWaypoint_) {
    std_msgs::Float32 msg;
    msg.data = (float)brakeGain_;
    curvaturePub_.publish(msg);
  }

  return waypointInCar;
}

double PurePursuit::calculateSteering(const tf::Vector3 &waypoint) {
  /*Estimate Steering Angle*/
  double eta = atan2(waypoint.y(), waypoint.x());
  return atan((L_ * sin(eta))/(practiceLfw_ / 2 + lfw_ * cos(eta)));
}

double PurePursuit::getSteering() {
  return calculateSteering(mapToCar_ * utils::pointToVector(mapPath_.poses[waypointIndex_].pose.position));
//  const int interpolateCount = 5;
//  int stepSize = floor(double(waypointIndex_ - carIndex_) / interpolateCount);
//  if(stepSize < 1) {
//    return calculateSteering(mapToCar * utils::pointToVector(mapPath_.poses[waypointIndex_].pose.position));
//  }
//  double totalSteering = 0.0;
//  for(int i = 1; i <= interpolateCount; i++) {
//    totalSteering += calculateSteering(mapToCar * utils::pointToVector(mapPath_.poses[carIndex_+stepSize*i].pose.position));
//  }
//  return totalSteering / (((double)interpolateCount+1)/2);
}


void PurePursuit::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclMsg) {
  amclPose_ = *amclMsg;
  if (goalReceived_) {
    double car2goal_x = goal_.x - amclMsg->pose.pose.position.x;
    double car2goal_y = goal_.y - amclMsg->pose.pose.position.y;
    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
    if (dist2goal < goalRadius_) {
      goalReached_ = true;
      goalReceived_ = false;
      ROS_INFO("Goal Reached !");
    }
  }
}


void PurePursuit::controlLoopCB(const ros::TimerEvent &) {

  geometry_msgs::Twist carVel = odom.twist.twist;

  if (goalReceived_) {
    getNextWaypoint();
    if(foundWaypoint_) {
      auto steer = getSteering();
      steering_ = baseAngle_ + steer * steeringGain_;
      /*Estimate Gas Input*/
      if (!goalReached_) {
        double vel = Vcmd_ <= minVelocity_ ? Vcmd_ : ((Vcmd_ - minVelocity_) * (1-brakeGain_) +
                                                                minVelocity_);
        velocity_ = smoothAccel_ ? std::min(velocity_ + speedIncremental_, vel) : vel;
        if (debugMode_) printf("Velocity = %.2f, Steering = %.2f\n", velocity_, steering_);
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
  ros::init(argc, argv, "pure_pursuit");
  PurePursuit controller;
  ros::AsyncSpinner spinner(2); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
