//
// Created by yenkn on 19-3-24.
//

#ifndef PROJECT_PD_CONTROLLER_H
#define PROJECT_PD_CONTROLLER_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <racecar_msgs/Obstacles.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <racecar_control/pd_controllerConfig.h>

class PDController {
public:
  PDController();
  void sendStop();


private:
  ros::NodeHandle nh_;
  ros::Subscriber odomSub_, pathSub_, goalSub_, amclSub_, obstacleSub_, collisionSub_;
  ros::Publisher cmdvelPub_, markerPub_, errorPub_;
  ros::Timer timer_;
  tf::TransformListener tfListener_;

  visualization_msgs::Marker points_, lineStrip_, goalCircle_, rangeLines_;
  geometry_msgs::Point goal_;
  racecar_msgs::Obstacles obstacles;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseWithCovarianceStamped amclPose_;
  nav_msgs::Path mapPath_;
  std::vector<double> pathCurvature_;

  double brakeGain_ = 1.0;
  tf::StampedTransform mapToCar_;

  double L_, lfw_;
  double pidP_, pidI_, pidD_;
  double errorDistance_, setSpeed_, previousError_;
  double errorLimit_;
  double steering_ = 0.0, velocity_ = 0.0;
  double steeringGain_, baseAngle_, goalRadius_, speedIncremental_, predictTime_, minVelocity_;
  bool foundWaypoint_ = false, foundCar_ = false;
  int waypointIndex_ = 0, carIndex_ = 0;
  bool goalReceived_, goalReached_, debugMode_, smoothAccel_;
  ros::Time startTime_;

  double maxLfw_ = 1.4, minLfw_ = 1.4, practiceLfw_ = 1.0;
  double maxSpeed_ = 2.0, minSpeed_ = 1.2;
  double distanceGain_ = 0.0;
  int stepSize_ = 20;

  void initMarker();
  double calculateSteering(const tf::Vector3 &waypoint);
  double getSteering();

  tf::Vector3 getNextWaypoint();
  dynamic_reconfigure::Server<racecar_control::pd_controllerConfig> dsrv_;

  void paramCallback(racecar_control::pd_controllerConfig &config, uint32_t level);
  void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
  void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclMsg);
  void obstacleCB(const racecar_msgs::ObstaclesConstPtr &msg);
  void controlLoopCB(const ros::TimerEvent &);

};

#endif //PROJECT_PD_CONTROLLER_H
