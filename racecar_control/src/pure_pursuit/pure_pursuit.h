//
// Created by yenkn on 19-3-24.
//

#ifndef PROJECT_PURE_PURSUIT_H
#define PROJECT_PURE_PURSUIT_H

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
#include <racecar_control/pure_pursuitConfig.h>

class PurePursuit {
public:
  PurePursuit();

private:
  ros::NodeHandle nh_;
  ros::Subscriber odomSub_, pathSub_, goalSub_, amclSub_, obstacleSub_, collisionSub_;
  ros::Publisher cmdvelPub_, markerPub_, curvaturePub_;
  ros::Timer timer_;
  tf::TransformListener tfListener_;

  visualization_msgs::Marker points_, lineStrip_, goalCircle_;
  geometry_msgs::Point goal_;
  racecar_msgs::Obstacles obstacles;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseWithCovarianceStamped amclPose_;
  nav_msgs::Path mapPath_;

  double brakeGain_ = 1.0;
  tf::StampedTransform mapToCar_;

  double L_, Lfw_, Vcmd_, lfw_;
  double practiceLfw_, steering_, velocity_;
  double steeringGain_, baseAngle_, goalRadius_, speedIncremental_, predictTime_, minVelocity_;
  bool foundWaypoint_ = false, foundCar_ = false;
  int waypointIndex_ = 0, carIndex_ = 0;
  bool goalReceived_, goalReached_, debugMode_, smoothAccel_;

  void initMarker();
  double calculateSteering(const tf::Vector3 &waypoint);
  double getSteering();

  bool getNearestForwardObstacle(racecar_msgs::Obstacle &obstacle);
  tf::Vector3 getNextWaypoint();
  tf::Vector3 predictedCarPos(double dt);

  dynamic_reconfigure::Server<racecar_control::pure_pursuitConfig> dsrv_;

  void paramCallback(racecar_control::pure_pursuitConfig &config, uint32_t level);
  void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
  void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amclMsg);
  void obstacleCB(const racecar_msgs::ObstaclesConstPtr &msg);
  void collisionCB(const geometry_msgs::PointStampedConstPtr &pt);
  void controlLoopCB(const ros::TimerEvent &);

};

#endif //PROJECT_PURE_PURSUIT_H
