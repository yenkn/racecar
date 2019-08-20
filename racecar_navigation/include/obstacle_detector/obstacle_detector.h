//
// Created by yenkn on 19-5-16.
//

#ifndef PROJECT_OBSTACLE_DETECTOR_NODE_H
#define PROJECT_OBSTACLE_DETECTOR_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <racecar_core/point.h>
#include <racecar_core/utils/kd_tree.h>
#include <racecar_core/utils/graph.hpp>
#include "laser_line/laser_feature_ros.h"
#include <racecar_msgs/ObstacleEvent.h>

struct cluster_t {
  int id;
  std::vector<Point2D> points;
  Point2D mean;
  Point2D relativeMean;
  int surviveCounter = 0;
  int dismissCounter = 0;
  bool stable = false;
  bool valid = true;
  int stage = -1;
  int side = 0;
  utils::distance_t distances;
};

class ObstacleDetector {
public:
  ObstacleDetector();

  static const int ADD = 1;
  static const int UPDATE = 2;
  static const int REMOVE = 3;

  std::vector<cluster_t> getObstacles();

private:
  ros::NodeHandle privateNode_;
  ros::Publisher markerPub_, obstaclePub_, scanPub_, obstacleEventPub_;
  ros::Subscriber scanSub_;
  tf::TransformListener tfListener_;

  tf::StampedTransform mapToCar_;
  sensor_msgs::LaserScan scan_;

  utils::KDTree<Point2D> kdTree_;
  laser_line::LaserFeatureROS detector_;

  std::map<int, cluster_t> obstacles_;
  int currentIncreamentId_ = 0;

  double lineInSegmentDistance_;
  double clusterTolerance_, obstacleMovementTolerance_;
  int minimalSurvival_, maximalReserve_, minimalPoints_, patchSize_;

  double includeRadius_, excludeRadius_, distanceFilter_;

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan);
  bool evolvePoints(std::vector<cluster_t> &newObstacles);
  bool isFamiliarObstacle(const cluster_t &o1, const cluster_t &o2);
  void publishObstacles(const std::vector<cluster_t> &obstacles);
  void publishEvent(const cluster_t &cluster, int event);
  std::vector<cluster_t> clusterPoints();
};

#endif //PROJECT_OBSTACLE_DETECTOR_NODE_H
