//
// Created by yenkn on 19-8-11.
//

#ifndef PROJECT_GRAPH_DETECTOR_H
#define PROJECT_GRAPH_DETECTOR_H

#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <racecar_core/point.h>
#include <racecar_core/utils/kd_tree.h>
#include <racecar_core/utils/tf.h>
#include <racecar_core/utils/laser.h>
#include <racecar_core/utils/math.h>
#include <racecar_core/utils/graph.hpp>
#include <racecar_msgs/Obstacles.h>
#include <racecar_msgs/ObstacleEvent.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/MarkerArray.h>
#include "laser_line/laser_feature_ros.h"

struct cluster_t {
  int id;
  std::vector<Point2D> points;
  Point2D mean;
  Point2D relativeMean;
  int surviveCounter = 0;
  int dismissCounter = 0;
  bool stable = false;
  int stage = -1;
  int side = 0;
  utils::distance_t distances;
};

enum class EVENT_TYPE {
  ADD, REMOVE, UPDATE,
};

class GraphDetector {
public:
  GraphDetector();

  typedef std::function<void(cluster_t *, EVENT_TYPE)> callbackFunction;

  void setObstacleCallback(const callbackFunction &callback) {
    callback_ = callback;
  }

private:
  ros::NodeHandle privateNode_;
  ros::Publisher markerPub_, obstaclePub_, obstacleEventPub_, scanPub_;
  ros::Subscriber scanSub_;
  ros::ServiceClient stageClient_;
  tf::TransformListener tfListener_;

  callbackFunction callback_;

  tf::StampedTransform mapToCar_;
  sensor_msgs::LaserScan scan_;
  std::mutex scanLock_;

  utils::KDTree<Point2D> kdTree_;
  laser_line::LaserFeatureROS detector_;

  std::map<int, cluster_t> obstacles_;
  int currentIncreamentId_ = 1;

  double lineInSegmentDistance_;
  double clusterTolerance_, obstacleMovementTolerance_, detectPadding_;
  int minimalSurvival_, maximalReserve_;

  double includeRadius_, excludeRadius_, distanceFilter_, transferDistance_;
  int minPoints_, patchSize_;

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan);
  void publishObstacles(const std::vector<cluster_t> &obstacles);
  void publishEvent(const cluster_t &cluster, EVENT_TYPE event);

  bool isFamiliarObstacle(const cluster_t &o1, const cluster_t &o2);
  void filterObstacles(std::vector<cluster_t> &obstacles);
  std::vector<cluster_t> getObstacles();
  bool evolvePoints(std::vector<cluster_t> &newObstacles);
  std::vector<cluster_t> clusterPoints(utils::KDTree<Point2D> &tree);
};


#endif //PROJECT_GRAPH_DETECTOR_H
