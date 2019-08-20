//
// Created by yenkn on 19-7-8.
//

#ifndef SRC_MATCHING_DETECTOR_H
#define SRC_MATCHING_DETECTOR_H

#include <ros/ros.h>
#include <racecar_core/point.h>
#include <racecar_core/utils/kd_tree.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

typedef struct {
    Point2D mean;
    std::vector<Point2D> points;
} obstacle;

typedef struct {
    Point2D point;
    int validate = 0;
} match;

class MatchingDetector {
public:
  MatchingDetector();

private:
  ros::Subscriber scanSub_, pathSub_, costmapSub_;
  ros::Publisher markerPub_, pathPub_;
  ros::ServiceClient clearClient_;
  sensor_msgs::LaserScan scan_;
  nav_msgs::OccupancyGrid costmap_;
  tf::TransformListener tf_;

  utils::KDTree<Point2D> tree_;
  ros::NodeHandle node_;

  std::vector<obstacle> obstacles_;

  double includeRadius_, excludeRadius_, distanceFilter_, pathInterval_;
  int minPoints_;
  int disappearTimes_, consistTimes_;
  bool pathLocked_, noPathFound_, obstaclePathFound_;
  ros::Time lastPathReciveTime_;

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan);
  void pathCallback(const nav_msgs::PathConstPtr &path);
  void costmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap);

  double scorePath(const nav_msgs::Path &path);
  void publishMarkers();

};

#endif //SRC_MATCHING_DETECTOR_H
