//
// Created by yenkn on 19-7-8.
//

#include "obstacle_detector/matching_detector.h"
#include <racecar_core/utils/tf.h>
#include <visualization_msgs/Marker.h>
#include <racecar_core/utils/color.h>
#include <nav_msgs/GridCells.h>
#include <std_srvs/Empty.h>
#include <racecar_core/utils/tf.h>

MatchingDetector::MatchingDetector(): node_("~") {
  scanSub_ = node_.subscribe("/scan", 1, &MatchingDetector::scanCallback, this);
  pathSub_ = node_.subscribe("/move_base/global_plan", 1, &MatchingDetector::pathCallback, this);

  markerPub_ = node_.advertise<visualization_msgs::Marker>("markers", 1);
  pathPub_ = node_.advertise<nav_msgs::Path>("/obstacle_plan", 1);

  clearClient_ = node_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

  node_.param("include_radius", includeRadius_, 0.10);
  node_.param("exclude_radius", excludeRadius_, 0.50);
  node_.param("min_points", minPoints_, 3);
  node_.param("consist_times", consistTimes_, 5);
  node_.param("distance_filter", distanceFilter_, 3.5);
  node_.param("path_interval", pathInterval_, 0.5);

}

void MatchingDetector::pathCallback(const nav_msgs::PathConstPtr &path) {
  if(pathLocked_) return;
  if(path->poses.empty()) {
    ROS_ERROR("No Path");
    return;
  }
  pathPub_.publish(*path);
//  if(path->poses.empty()) {
//    if((ros::Time::now() - lastPathReciveTime_).toSec() > pathInterval_) {
//      noPathFound_ = true;
//    }
//  } else {
//    lastPathReciveTime_ = ros::Time::now();
//    noPathFound_ = false;
//  }
//
//  if(pathLocked_) {
//    if(noPathFound_) {
//      ROS_ERROR("MEET OBSTACLE WITH NO PATH! clearing costmap.");
//      std_srvs::Empty srv;
//      clearClient_.call(srv);
//    } else if(!obstaclePathFound_) {
//      double score = scorePath(*path);
//      if(score < 100) {
//        obstaclePathFound_ = true;
//        pathPub_.publish(*path);
//      } else {
//        ROS_WARN("Path invalid, keep waiting...");
//      }
//    }
//  } else {
//    obstaclePathFound_ = false;
//    if(!path->poses.empty()) {
//      pathPub_.publish(*path);
//    }
//  }
}

void MatchingDetector::scanCallback(const sensor_msgs::LaserScanConstPtr &scan) {
  scan_ = *scan;
  std::vector<Point2D> points;

  for(int i = 0; i < scan->ranges.size(); i++) {
    if (scan->ranges[i] < scan->range_min || scan->ranges[i] >= scan->range_max) continue;
    auto angle = scan->angle_min + i * scan->angle_increment;
    auto point = utils::polarToCartesian(scan->ranges[i], angle);
    if (point.length() < 0.1) continue; // self
    points.emplace_back(point.x(), point.y());
  }

  tree_.build(points);

  auto clusters = tree_.cluster(0.06, 3);

  obstacles_ = std::vector<obstacle>();
  for(auto &pts: clusters) {
    Point2D mean(0, 0);
    for(auto &pt : pts) {
      mean += pt / pts.size();
    }
    if(tree_.radiusSearch(mean, includeRadius_).size() == tree_.radiusSearch(mean, excludeRadius_).size() && mean.length() < distanceFilter_ && mean.x > 0) {
      obstacle o;
      o.points = pts;
      obstacles_.push_back(o);

      if(!pathLocked_) {
        std::cout << "obstacle found at " << mean.x << ", " << mean.y << ": lock" << std::endl;
        pathLocked_ = true;
        disappearTimes_ = consistTimes_;
      }
    }
  }

  if(obstacles_.empty() && pathLocked_) {
    if(disappearTimes_-- <= 0) {
      std::cout << "no obstacle: unlock" << std::endl;
      pathLocked_ = false;
    }
  }

  publishMarkers();
}
//
//double MatchingDetector::scorePath(const nav_msgs::Path &path) {
//  if(obstacles_.empty()) return 0.0;
//  double obstacleDistance = obstacles_[0].mean.length();
//
//  tf::StampedTransform pathToCostmap, pathToCar;
//
//  if(!utils::getTFTransform(tf_, path.header.frame_id, "base_footprint", pathToCar)) {
//    ROS_ERROR("can't find path to car transform");
//    return std::numeric_limits<double>::max();
//  }
//
//  double distance = 0.0;
//  for(int i = 1; i < path.poses.size(); i++) {
//    double obDistance = hypot(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y, path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
//    distance += hypot(path.poses[i].pose.position.y - path.poses[0].pose.position.y, path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
//    if(distance >= obstacleDistance) {
//      auto waypoint = pathToCar * tf::Vector3(path.poses[i].pose.position.x, path.poses[i].pose.position.y, 0);
//      return waypoint.length();
//    }
//  }
//
//}

//std::vector<Point2D> MatchingDetector::templateMatching() {
//  auto &points = tree_.points();
//  std::vector<Point2D> matches;
//  std::set<int> processed;
//
//  for(int i = 0; i < points.size(); i++) {
//    if(processed.find(i) != processed.end()) continue;
//
//    auto indices = tree_.radiusSearch(points[i], includeRadius_);
//    if(indices.size() >= minPoints_) {
//      auto excludeIndices = tree_.radiusSearch(points[i], excludeRadius_);
//      if(excludeIndices.size() == indices.size()) {
//        // template matched
//        processed.insert(indices.cbegin(), indices.cend());
//        matches.push_back(points[i]);
//      }
//    }
//  }
//  return matches;
//}

void MatchingDetector::publishMarkers() {
  visualization_msgs::Marker msg, circle;
  msg.header = scan_.header;
  msg.action = visualization_msgs::Marker::ADD;
  msg.ns = "Markers";
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;

  msg.scale.x = 0.05;
  msg.scale.y = 0.05;

  for(const obstacle &o : obstacles_) {
    auto color = utils::Color::fromHSV(rand() % 360, 1, 1).toColorRGBA();
    for(const Point2D &pt : o.points) {
      msg.points.push_back(pt.toPoint());
      msg.colors.push_back(color);
    }
  }

  markerPub_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "matching_detector");
  MatchingDetector detector;
  ros::spin();
  return 0;
}