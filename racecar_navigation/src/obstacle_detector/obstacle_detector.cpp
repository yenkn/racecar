//
// Created by yenkn on 19-5-16.
//

#include "obstacle_detector/obstacle_detector.h"
#include <racecar_core/utils/tf.h>
#include <racecar_core/utils/laser.h>
#include <racecar_core/utils/std.h>
#include <racecar_core/utils/math.h>
#include <racecar_msgs/Obstacles.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/MarkerArray.h>
#include <assert.h>
#include <costmap_2d/cost_values.h>
#include <opencv2/opencv.hpp>
#include <racecar_core/utils/cv.h>

ObstacleDetector::ObstacleDetector(): privateNode_("~"), detector_("laser_line") {
  scanSub_ = privateNode_.subscribe("/scan", 1, &ObstacleDetector::scanCallback, this);

  scanPub_ = privateNode_.advertise<sensor_msgs::LaserScan>("/obstacle_scan", 1);
  obstaclePub_ = privateNode_.advertise<racecar_msgs::Obstacles>("obstacles", 1);
  obstacleEventPub_ = privateNode_.advertise<racecar_msgs::ObstacleEvent>("event", 100);
  markerPub_ = privateNode_.advertise<visualization_msgs::MarkerArray>("makers", 1);

  privateNode_.param("line_segment_distance", lineInSegmentDistance_, 0.2);
  privateNode_.param("cluster_tolerance", clusterTolerance_, 0.1);
  privateNode_.param("obstacle_movement_tolerance", obstacleMovementTolerance_, 1.0);
  privateNode_.param("minimal_survival", minimalSurvival_, 3);
  privateNode_.param("maximal_reserve", maximalReserve_, 10);
  privateNode_.param("minimal_points", minimalPoints_, 2);
  privateNode_.param("patch_size", patchSize_, 50);

  privateNode_.param("include_radius", includeRadius_, 0.10);
  privateNode_.param("exclude_radius", excludeRadius_, 0.60);
  privateNode_.param("distance_filter", distanceFilter_, 4.0);

  ROS_INFO("cluster_tolerance: %.2f", clusterTolerance_);
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScanConstPtr &scan) {
  if(scan_.header.seq == 0) {
    scan_ = *scan;
    return;
  }
  scan_ = *scan;

  tf::StampedTransform mapToCar;
  if(!utils::getTFTransform(tfListener_, "map", scan->header.frame_id, mapToCar)) {
    return;
  }
  mapToCar_ = mapToCar;

  auto lines = detector_.processScan(scan);

  sensor_msgs::LaserScan scanMsg;
  scanMsg = scan_;

  std::vector<Point2D> points;
  for(int i = 0; i < scan_.ranges.size(); i++) {
    if(scan_.ranges[i] < scan_.range_min || scan_.ranges[i] >= scan_.range_max) continue;
    auto angle = scan_.angle_min + i * scan_.angle_increment;
    auto point = Point2D::fromPoolarCoords(scan_.ranges[i], angle);

//    bool foundInLine = false;
//    for(auto &line : lines) {
//      if(utils::distaceToLineSegment(point, Point2D(line.x1, line.y1), Point2D(line.x2, line.y2)) < lineInSegmentDistance_) {
//        foundInLine = true;
//        break;
//      }
//    }

//  if(!foundInLine) points.push_back(point);
    points.push_back(point);
  }

  kdTree_.build(points);
  if(!points.empty()) {
    auto obstacles = clusterPoints();
    evolvePoints(obstacles);
    obstacles = getObstacles();
    publishObstacles(obstacles);

    for(auto &ob : obstacles) {
      if(ob.relativeMean.x < 0) {
        obstacles_[ob.id].valid = false;
        continue;
      }

      // find closest line
      Point2D leastPoint;
      double minDistance = INFINITY;
      for(auto &line : lines) {
        Point2D outPoint;
        double distance = utils::distaceToLineSegment(ob.relativeMean, Point2D(line.x1, line.y1), Point2D(line.x2, line.y2), &outPoint);
        if(distance < minDistance) {
          minDistance = distance;
          leastPoint = outPoint;
        }
      }

      double r = ob.relativeMean.length();
      double theta = ob.relativeMean.angleDeg();

      int side = ob.side; // -1 to left, 1 to right

      if(side == 0) {
        side = theta > 5 ? -1 : (theta < -5 ? 1 : 0);
        obstacles_[ob.id].side = side;
        // publishEvent(obstacles_[ob.id], ADD);
      } else {
        // publishEvent(obstacles_[ob.id], UPDATE);
      }

      if(side < 0) {
        int rangeIndex = int((ob.relativeMean.angle() - scanMsg.angle_min) / scanMsg.angle_increment);
        for(int i = 0; i < patchSize_; i+=2) {
          if(rangeIndex + i >= scanMsg.ranges.size()) break;
          scanMsg.ranges[rangeIndex + i] = (float)r;
        }
      } else if(side > 0) {
        int rangeIndex = int((ob.relativeMean.angle() + scanMsg.angle_max) / scanMsg.angle_increment);
        for(int i = 0; i < patchSize_; i+=2) {
          if(rangeIndex - i < 0) break;
          scanMsg.ranges[rangeIndex - i] = (float)r;
        }
      }
    }
  }

  scanPub_.publish(scanMsg);
}

bool ObstacleDetector::isFamiliarObstacle(const cluster_t &o1, const cluster_t &o2) {
  return o1.mean.distance(o2.mean) < obstacleMovementTolerance_;
}

bool ObstacleDetector::evolvePoints(std::vector<cluster_t> &newObstacles) {
  // new -> staged -> stable
  bool changed = false;

  if(obstacles_.empty()) {
    for(auto &o : newObstacles) {
      o.id = currentIncreamentId_++;
      obstacles_[o.id] = o;
    }
    return changed;
  }

  std::vector<int> surviveList(currentIncreamentId_, 0);

  for(auto &obstacle : newObstacles) {
    // find familiar obstacle
    bool foundFamiliar = false;

    // add stable obstacles survive counter
    for(auto &pair : obstacles_) {
      if(!pair.second.valid) continue;
      if(isFamiliarObstacle(pair.second, obstacle)) {
        pair.second.surviveCounter++;
        pair.second.mean = obstacle.mean;
        pair.second.relativeMean = obstacle.relativeMean;
        pair.second.distances = obstacle.distances;
        pair.second.stage = obstacle.stage;
        pair.second.dismissCounter = 0;
        surviveList[pair.first]++;

        if(!pair.second.stable && pair.second.surviveCounter == minimalSurvival_) {
          pair.second.stable = true;
          pair.second.surviveCounter = 0;
        } else {
          // update
        }
        foundFamiliar = true;
        break;
      }
    }

    if(!foundFamiliar) {
      // new obstacle
      obstacle.id = currentIncreamentId_++;
      obstacles_[obstacle.id] = obstacle;
    }
  }

  for(int i = 0; i < surviveList.size(); i++) {
    if(surviveList[i] == 0) {
      obstacles_[i].dismissCounter++;
      if(obstacles_[i].valid && obstacles_[i].dismissCounter == maximalReserve_) {
        obstacles_[i].valid = false;
      }
    }
  }

  return changed;
}

void ObstacleDetector::publishEvent(const cluster_t &cluster, int event) {
  racecar_msgs::ObstacleEvent msg;
  msg.obstacle.stage = cluster.side;
  msg.obstacle.mean = cluster.mean.toPoint();
  msg.obstacle.relativeMean = cluster.relativeMean.toPoint();
  msg.obstacle.forward = cluster.distances.forward;
  msg.obstacle.backward = cluster.distances.backward;
  msg.obstacle.leftward = cluster.distances.leftward;
  msg.obstacle.rightward = cluster.distances.rightward;
  msg.obstacle.id = cluster.id;
  msg.obstacle.survive = cluster.surviveCounter;

  if(event == ADD) {
    ROS_INFO("detected obstacle[%d]: %.2f, %.2f", cluster.id, cluster.mean.x, cluster.mean.y);
    msg.event = racecar_msgs::ObstacleEvent::ADD;
  } else if(event == UPDATE) {
    msg.event = racecar_msgs::ObstacleEvent::MODIFY;
  }

  obstacleEventPub_.publish(msg);
}

std::vector<cluster_t> ObstacleDetector::getObstacles() {
  std::vector<cluster_t> stableObstacles;
  for(const auto &pair : obstacles_) {
    if(pair.second.stable && pair.second.valid) {
      stableObstacles.push_back(pair.second);
    }
  }
  return stableObstacles;
}

void ObstacleDetector::publishObstacles(const std::vector<cluster_t> &obstacles) {
  visualization_msgs::MarkerArray arr;

  visualization_msgs::Marker msg;
  msg.header.frame_id = "map";
  msg.ns = "Markers";

  msg.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(msg);

  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;

  msg.type = visualization_msgs::Marker::CYLINDER;

  msg.scale.y = msg.scale.x = 0.31;
  msg.scale.z = 0.01;

  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  for(int i = 0; i < obstacles.size(); i++) {
    msg.id = i;
    msg.pose.position = obstacles[i].mean.toPoint();
    arr.markers.push_back(msg);
  }

  markerPub_.publish(arr);
}

std::vector<cluster_t> ObstacleDetector::clusterPoints() {
  auto clusters = kdTree_.cluster(clusterTolerance_, minimalPoints_);
  auto obstacles = std::vector<cluster_t>();
  for(auto &pts: clusters) {
    Point2D mean(0, 0);
    for(auto &pt : pts) {
      mean += pt / pts.size();
    }
    auto cpt = kdTree_.radiusSearch(mean, includeRadius_).size();
    if(cpt == pts.size() && cpt == kdTree_.radiusSearch(mean, excludeRadius_).size() && mean.length() < distanceFilter_) {
      cluster_t o;
      o.points = pts;
      auto mapMean = mapToCar_.inverse() * tf::Vector3(mean.x, mean.y, 0);
      o.relativeMean = mean;
      o.mean = Point2D(mapMean.x(), mapMean.y());
      obstacles.push_back(o);
    }
  }
  return obstacles;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_detector");
  ObstacleDetector detector;

  ros::spin();
  return 0;
}