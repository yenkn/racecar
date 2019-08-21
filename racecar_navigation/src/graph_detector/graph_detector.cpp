//
// Created by yenkn on 19-8-11.
//

#include "graph_detector.h"
#include <racecar_msgs/QueryStage.h>

GraphDetector::GraphDetector(): privateNode_("~"), detector_("laser_line") {
  scanSub_ = privateNode_.subscribe("/scan", 10, &GraphDetector::scanCallback, this);

  obstaclePub_ = privateNode_.advertise<racecar_msgs::Obstacles>("obstacles", 1);
  obstacleEventPub_ = privateNode_.advertise<racecar_msgs::ObstacleEvent>("event", 100);
  markerPub_ = privateNode_.advertise<visualization_msgs::MarkerArray>("makers", 1);

  stageClient_ = privateNode_.serviceClient<racecar_msgs::QueryStage>("/query_stage");

  privateNode_.param("line_segment_distance", lineInSegmentDistance_, 0.1);
  privateNode_.param("cluster_tolerance", clusterTolerance_, 0.10); // 0.1
  privateNode_.param("obstacle_movement_tolerance", obstacleMovementTolerance_, 1.0);
  privateNode_.param("minimal_survival", minimalSurvival_, 2);
  privateNode_.param("maximal_reserve", maximalReserve_, 5);
  privateNode_.param("detect_padding", detectPadding_, 0.2);

  privateNode_.param("include_radius", includeRadius_, 0.10); // 0.1
  privateNode_.param("exclude_radius", excludeRadius_, 0.40); // 0.5
  privateNode_.param("min_points", minPoints_, 2);
  privateNode_.param("distance_filter", distanceFilter_, 6.0);
  privateNode_.param("transfer_distance", transferDistance_, 0.2);

  ROS_INFO("cluster_tolerance: %.4f", clusterTolerance_);
}

void GraphDetector::scanCallback(const sensor_msgs::LaserScanConstPtr &scan) {
  // if(!mapRecived_) return;
  if(scan_.header.seq == 0) {
    scan_ = *scan;
    return;
  }

  tf::StampedTransform mapToCar;
  if(!utils::getTFTransform(tfListener_, "map", scan->header.frame_id, mapToCar)) {
    return;
  }
  mapToCar_ = mapToCar;

  scan_ = *scan;

  auto lines = detector_.processScan(scan);

  std::vector<Point2D> points;
  for(int i = 0; i < scan_.ranges.size(); i++) {
    if(scan_.ranges[i] < scan_.range_min || scan_.ranges[i] >= scan_.range_max) {
      // scan_.ranges[i] = INFINITY;
      continue;
    }
    auto angle = scan_.angle_min + i * scan_.angle_increment;
    auto point = utils::polarToCartesian(scan_.ranges[i], angle);

    points.emplace_back(point.x(), point.y());
  }

  if(!points.empty()) {
    utils::KDTree<Point2D> tree(points);
    auto obstacles = clusterPoints(tree);
    for(auto &ob: obstacles) {
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
      ob.side = minDistance < 2.0 ? (leastPoint.y > ob.relativeMean.y ? 1 : -1) : 0;
    }
    filterObstacles(obstacles);
    evolvePoints(obstacles);
    obstacles = getObstacles();
    publishObstacles(obstacles);
  }
}

void GraphDetector::filterObstacles(std::vector<cluster_t> &obstacles) {
  auto iter = obstacles.begin();
  while(iter != obstacles.end()) {
    racecar_msgs::QueryStageRequest req;
    racecar_msgs::QueryStageResponse res;

    req.knownStage = -1;
    req.point = iter->mean.toPoint();
    if(stageClient_.call(req, res)) {
      if(res.stage < 0 || std::abs(res.currentStage - res.stage) > 2 || res.leftward < detectPadding_ || res.rightward < detectPadding_) {
        iter = obstacles.erase(iter);
        continue;
      }

      if(res.backward < transferDistance_) {
        req.knownStage = res.stage-1;
        stageClient_.call(req, res);
      }

      iter->stage = res.stage;
      iter->distances = utils::distance_t(res.forward, res.backward, res.leftward, res.rightward);
    }
    iter++;
  }
}

bool GraphDetector::isFamiliarObstacle(const cluster_t &o1, const cluster_t &o2) {
  return o1.mean.distance(o2.mean) < obstacleMovementTolerance_;
}

bool GraphDetector::evolvePoints(std::vector<cluster_t> &newObstacles) {
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
          pair.second.side = obstacle.side;
          pair.second.surviveCounter = 0;

          publishEvent(pair.second, EVENT_TYPE::ADD);
        } else {
          publishEvent(pair.second, EVENT_TYPE::UPDATE);
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
//      if(stableObstacles_[i].valid && stableObstacles_[i].dismissCounter == maximalReserve_) {
//        stableObstacles_[i].valid = false;
//        if(callback_) callback_(&stableObstacles_[i], REMOVE);
//        changed = true;
//      }
    }
  }

  return changed;
}

std::vector<cluster_t> GraphDetector::getObstacles() {
  std::vector<cluster_t> stableObstacles;
  for(const auto &pair : obstacles_) {
    if(pair.second.stable) {
      stableObstacles.push_back(pair.second);
    }
  }
  return stableObstacles;
}

void GraphDetector::publishEvent(const cluster_t &cluster, EVENT_TYPE event) {
  racecar_msgs::ObstacleEvent msg;
  msg.obstacle.stage = cluster.stage;
  msg.obstacle.mean = cluster.mean.toPoint();
  msg.obstacle.relativeMean = cluster.relativeMean.toPoint();
  msg.obstacle.forward = cluster.distances.forward;
  msg.obstacle.backward = cluster.distances.backward;
  msg.obstacle.leftward = cluster.distances.leftward;
  msg.obstacle.rightward = cluster.distances.rightward;
  msg.obstacle.side = cluster.side;
  msg.obstacle.id = cluster.id;
  msg.obstacle.survive = cluster.surviveCounter;

  if(event == EVENT_TYPE::ADD) {
    ROS_INFO("detected obstacle[%d]: %.2f, %.2f", cluster.id, cluster.mean.x, cluster.mean.y);
    msg.event = racecar_msgs::ObstacleEvent::ADD;
  } else if(event == EVENT_TYPE::UPDATE) {
    msg.event = racecar_msgs::ObstacleEvent::MODIFY;
  }

  obstacleEventPub_.publish(msg);
}

void GraphDetector::publishObstacles(const std::vector<cluster_t> &obstacles) {
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

std::vector<cluster_t> GraphDetector::clusterPoints(utils::KDTree<Point2D> &tree) {
  auto clusters = tree.cluster(clusterTolerance_, minPoints_);
  auto obstacles = std::vector<cluster_t>();
  for(auto &pts: clusters) {
    Point2D mean(0, 0);
    for(auto &pt : pts) {
      mean += pt / pts.size();
    }
    auto cpt = tree.radiusSearch(mean, includeRadius_).size();
    if(cpt == pts.size() && cpt == tree.radiusSearch(mean, excludeRadius_).size() && mean.length() < distanceFilter_) {
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
  ros::init(argc, argv, "graph_detector");
  GraphDetector detector;

  ros::spin();
  return 0;
}