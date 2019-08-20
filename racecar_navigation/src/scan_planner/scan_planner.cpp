//
// Created by yenkn on 19-8-17.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <racecar_core/utils/tf.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <racecar_core/point.h>
#include <racecar_msgs/ObstacleEvent.h>
#include <racecar_msgs/Stage.h>

#include "path_planner/base_path_planner.h"
#include "path_planner/planner_core.h"
#include "sbpl_planner/sbpl_planner.h"

class ScanPlanner {
public:
  ScanPlanner();
  ~ScanPlanner();

private:
  tf::TransformListener tf_;
  ros::NodeHandle privateNode_, node_;
  ros::Publisher markerPub_, startPub_, goalPub_, pathPub_;
  ros::Subscriber scanSub_, pathSub_, goalSub_, obstacleEventSub_, stageSub_;
  costmap_2d::Costmap2DROS localCostmap_;
  Point2D goal_;
  geometry_msgs::PoseStamped globalGoal_;

  racecar_msgs::Stage stage_;
  geometry_msgs::PoseStamped carPose_;
  std::vector<Point2D> path_;
  int carIndex_ = 0;
  double goalDistance_;
  int replanStep_;
  bool goalRecived_ = false;


  int pathLocker = 0;
  double switchTime_ = 0.2; // given time to replan
  double lastSwitchTime_ = 0.0;

  std::vector<Point2D> previousPath_;
  double pathLockLength_ = 1.5;

  std::string plannerType_;
  path_planner::BasePathPlanner *planner_;


  void replan();
  bool isOccupied(const Point2D &pt, bool &occupied);

  void stageCallback(const racecar_msgs::StageConstPtr &msg);
  void obstacleCallback(const racecar_msgs::ObstacleEventConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void pathCallback(const nav_msgs::PathConstPtr &msg);
  bool getRobotPose(geometry_msgs::PoseStamped &pose);
};

ScanPlanner::ScanPlanner() :
  privateNode_("~"),
  tf_(ros::Duration(10.0)),
  localCostmap_("local_costmap", tf_) {
  localCostmap_.pause();

  privateNode_.param<std::string>("planner_type", plannerType_, "global");
  privateNode_.param("goal_distance", goalDistance_, 3.0);
  privateNode_.param("replan_step", replanStep_, 50);

  if(plannerType_ == "global") {
    planner_ = new path_planner::PathPlanner("path_planner", localCostmap_.getCostmap(), localCostmap_.getGlobalFrameID());
  } else if(plannerType_ == "sbpl") {
    planner_ = new sbpl_planner::SBPLLatticePlanner("sbpl_planner", localCostmap_.getCostmap(), &localCostmap_);
  }
  localCostmap_.start();

  scanSub_ = node_.subscribe("/scan", 1, &ScanPlanner::scanCallback, this);
  pathSub_ = node_.subscribe("/graph_planner/plan", 1, &ScanPlanner::pathCallback, this);
  goalSub_ = node_.subscribe("/move_base_simple/goal", 1, &ScanPlanner::goalCallback, this);
  obstacleEventSub_ = node_.subscribe("/graph_detector/event", 100, &ScanPlanner::obstacleCallback, this);
  stageSub_ = node_.subscribe("/stage", 1, &ScanPlanner::stageCallback, this);

  pathPub_ = privateNode_.advertise<nav_msgs::Path>("plan", 1);
  startPub_ = privateNode_.advertise<geometry_msgs::PointStamped>("start", 1);
  goalPub_ = privateNode_.advertise<geometry_msgs::PointStamped>("goal", 1);


}

ScanPlanner::~ScanPlanner() {
  delete planner_;
}

void ScanPlanner::stageCallback(const racecar_msgs::StageConstPtr &msg) {
  stage_ = *msg;
}

void ScanPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  globalGoal_ = *msg;
  goalRecived_ = true;
}

void ScanPlanner::obstacleCallback(const racecar_msgs::ObstacleEventConstPtr &msg) {
  return;
  auto now = ros::Time::now().toSec();
  if(msg->event == racecar_msgs::ObstacleEvent::MODIFY) {
    if(hypot(msg->obstacle.relativeMean.x, msg->obstacle.relativeMean.y) <= 2.0 && msg->obstacle.relativeMean.x > 0 && pathLocker == 0 && (stage_.stage ==11 || stage_.stage ==12)) {
        replan();
        pathLocker = msg->obstacle.id;
        std::cout << "path locked by: " << pathLocker << std::endl;
        switchTime_ = now;
    } else if(pathLocker == msg->obstacle.id && msg->obstacle.relativeMean.x <= 0) {
        std::cout << "path unlocked by: " << pathLocker << std::endl;
        pathLocker = 0;
        switchTime_ = now;
    }
  }
}

bool ScanPlanner::isOccupied(const Point2D &pt, bool &occupied) {
  unsigned int mx, my;
  if(!localCostmap_.getCostmap()->worldToMap(pt.x, pt.y, mx, my)) {
    return false;
  }

  auto cost = localCostmap_.getCostmap()->getCost(mx, my);
  occupied = (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::LETHAL_OBSTACLE);
  return true;
}

void ScanPlanner::scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
  if(!goalRecived_ || pathLocker != 0) return;

  replan();
}

void ScanPlanner::pathCallback(const nav_msgs::PathConstPtr &msg) {
  path_.clear();
  for(auto &pt : msg->poses) {
    path_.emplace_back(pt.pose.position.x, pt.pose.position.y);
  }
}

bool ScanPlanner::getRobotPose(geometry_msgs::PoseStamped &pose) {
  geometry_msgs::PoseStamped robot_pose, global_pose;
  robot_pose.header.frame_id = "base_footprint";
  robot_pose.header.stamp = ros::Time();
  tf::poseTFToMsg(tf::Transform::getIdentity(), robot_pose.pose);

  try {
    tf_.transformPose("map", ros::Time(0), robot_pose, "map", pose);
    return true;
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

void ScanPlanner::replan() {
  if(path_.empty()) {
    ROS_ERROR("no global path");
    return;
  }


  double minDistance = FLT_MAX;
  bool foundGoal = false, outBound = false, occupied = false;
  if(!getRobotPose(carPose_)) {
    return;
  }
  Point2D carPoint(carPose_.pose.position.x, carPose_.pose.position.y);
  Point2D planStart = carPoint;
  auto prevPath = previousPath_;

  if(prevPath.size() > replanStep_) {
    int prevStartIndex = 0;
    for (int i = 0; i < prevPath.size(); i++) {
      double distance = prevPath[i].distance(carPoint);
      if (distance < minDistance) {
        minDistance = distance;
        prevStartIndex = i;
      }
      // if (isOccupied(prevPath[i], occupied) && occupied) break;
    }
    prevPath.erase(prevPath.begin(), prevPath.begin() + prevStartIndex);

    if(replanStep_ < prevPath.size()) {
      planStart = prevPath[prevPath.size() - replanStep_];
      prevPath.erase(prevPath.end() - replanStep_, prevPath.end()); // remove tail points
    } else {
      prevPath.clear();
    }
  }

  minDistance = FLT_MAX;
  int startIndex = std::max(carIndex_ - 10, 0);
  for (int i = startIndex; i < path_.size(); i++) {
    auto distance = carPoint.distance(path_[i]);
    if(distance < minDistance) {
      minDistance = distance;
      carIndex_ = i;
    }
  }

  double totalDistance = 0.0;
  for (int i = carIndex_+1; i < path_.size(); i++) {
    totalDistance += path_[i].distance(path_[i-1]);

    if(!isOccupied(path_[i],occupied)) {
      outBound = true;
      break;
    }

    if(totalDistance >= goalDistance_ && !occupied) {
      goal_ = path_[i];
      foundGoal = true;
      break;
    }
  }

  if(!foundGoal && !outBound) {
    goal_ = Point2D(globalGoal_.pose.position.x, globalGoal_.pose.position.y);
  }

  if(outBound) {
    ROS_WARN("out bound, keeping last plan");
    return;
  }

  geometry_msgs::PointStamped startPt, goal;
  startPt.header.frame_id = goal.header.frame_id = "map";
  startPt.header.stamp = goal.header.stamp = ros::Time::now();
  goal.point = goal_.toPoint();
  goalPub_.publish(goal);

  startPt.point = planStart.toPoint();
  startPub_.publish(startPt);


  geometry_msgs::PoseStamped start, end;
  start.header.frame_id = end.header.frame_id = "map";
  start.header.stamp = end.header.stamp = ros::Time::now();
  start.pose.position.x = planStart.x;
  start.pose.position.y = planStart.y;
  start.pose.orientation.w = 1.0;

  end.pose.position = goal_.toPoint();
  end.pose.orientation.w = 1.0;

  nav_msgs::Path path;
  if(!planner_->makePlan(start, end, path.poses)) {
    ROS_ERROR("Failed to make plan");
    return;
  }

  for(auto &pt: path.poses) {
    prevPath.emplace_back(pt.pose.position.x, pt.pose.position.y);
  }

//  double yaw = tf::getYaw(carPose_.pose.orientation);
//  double relateX = (path.poses[20].pose.position.x - carPose_.pose.position.x)*cos(yaw) + (path.poses[20].pose.position.y - carPose_.pose.position.y)*sin(yaw);
//  if(path.poses.size() > 20 && relateX < 0) {
//    if(replan) {
//      ROS_ERROR("drop reverse path");
//      return;
//    } else {
//      ROS_WARN("generating reverse path, replanning.");
//      replan = true;
//      goto plan;
//    }
//  }

  nav_msgs::Path final;
  final.header.stamp = ros::Time::now();
  final.header.frame_id = "map";

  geometry_msgs::PoseStamped position;
  position.header = final.header;
  position.pose.orientation.w = 1.0;
  for(int i = 0; i < prevPath.size(); i++) {
    if(i < 20 || i >= prevPath.size()-20) {
      position.pose.position = prevPath[i].toPoint();
    } else {
      position.pose.position = ((prevPath[i] + prevPath[i-20] + prevPath[i+20]) / 3).toPoint();
    }
    final.poses.push_back(position);
  }

  pathPub_.publish(final);
  previousPath_ = prevPath;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_planner");
  ScanPlanner node;
  ros::spin();
  return 0;
}