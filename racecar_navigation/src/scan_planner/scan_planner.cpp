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
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <racecar_core/point.h>
#include <racecar_msgs/ObstacleEvent.h>
#include <racecar_msgs/Stage.h>
#include <racecar_msgs/GetPathSimilarity.h>

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
  ros::Publisher markerPub_, startPub_, goalPub_, pathPub_, similarityPub_;
  ros::Subscriber scanSub_, pathSub_, goalSub_, obstacleEventSub_, stageSub_;
  ros::ServiceClient similarityClient_;
  costmap_2d::Costmap2DROS localCostmap_;
  Point2D goal_;
  geometry_msgs::PoseStamped globalGoal_;

  racecar_msgs::Stage stage_;
  geometry_msgs::PoseStamped carPose_;
  std::vector<Point2D> path_;
  int carIndex_ = 0;
  double goalDistance_;
  int maxKeep_;
  double minSimilarity_;
  double switchDistance_;
  bool goalRecived_ = false;

  std::set<int> globalStages_;

  int keepCounter_ = 0;

  int pathLocker = 0;
  double switchTime_ = 0.2; // given time to replan
  double lastSwitchTime_ = 0.0;

  nav_msgs::Path previousPath_;
  double pathLockLength_ = 1.5;

  std::string plannerType_;
  path_planner::BasePathPlanner *planner_;


  void replan();
  bool isOccupied(const Point2D &pt, bool &occupied);
  double pathSimilarity(const nav_msgs::Path &path1, const nav_msgs::Path &path2);

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
  privateNode_.param("max_keep", maxKeep_, 5);
  privateNode_.param("min_similarity", minSimilarity_, 0.8);
  privateNode_.param("switch_distance", switchDistance_, 1.0);

  std::vector<int> stages;
  privateNode_.param("global_stages", stages, {});
  globalStages_ = std::set<int>(stages.begin(), stages.end());

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
  similarityClient_ = node_.serviceClient<racecar_msgs::GetPathSimilarity>("compare_path");

  pathPub_ = privateNode_.advertise<nav_msgs::Path>("plan", 1);
  startPub_ = privateNode_.advertise<geometry_msgs::PointStamped>("start", 1);
  goalPub_ = privateNode_.advertise<geometry_msgs::PointStamped>("goal", 1);
  similarityPub_ = privateNode_.advertise<std_msgs::Float64>("similarity", 1);
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

double ScanPlanner::pathSimilarity(const nav_msgs::Path &path1, const nav_msgs::Path &path2) {
  racecar_msgs::GetPathSimilarityRequest req;
  racecar_msgs::GetPathSimilarityResponse res;

  req.path1 = path1;
  req.path2 = path2;
  if(similarityClient_.call(req, res)) {
    return res.similarity;
  }
  return -1.0;
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

  if(!getRobotPose(carPose_)) {
    return;
  }
  Point2D carPoint(carPose_.pose.position.x, carPose_.pose.position.y);
  nav_msgs::Path globalLocalPath;

  double minDistance = FLT_MAX;
  bool foundGoal = false, outBound = false, occupied = false;
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
    globalLocalPath.poses.push_back(path_[i].toPoseStamped("map"));

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
    keepCounter_++;
    return;
  }

  nav_msgs::Path final;
  if(globalStages_.find(stage_.stage) != globalStages_.end() // current stage
    && (globalStages_.find(stage_.stage+1) != globalStages_.end() || stage_.forward_distance > switchDistance_)) {
    final.poses = globalLocalPath.poses;
  } else {
    geometry_msgs::PointStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "map";
    start.header.stamp = goal.header.stamp = ros::Time::now();
    goal.point = goal_.toPoint();
    goalPub_.publish(goal);

    start.point = carPoint.toPoint();
    startPub_.publish(start);

    geometry_msgs::PoseStamped startPose, endPose;
    startPose.header.frame_id = endPose.header.frame_id = "map";
    startPose.header.stamp = endPose.header.stamp = ros::Time::now();
    startPose.pose.position = start.point;
    startPose.pose.orientation.w = 1.0;

    endPose.pose.position = goal.point;
    endPose.pose.orientation.w = 1.0;

    nav_msgs::Path path;
    if (!planner_->makePlan(startPose, endPose, path.poses)) {
      ROS_ERROR("Failed to make plan");
      keepCounter_++;
      return;
    }
    final.poses = path.poses;

    double similarity = pathSimilarity(path, previousPath_);
    std_msgs::Float64 simMsg;
    simMsg.data = similarity;
    similarityPub_.publish(simMsg);

    if (similarity > 0 && similarity < minSimilarity_ && keepCounter_ < maxKeep_) {
      ROS_WARN("path changed too much, dropping");
      keepCounter_++;
      return;
    }
  }

  final.header.stamp = ros::Time::now();
  final.header.frame_id = "map";

  keepCounter_ = 0;
  pathPub_.publish(final);
  previousPath_ = final;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_planner");
  ScanPlanner node;
  ros::spin();
  return 0;
}