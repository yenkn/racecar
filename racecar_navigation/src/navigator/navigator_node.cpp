//
// Created by yenkn on 19-5-17.
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

#include "path_planner/base_path_planner.h"
#include "path_planner/planner_core.h"
#include "sbpl_planner/sbpl_planner.h"

#include "obstacle_detector/obstacle_detector.h"
#include "costmap_wrapper.h"

class NavigatorNode {
public:
  NavigatorNode();
  ~NavigatorNode();

private:
  tf::TransformListener tf_;
  ros::NodeHandle privateNode_, node_;
  ros::Publisher markerPub_, collisionPub_, pathPub_;
  ros::Subscriber goalSub_, amclSub_;
  costmap_2d::Costmap2DROS globalCostmap_;
  CostmapWrapper plannerCostmap_;
  geometry_msgs::PoseStamped goal_;
  bool goalRecived_ = false;

  std::string plannerType_;
  path_planner::BasePathPlanner *planner_;
  ObstacleDetector detector_;

  void replan();

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
  bool getRobotPose(geometry_msgs::PoseStamped &pose);
  void obstacleCallback(ObstacleDetector::cluster *obstacle, int type);

  Point2D detectCollision(const geometry_msgs::Pose &pose);
};

NavigatorNode::NavigatorNode() :
  privateNode_("~"),
  tf_(ros::Duration(10.0)),
  globalCostmap_("global_costmap", tf_),
  plannerCostmap_(&privateNode_),
  detector_("obstacle_detector", globalCostmap_.getCostmap()) {
  globalCostmap_.pause();
  plannerCostmap_.updateMap(*globalCostmap_.getCostmap());

  privateNode_.param<std::string>("planner_type", plannerType_, "global");

  if(plannerType_ == "global") {
    planner_ = new path_planner::PathPlanner("path_planner", plannerCostmap_.getCostmap(), globalCostmap_.getGlobalFrameID());
  } else if(plannerType_ == "sbpl") {
    planner_ = new sbpl_planner::SBPLLatticePlanner("sbpl_planner", plannerCostmap_.getCostmap(), &globalCostmap_);
  }
  globalCostmap_.start();

  goalSub_ = node_.subscribe("/move_base_simple/goal", 1, &NavigatorNode::goalCallback, this);
  amclSub_ = node_.subscribe("/amcl_pose", 1, &NavigatorNode::amclCallback, this);

  collisionPub_ = privateNode_.advertise<geometry_msgs::PointStamped>("collision", 1);
  pathPub_ = privateNode_.advertise<nav_msgs::Path>("plan", 1);

  detector_.setObstacleCallback(std::bind(&NavigatorNode::obstacleCallback, this, std::placeholders::_1, std::placeholders::_2));
}

NavigatorNode::~NavigatorNode() {
  delete planner_;
}

void NavigatorNode::obstacleCallback(ObstacleDetector::cluster *obstacle, int type) {
  if(!goalRecived_) return;

  tf::StampedTransform mapToCar;
  if(!utils::getTFTransform(tf_, "map", "base_footprint", mapToCar)) return;
  auto obstacleInCar = mapToCar * tf::Vector3(obstacle->mean.x, obstacle->mean.y, 0);

  if(type == ObstacleDetector::ADD) {
    std::cout << "add obstacle at (" << obstacleInCar.x() << ", " << obstacleInCar.y() << ")" << std::endl;
  } else if(type == ObstacleDetector::REMOVE) {
    std::cout << "remove obstacle at (" << obstacleInCar.x() << ", " << obstacleInCar.y() << ")" << std::endl;
  }

  if(obstacleInCar.x() > 0) replan();
}


bool NavigatorNode::getRobotPose(geometry_msgs::PoseStamped &pose) {
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

void NavigatorNode::replan() {
  plannerCostmap_.updateMap(*globalCostmap_.getCostmap(), detector_.getObstacles());
  geometry_msgs::PoseStamped start;
  getRobotPose(start);
  nav_msgs::Path path;
  if(!planner_->makePlan(start, goal_, path.poses)) {
    ROS_ERROR("Failed to make plan");
    return;
  }
  path.header.stamp = ros::Time::now();
  path.header.frame_id = globalCostmap_.getGlobalFrameID();
  pathPub_.publish(path);
}

void NavigatorNode::amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
  auto pt = detectCollision(pose->pose.pose);
  // std::cout << "Collision Point: " << pt << ", distance: " << pt.distance(Point2D(pose->pose.pose.position.x, pose->pose.pose.position.y)) << std::endl;

  geometry_msgs::PointStamped pts;
  pts.header.frame_id = "map";
  pts.header.stamp = ros::Time::now();
  pts.point = pt.toPoint();

  collisionPub_.publish(pts);
}

Point2D NavigatorNode::detectCollision(const geometry_msgs::Pose &pose) {
  tf::StampedTransform mapToCar;
  if(utils::getTFTransform(tf_, "map", "base_footprint", mapToCar)) {
    double angleLow = (double)(-15) / 180 * M_PI;
    double angleHigh = (double)15 / 180 * M_PI;
    auto obstacles = detector_.getObstacles();
    for(auto &obstacle : obstacles) {
      auto obstacleInCar = mapToCar * tf::Vector3(obstacle.mean.x, obstacle.mean.y, 0);
      auto theta = atan2(obstacleInCar.y(), obstacleInCar.x());
      if(theta > angleLow && theta < angleHigh) return obstacle.mean;
    }
  }

  auto costmap = plannerCostmap_.getCostmap();
  auto charmap = costmap->getCharMap();
  double posX = pose.position.x, posY = pose.position.y, yaw = tf::getYaw(pose.orientation);
  while(true) {
    unsigned int mapX, mapY;
    if(!costmap->worldToMap(posX, posY, mapX, mapY)) {
      return Point2D(pose.position.x, pose.position.y);
    }
    auto index = costmap->getIndex(mapX, mapY);
    if(charmap[index] == costmap_2d::LETHAL_OBSTACLE) {
      double wx, wy;
      costmap->mapToWorld(mapX, mapY, wx, wy);
      return Point2D(wx, wy);
    }
    posX += costmap->getResolution() * cos(yaw);
    posY += costmap->getResolution() * sin(yaw);
  }
}

void NavigatorNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  goalRecived_ = true;
  goal_ = *goal;
  replan();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigator");
  NavigatorNode node;
  ros::spin();
  return 0;
}