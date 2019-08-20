//
// Created by yenkn on 19-4-19.
//
#include "border_adjustment_ros.h"

BorderAdjustmentROS::BorderAdjustmentROS(int obstacleThreshold): privateNode_("~"), BorderAdjustment(obstacleThreshold) {
  costmapSub_ = privateNode_.subscribe("/move_base/local_costmap/costmap", 1, &BorderAdjustmentROS::costmapCallback, this);
  globalSub_ = privateNode_.subscribe("/move_base/global_costmap/costmap", 1, &BorderAdjustmentROS::globalCostmapCallback, this);

  pathPub_ = privateNode_.advertise<nav_msgs::Path>("/border/path", 1);
  leftPub_ = privateNode_.advertise<nav_msgs::Path>("/border/left_path", 1);
  rightPub_ = privateNode_.advertise<nav_msgs::Path>("/border/right_path", 1);
  potentialPub_ = privateNode_.advertise<nav_msgs::OccupancyGrid>("/border/potential", 1);
  costmapPub_ = privateNode_.advertise<nav_msgs::OccupancyGrid>("/border/costmap", 1);
}

geometry_msgs::PoseStamped BorderAdjustmentROS::vectorToPoseStamped(const std::string &frame_id, const tf::Vector3 &pt) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = pt.x();
  pose.pose.position.y = pt.y();
  pose.pose.position.z = pt.z();
  pose.pose.orientation.w = 1;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = frame_id;
  return pose;
}

void BorderAdjustmentROS::costmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap) {
  costmapMsg_ = *costmap;
}

void BorderAdjustmentROS::globalCostmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap) {
  globalCostmapMsg_ = *costmap;
}

bool BorderAdjustmentROS::getBorder(const nav_msgs::Path &path, std::vector<tf::Vector3> &left, std::vector<tf::Vector3> &right, std::vector<tf::Vector3> &optimalPath) {
  setCostmap(costmapMsg_, globalCostmapMsg_);
  setGlobalPath(path);

  optimalPath = getPath();
  BorderAdjustment::getBorder(optimalPath, left, right);

  nav_msgs::OccupancyGrid msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "odom";
  msg.info = costmapMsg_.info;
  int maxElement = 1;
  int max = costmapMsg_.info.width * costmapMsg_.info.height;
  for(int i = 0; i < max; i++) {
    maxElement = (dist_[i] > maxElement && dist_[i] < INT_MAX) ? dist_[i] : maxElement;
  }
  for(int i = 0; i < max; i++) {
    msg.data.push_back(int8_t(dist_[i] == INT_MAX ? -1 : dist_[i] * 100 / maxElement));
  }
  potentialPub_.publish(msg);

  msg.data.clear();
  std::copy_n(costmap_.data(), costmap_.size(), std::back_inserter(msg.data));
  costmapPub_.publish(msg);

  nav_msgs::Path pathMsg;
  pathMsg.header.stamp = ros::Time::now();
  pathMsg.header.frame_id = "odom";
  for(const auto &pt : optimalPath) {
    pathMsg.poses.push_back(vectorToPoseStamped(pathMsg.header.frame_id, pt));
  }
  pathPub_.publish(pathMsg);

  nav_msgs::Path leftMsg;
  leftMsg.header.stamp = ros::Time::now();
  leftMsg.header.frame_id = "base_footprint";
  for(const auto &pt : left) {
    leftMsg.poses.push_back(vectorToPoseStamped(leftMsg.header.frame_id, pt));
  }
  leftPub_.publish(leftMsg);

  leftMsg.poses.clear();
  for(auto pt : right) {
    leftMsg.poses.push_back(vectorToPoseStamped(leftMsg.header.frame_id, pt));
  }
  rightPub_.publish(leftMsg);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "border_adjustment");
  BorderAdjustmentROS borderAdjustmentROS;

  ros::spin();
}