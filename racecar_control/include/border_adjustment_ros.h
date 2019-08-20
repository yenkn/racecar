//
// Created by yenkn on 19-4-25.
//
#include "border_adjustment.h"

#ifndef PROJECT_BORDER_ADJUSTMENT_ROS_H
#define PROJECT_BORDER_ADJUSTMENT_ROS_H

class BorderAdjustmentROS : private BorderAdjustment {
public:
  explicit BorderAdjustmentROS(int obstacleThreshold = 99);
  bool getBorder(const nav_msgs::Path &path, std::vector<tf::Vector3> &left, std::vector<tf::Vector3> &right, std::vector<tf::Vector3> &optimalPath);
  geometry_msgs::PoseStamped vectorToPoseStamped(const std::string &frame_id, const tf::Vector3 &pt);
  inline void clearPrevious() {
    BorderAdjustment::clearPreviousPath();
  }

private:
  ros::NodeHandle privateNode_;
  ros::Subscriber costmapSub_, globalSub_;
  ros::Publisher pathPub_, leftPub_, rightPub_, potentialPub_, costmapPub_;
  nav_msgs::OccupancyGrid costmapMsg_;
  nav_msgs::OccupancyGrid globalCostmapMsg_;

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap);
  void globalCostmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap);
};

#endif //PROJECT_BORDER_ADJUSTMENT_ROS_H
