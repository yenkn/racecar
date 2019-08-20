//
// Created by yenkn on 19-4-15.
//

#ifndef PROJECT_BORDER_ADJUSTMENT_H
#define PROJECT_BORDER_ADJUSTMENT_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>

#include "grid_path.h"

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix8i;

class BorderAdjustment {
public:
  explicit BorderAdjustment(int obstacleThreshold = 99): obstacleThreshold_(obstacleThreshold), nodeHandle_("~") {
    markerPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/border/marker", 1);
  }
  ~BorderAdjustment();

  void setGlobalPath(const nav_msgs::Path &path);
  void setCostmap(const nav_msgs::OccupancyGrid &costmap, const nav_msgs::OccupancyGrid &globalCostmap);

  std::vector<tf::Vector3> getPath();
  bool getBorder(const std::vector<tf::Vector3> &path, std::vector<tf::Vector3> &left, std::vector<tf::Vector3> &right);

protected:
  struct heapNode {
    double distance;
    int index;

    heapNode() = default;
    heapNode(double d, int i): distance(d), index(i) {}
    bool operator <(const heapNode & a) const
    {
        return distance > a.distance;
    }
  };

  tf::TransformListener tfListener_;

  bool *visited_ = nullptr;
  int *prev_ = nullptr, *dist_ = nullptr;
  Matrix8i costmap_;

  inline tf::Vector3 pointToVector(const geometry_msgs::Point &point) {
    return { point.x, point.y, point.z };
  }

  inline geometry_msgs::Point vectorToPoint(const tf::Vector3 &pt) {
    geometry_msgs::Point point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    return point;
  }

  inline void clearPreviousPath() {
    previousPath_.clear();
  }

private:
  nav_msgs::Path globalPath_;
  int obstacleThreshold_;
  int width_ = 0, height_ = 0;
  double resolution_ = 0;
  tf::Vector3 origin_;

  ros::NodeHandle nodeHandle_;
  ros::Publisher markerPub_;

  std::vector<tf::Vector3> previousPath_;
  GridPath pathMaker_;

  inline tf::Vector3 costmapToOdom(const tf::Vector3 &costmap);
  inline tf::Vector3 odomToCostmap(const tf::Vector3 &odom);
  bool getTFTransform(const std::string &from, const std::string &target, tf::StampedTransform &transform);

  std::vector<int> getNeighbors(const tf::Vector3 &start, const tf::Vector3 &goal);
  std::vector<tf::Vector3> findOptimalPath(const tf::Vector3 &start, tf::Vector3 &goal);
  tf::Vector3 findGoalPoint();
  tf::Vector3 adjustGoalPoint(const tf::Vector3 &goal);
};

#endif //PROJECT_BORDER_ADJUSTMENT_H
