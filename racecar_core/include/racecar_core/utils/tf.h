//
// Created by yenkn on 19-5-16.
//

#ifndef PROJECT_UTILS_TF_H
#define PROJECT_UTILS_TF_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/MapMetaData.h>

namespace utils {

bool getTFTransform(
    const tf::TransformListener &listener,
    const std::string &from,
    const std::string &target,
    tf::StampedTransform &transform) {
  try {
    listener.waitForTransform(target, from, ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(target, from, ros::Time(0), transform);
  } catch (const tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  return true;
}

inline tf::Vector3 pointToVector(const geometry_msgs::Point &point) {
  return {point.x, point.y, point.z};
}

inline geometry_msgs::Point vectorToPoint(const tf::Vector3 &pt) {
  geometry_msgs::Point point;
  point.x = pt.x();
  point.y = pt.y();
  point.z = pt.z();
  return point;
}

inline tf::Vector3 costmapToOdom(const nav_msgs::MapMetaData &info, const tf::Vector3 &costmap) {
  return {
      info.origin.position.x + floor(costmap.x()) * info.resolution,
      info.origin.position.y + floor(costmap.y()) * info.resolution,
      0
  };
}

bool odomToCostmap(const nav_msgs::MapMetaData &info, const tf::Vector3 &odom, tf::Vector3 &costmap) {
  double x = floor((odom.x() - info.origin.position.x) / info.resolution);
  double y = floor((odom.y() - info.origin.position.y) / info.resolution);
  costmap = { x, y, 0 };
  return x >= 0 && y >= 0 && x < info.width && y < info.height;
}

inline tf::Vector3 polarToCartesian(double r, double phi) {
  return { r * cos(phi), r * sin(phi), 0 };
}

tf::Transform createTfFromXYTheta(
    double x, double y, double theta)
{
  tf::Transform t;
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
  return t;
}

}

#endif //PROJECT_UTILS_TF_H
