//
// Created by yenkn on 19-5-16.
//

#ifndef PROJECT_UTILS_LASER_H
#define PROJECT_UTILS_LASER_H

#include <sensor_msgs/LaserScan.h>

namespace utils {

constexpr double degreeToRadius(double degree) {
  return degree / 180 * M_PI;
}

std::vector<float> cutLaser(const sensor_msgs::LaserScan &scan, const double &beginRadius, const double &endRadius) {
  ROS_ASSERT(endRadius > beginRadius);

  auto start = int(beginRadius / scan.angle_increment);
  auto count = int((endRadius - beginRadius) / scan.angle_increment);

  auto zeroIterator = scan.ranges.cbegin() + int(-scan.angle_min / scan.angle_increment);
  std::vector<float> result;
  for (int i = start; i < count; i++) {
    auto iterator = zeroIterator + i;
    if (iterator < scan.ranges.cbegin()) {
      iterator += scan.ranges.size();
    } else if (iterator >= scan.ranges.cend()) {
      iterator -= scan.ranges.size();
    }
    result.push_back(*iterator);
  }

  return result;
}

}

#endif //PROJECT_UTILS_LASER_H
