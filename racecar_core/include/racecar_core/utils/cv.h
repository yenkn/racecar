//
// Created by yenkn on 19-7-15.
//

#ifndef SRC_CV_H
#define SRC_CV_H

#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

namespace utils {

cv::Mat mapToImage(const nav_msgs::OccupancyGrid &map, int pick = 100 /* occupied */, bool flip = false) {
  cv::Mat mapImg(map.info.height, map.info.width, CV_8UC1, cv::Scalar(0));
  size_t len = map.data.size();
  for (int i = 1; i < len; i++) {
    if (map.data[i] == pick) {
      int y = (int) (i / map.info.width);
      mapImg.at<uchar>(flip ? map.info.height - y : y, int(i % map.info.width)) = 225;
    }
  }
  return mapImg;
}

cv::Mat scanToImage(const sensor_msgs::LaserScan &scan, double resolution, double rotation = 0.0) {
  int scanWidth = scan.range_max * 2 / resolution, scanHeight = scan.range_max * 2 / resolution;
  Point2D center(scanWidth / 2, scanHeight / 2);
  cv::Mat scanImage(scanHeight, scanWidth, CV_8UC1, cv::Scalar(0));

  for(int i = 0; i < scan.ranges.size(); i++) {
    double range = scan.ranges[i];
    if (range < scan.range_min || range >= scan.range_max) {
      continue;
    }
    auto angle = scan.angle_min + i * scan.angle_increment;
    auto pixel = Point2D::fromPoolarCoords(range, angle).rotate(rotation) / resolution + center;
    if(pixel.x < 0 || pixel.x >= scanWidth || pixel.y < 0 || pixel.y >= scanHeight) {
      continue;
    }
    scanImage.at<uchar>(scanHeight - (int)pixel.y, (int)pixel.x) = 225;
  }
  return scanImage;
}

    
}

#endif //SRC_CV_H
