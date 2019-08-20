//
// Created by yenkn on 19-5-18.
//

#ifndef PROJECT_COSTMAP_WRAPPER_H
#define PROJECT_COSTMAP_WRAPPER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include "obstacle_detector/obstacle_detector.h"

class CostmapWrapper {
public:
  explicit CostmapWrapper(ros::NodeHandle *node): node_(node), publisher_(node, &costmap_, "map", "costmap", true) {
    inflationRadius_ = node->param<double>("global_costmap/inflation/inflation_radius", 0.5);
  }

  costmap_2d::Costmap2D *getCostmap() {
    return &costmap_;
  }

  void updateMap(const costmap_2d::Costmap2D &costmap) {
    costmap_ = costmap;
  }

  void updateMap(const costmap_2d::Costmap2D &costmap, const std::vector<ObstacleDetector::cluster> &obstacles) {
    costmap_ = costmap;

    unsigned char *charmap = costmap_.getCharMap();
    for(auto &obstacle: obstacles) {
      drawCircle(charmap, obstacle.mean, inflationRadius_, 80);
      drawCircle(charmap, obstacle.mean, 0.2, costmap_2d::LETHAL_OBSTACLE);
    }

    publisher_.publishCostmap();

  }

private:
  costmap_2d::Costmap2D costmap_;
  costmap_2d::Costmap2DPublisher publisher_;
  ros::NodeHandle *node_;
  double inflationRadius_;

  void drawCircle(unsigned char *charmap, Point2D center, double radius, unsigned char color) {
    int r = int(radius / costmap_.getResolution());
    unsigned int x, y;
    costmap_.worldToMap(center.x, center.y, x, y);
    int r2 = r * r;
    int area = r2 << 2;
    int rr = r << 1;

    for (int i = 0; i < area; i++)
    {
      int tx = (i % rr) - r;
      int ty = (i / rr) - r;

      if (tx * tx + ty * ty <= r2) {
        auto index = costmap_.getIndex(x + tx, y + ty);
        if(charmap[index] < color) charmap[index] = color;
      }
    }
  }
};

#endif //PROJECT_COSTMAP_WRAPPER_H
