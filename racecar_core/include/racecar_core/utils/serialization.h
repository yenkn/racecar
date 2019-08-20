//
// Created by yenkn on 19-8-15.
//

#ifndef PROJECT_SERIALIZATION_H
#define PROJECT_SERIALIZATION_H

#include <string>
#include <fstream>

namespace utils {

bool readPoints(const std::string &file, std::vector<Point2D> &points) {
  std::ifstream fs(file, std::ifstream::in);
  if(!fs.is_open()) {
    return false;
  }

  double x, y;
  bool res = false;
  while (fs >> x >> y) {
    points.emplace_back(x, y);
    res = true;
  }
  return res;
}

bool writePoints(const std::string &file, const std::vector<Point2D> &points) {
  std::ofstream fs(file, std::ofstream::out | std::ofstream::trunc);
  if(!fs.is_open()) {
    return false;
  }
  for(auto &pt : points) {
    fs << std::setprecision(4) << pt.x << ' ' << pt.y << std::endl;
  }
  return true;
}

}

#endif //PROJECT_SERIALIZATION_H
