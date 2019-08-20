
#include <queue>
#include <eigen3/Eigen/QR>
#include <border_adjustment.h>

#include "border_adjustment.h"
#include "gradient_path.h"

tf::Vector3 BorderAdjustment::findGoalPoint() {
  int odomX = int(-origin_.x() / resolution_);
  int odomY = int(-origin_.y() / resolution_);
  int currentX = -1, currentY = -1;

  for(size_t i = globalPath_.poses.size()-1; i > 0; i--) {
    auto pose = globalPath_.poses[i].pose;
    currentX = int(odomX + pose.position.x / resolution_);
    currentY = int(odomY + pose.position.y / resolution_);
    if(currentX >= 0 && currentX < width_ && currentY >= 0 && currentY < height_) {
      break;
    }
  }
  return { (double)currentX, (double)currentY, 0.0 };
}

tf::Vector3 BorderAdjustment::adjustGoalPoint(const tf::Vector3 &goal) {
  //  if(goalOccupied) {
//    auto offCenterX = std::abs(goal.x - (double)width/2) / width;
//    auto offCenterY = std::abs(goal.y - (double)height/2) / height;
//    if(offCenterX > offCenterY) {
//      auto minDist = INT_MAX;
//      auto minY = goal.y;
//      for(int i = 0; i < height; i++) {
//        int index = int(i*width + goal.x);
//        if(dist[index] < minDist) {
//          minY = goal.x;
//          minDist = dist[index];
//        }
//      }
//      goal.y = minY;
//    } else {
//      auto minIndex = std::min_element(dist+(int)goal.y, dist+(int)goal.y+width);
//      goal.x = std::distance(dist+(int)goal.y, minIndex);
//    }
//  }
  return goal;
}

std::vector<tf::Vector3> BorderAdjustment::findOptimalPath(const tf::Vector3 &start, tf::Vector3 &goal) {
  std::vector<tf::Vector3> path;
  std::priority_queue<heapNode> q;

  int max = width_ * height_;
  std::fill_n(dist_, max, INT_MAX);
  std::fill_n(prev_, max, -1);
  std::fill_n(visited_, max, false);

  if(resolution_ <= 0 || start.x() < 0 || start.y() < 0) return path;
  int startIndex = int(start.y() * width_ + start.x());
  dist_[startIndex] = 0;
  auto startNode = heapNode(0, startIndex);
  q.push(startNode);
  auto goalOccupied = costmap_((int)goal.y(), (int)goal.x()) >= obstacleThreshold_;
  auto goalIndex = int(goal.y() * width_ + goal.x());
  auto neighbors = getNeighbors(start, goal);

  while(!q.empty()) {
    auto node = q.top(); q.pop();
    if(visited_[node.index]) continue;
    visited_[node.index] = true;

//    if(node.index == goalIndex) break;

    for(int i : neighbors) {
      int row = int(node.index / width_) - 1 + i / 3;
      int col = node.index % width_ - 1 + i % 3;

      if(col >= 0 && col < width_ && row >= 0 && row < height_) {
        int v = row * width_ + col;
        int cost = 1; // diagonal costs more

         int alt = (costmap_(row, col) >= obstacleThreshold_) ? INT_MAX : (dist_[node.index] + cost + costmap_(row, col));
        // int alt = dist_[node.index] + cost;

        if(alt < dist_[v]) {
          dist_[v] = alt;
          prev_[v] = node.index;
          q.push(heapNode(dist_[v], v));
        }
      }
    }
  }

//  auto v = goalIndex;
//  while(v != startIndex) {
//    auto point = tf::Vector3(v%width_, int(v/width_), 0);
//    path.push_back(costmapToOdom(point));
//    if(prev_[v] == -1) {
//      std::cout << "Path not found at: " << (v%width_) << ',' << (v / width_) << std::endl;
//      return path;
//    }
//    v = prev_[v];
//  }
//  path.push_back(costmapToOdom(start));
//  std::reverse(path.begin(), path.end());

  std::vector<std::pair<float, float>> costmapPath;
  if(pathMaker_.getPath(dist_, start.x(), start.y(), goal.x(), goal.y(), costmapPath)) {
    for(int i = (int)costmapPath.size()-1; i >= 0; i--) {
      assert(std::abs(costmapPath[i].first) < width_ && std::abs(costmapPath[i].second) < height_);
      path.push_back(costmapToOdom(tf::Vector3(costmapPath[i].first, costmapPath[i].second, 0)));
    }
  } else {
    std::cout << "Path not found." << std::endl;
  }

  return path;
}

tf::Vector3 BorderAdjustment::costmapToOdom(const tf::Vector3 &costmap) {
  return { origin_.x() + floor(costmap.x()) * resolution_, origin_.y() + floor(costmap.y()) * resolution_, 0 };
}

tf::Vector3 BorderAdjustment::odomToCostmap(const tf::Vector3 &odom) {
  return { floor((odom.x() - origin_.x()) / resolution_), floor((odom.y() - origin_.y()) / resolution_), 0 };
}

std::vector<int> BorderAdjustment::getNeighbors(const tf::Vector3 &start, const tf::Vector3 &goal) {
  return { 1, 3, 5, 7 };
  auto theta = atan2(goal.y()-start.y(), goal.x()-start.x());
  // std::cout << "theta is: " << theta << std::endl;
  if(theta > -M_PI/4 && theta <= M_PI/4) return { 1, 2, 5, 7, 8 }; // block left 036
  else if(theta > M_PI/4 && theta <= 4*M_PI/3) return { 3, 5, 6, 7, 8 }; // block bottom 678
  else if(theta > -3*M_PI/4 && theta <= -4*M_PI) return { 0, 1, 2, 3, 5 }; // block top 012
  else return { 0, 1, 3, 6, 7 }; // block right 258
}

void BorderAdjustment::setGlobalPath(const nav_msgs::Path &path) {
  globalPath_ = path;
}

void BorderAdjustment::setCostmap(const nav_msgs::OccupancyGrid &costmap, const nav_msgs::OccupancyGrid &global) {
  resolution_ = costmap.info.resolution;
  origin_ = pointToVector(costmap.info.origin.position);

  if(costmap.info.width != width_ || costmap.info.height != height_) {
    width_ = costmap.info.width;
    height_ = costmap.info.height;

    delete[] visited_;
    delete[] dist_;
    delete[] prev_;
    int max = width_ * height_;
    visited_ = new bool[max];
    dist_ = new int[max];
    prev_ = new int[max];
    pathMaker_.setSize(width_, height_);
  }


  costmap_ = Eigen::Map<Matrix8i>((int8_t *)costmap.data.data(), height_, width_);

  // special values:
//  cost_translation_table_[0] = 0;  // NO obstacle
//  cost_translation_table_[253] = 99;  // INSCRIBED obstacle
//  cost_translation_table_[254] = 100;  // LETHAL obstacle
//  cost_translation_table_[255] = -1;  // UNKNOWN

  if(global.info.resolution > 0) {
    // combine global costmap and local costmap
    tf::StampedTransform mapToOdom;
    tf::Transform odomToMap;
    if(!getTFTransform("map", "odom", mapToOdom)) {
      return;
    }
    odomToMap = mapToOdom.inverse();
    for(int row = 0; row < height_; row++) {
      for(int col = 0; col < width_; col++) {
        auto mapPos = odomToMap * costmapToOdom(tf::Vector3(col, row, 0));
        int mapRow = int((mapPos.y() - global.info.origin.position.y) / global.info.resolution);
        int mapCol = int((mapPos.x() - global.info.origin.position.x) / global.info.resolution);;
        if(mapCol < 0 || mapRow < 0 || mapCol >= global.info.width || mapRow >= global.info.height) {
          continue;
        }
        int mapIndex = mapRow * global.info.width + mapCol;
        if(global.data[mapIndex] >= obstacleThreshold_) {
          costmap_(row, col) = global.data[mapIndex];
        } else if(costmap_(row, col) < obstacleThreshold_) {
          costmap_(row, col) = (costmap_(row, col) + global.data[mapIndex]) / 2;
        }
      }
    }
  }
}

inline std_msgs::ColorRGBA makeColor(float r, float g, float b, float a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std::vector<tf::Vector3> BorderAdjustment::getPath() {
  tf::Vector3 goal = findGoalPoint();
  tf::Vector3 start, findStart;
  tf::StampedTransform odomToCar;
  if(getTFTransform("odom", "base_footprint", odomToCar)) {
    start = odomToCostmap(odomToCar.inverse() * tf::Vector3(0, 0, 0));
  } else {
    start = odomToCostmap(pointToVector(globalPath_.poses[0].pose.position));
  }
  findStart = start;

  // transform previous path to current time costmap coordinate
  std::vector<tf::Vector3> prevPath;
  int replanStep = 10;
  int prevStartIndex = 0;

  tf::StampedTransform mapToOdom;
  if(previousPath_.size() > replanStep && getTFTransform("map", "odom", mapToOdom)) {
    double minDistance = 1e6;
    for (int i = 0; i < previousPath_.size(); i++) {
      auto odomPoint = mapToOdom * previousPath_[i];
      auto costmapPoint = odomToCostmap(odomPoint);
      prevPath.push_back(odomPoint);

      // find current start point in previous path
      auto distance = costmapPoint.distance2(start);
      if (distance < minDistance) {
        prevStartIndex = i;
        minDistance = distance;
      }
      if(costmapPoint.x() >= 0 && costmapPoint.x() < width_ && costmapPoint.y() >= 0 && costmapPoint.y() < height_
         && costmap_((int)costmapPoint.y(), (int)costmapPoint.x()) >= obstacleThreshold_) {
        break;
      }
    }
    prevPath.erase(prevPath.begin(), prevPath.begin() + prevStartIndex);

    if(replanStep < prevPath.size()) {
      findStart = odomToCostmap(prevPath[prevPath.size() - replanStep]);
      prevPath.erase(prevPath.end() - replanStep - 1, prevPath.end()); // remove tail points
    } else {
      prevPath.clear();
    }

    visualization_msgs::Marker markerMsg;
    markerMsg.header.frame_id = "odom";
    markerMsg.ns = "Markers";
    markerMsg.action = visualization_msgs::Marker::ADD;
    markerMsg.type = visualization_msgs::Marker::POINTS;
    markerMsg.pose.orientation.w = 1.0;
    markerMsg.scale.x = 0.2;
    markerMsg.scale.y = 0.2;

    markerMsg.points.push_back(vectorToPoint(costmapToOdom(start)));
    markerMsg.colors.push_back(makeColor(1, 0, 0, 1));

    markerMsg.points.push_back(vectorToPoint(costmapToOdom(goal)));
    markerMsg.colors.push_back(makeColor(0, 1, 0, 1));

    markerMsg.points.push_back(vectorToPoint(prevPath[prevStartIndex]));
    markerMsg.colors.push_back(makeColor(1, 1, 0, 1));

    markerMsg.points.push_back(vectorToPoint(costmapToOdom(findStart)));
    markerMsg.colors.push_back(makeColor(0, 1, 1, 1));

    markerPub_.publish(markerMsg);
  }

//  std::cout << "Prev Index: " << prevStartIndex << ", Find Index: " << findStartIndex << std::endl;
//  std::cout << "Start is: " << start.x() << ", " << start.y() << ", Goal is: " << goal.x() << ", " << goal.y() << std::endl;

  auto optimalPath = findOptimalPath(findStart, goal);
  prevPath.insert(prevPath.end(), optimalPath.begin(), optimalPath.end());

  previousPath_.clear();
  if(getTFTransform("map", "odom", mapToOdom)) {
    auto odomToMap = mapToOdom.inverse();
    for(const auto &point : prevPath) {
      previousPath_.push_back(odomToMap * point);
    }
  }

  return prevPath;
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
    A(i, 0) = 1.0;

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
      A(j, i + 1) = A(j, i) * xvals(j);
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

bool BorderAdjustment::getBorder(const std::vector<tf::Vector3> &path,
    std::vector<tf::Vector3> &left, std::vector<tf::Vector3> &right) {
  if(path.empty()) return false;
  auto start = *path.cbegin();
  auto goal = *(path.cend() - 1);
  int max = width_ * height_;

  tf::StampedTransform odomToCar;
  if(!getTFTransform("odom", "base_footprint", odomToCar)) {
    return false;
  }
  auto carToOdom = odomToCar.inverse();

//  std::vector<double> xs;
//  std::vector<double> ys;
//  for(int i = 0; i < path.size(); i+=2) {
//    auto pos = odomToCar * path[i];
//    if(pos.x() >= 0) {
//      xs.push_back(pos.x());
//      ys.push_back(pos.y());
//    }
//  }
//  if(xs.size() < 4) {
//    return false;
//  }
//  Eigen::Map<Eigen::VectorXd> x_veh(xs.data(), xs.size());
//  Eigen::Map<Eigen::VectorXd> y_veh(ys.data(), ys.size());
//  auto coeffs = polyfit(x_veh, y_veh, 4);

  // translate to car coordinate
//  TableInterpolation path_interpolator(InterpolateMode::Linear, BoundaryType::Clamp);
//  std::map<double, double> table;
  for(const auto &point : path) {
    auto pos = odomToCar * point;
    if(pos.x() < 0) continue;

//    double slope = 0.0;
//    for (int i = 1; i < coeffs.size(); i++)
//    {
//      slope += i*coeffs[i] * pow(pos.x(), i-1); // f'(x0)
//    }
    double theta = M_PI / 2;
    // std::cout << "Slope: " << slope << ", Theta: " << theta << std::endl;

    // table[pos.x()] = pos.y();
    auto cur = pos;
    double direction = -1;
    bool foundBorder = false;
    while(true) {
      auto costmapPos = odomToCostmap(carToOdom * cur);
      if(foundBorder || costmapPos.x() < 0 || costmapPos.y() < 0 || costmapPos.x() >= width_ || costmapPos.y() >= height_) {
        foundBorder = false;
        if(direction > 0) {
          break;
        } else {
          direction = 1;
          cur = pos;
          continue;
        }
      }
      if(costmap_((int)costmapPos.y(), (int)costmapPos.x()) >= obstacleThreshold_) {
        (direction > 0 ? &right : &left)->push_back(cur);
        foundBorder = true;
      }
      // assert((direction < 0 && theta >= 0) || (direction > 0 && theta <= 0));
      cur.setY(cur.y() + direction * resolution_ * sin(theta));
      cur.setX(cur.x() + direction * resolution_ * cos(theta));
    }
  }
  return true;
  //path_interpolator.setPoints(table);
}

BorderAdjustment::~BorderAdjustment() {
  delete[] visited_;
  delete[] dist_;
  delete[] prev_;
}

bool BorderAdjustment::getTFTransform(const std::string &from, const std::string &target, tf::StampedTransform &transform) {
  try {
    tfListener_.waitForTransform(target, from, ros::Time(0), ros::Duration(10.0));
    tfListener_.lookupTransform(target, from, ros::Time(0), transform);
  } catch(const tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  return true;
}

