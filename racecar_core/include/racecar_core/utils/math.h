//
// Created by yenkn on 19-7-3.
//

#ifndef SRC_MATH_H
#define SRC_MATH_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <nav_msgs/Path.h>
#include <racecar_core/point.h>

namespace utils {

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
    A(i, 0) = 1.0;

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++)
      A(j, i + 1) = A(j, i) * xvals(j);
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


/***
 * Get distance between point and line segment
    https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 * @return
 */
double distaceToLineSegment(const Point2D &point, const Point2D &startPoint, const Point2D &endPoint, Point2D *leastPoint = nullptr) {
  double A = point.x - startPoint.x;
  double B = point.y - startPoint.y;
  double C = endPoint.x - startPoint.x;
  double D = endPoint.y - startPoint.y;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = -1;
  if (len_sq != 0) //in case of 0 length line
    param = dot / len_sq;

  double xx, yy;

  if (param < 0) {
    xx = startPoint.x;
    yy = startPoint.y;
  } else if (param > 1) {
    xx = endPoint.x;
    yy = endPoint.y;
  } else {
    xx = startPoint.x + param * C;
    yy = startPoint.y + param * D;
  }

  if(leastPoint != nullptr) {
    leastPoint->x = xx;
    leastPoint->y = yy;
  }

  double dx = point.x - xx;
  double dy = point.y - yy;
  return sqrt(dx * dx + dy * dy);
}

/***
 * Get projection distance between point and line
    https://stackoverflow.com/questions/40970478/python-3-5-2-distance-from-a-point-to-a-line
 * @return
 */
double distanceToLine(const Point2D &point, const Point2D &startPoint, const Point2D &endPoint) {
  double px = point.x, py = point.y;
  double x_diff = endPoint.x - startPoint.x;
  double y_diff = endPoint.y - startPoint.y;
  double num = fabs(y_diff * px - x_diff * py + endPoint.x * startPoint.y - endPoint.y * startPoint.x);
  double den = sqrt(y_diff * y_diff + x_diff * x_diff);
  return num / den;
}

/**
 * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_an_equation
 * @param point
 * @param startPoint
 * @param endPoint
 * @return
 */
Point2D lineProjectPoint(const Point2D &point, const Point2D &startPoint, const Point2D &endPoint) {
  double a = startPoint.y - endPoint.y, b = endPoint.x - startPoint.x, c = startPoint.x * endPoint.y - endPoint.x * startPoint.y;
  double x = (b*(b*point.x - a*point.y) - a*c)/(a*a+b*b);
  double y = (a*(-b*point.x + a*point.y) - b*c)/(a*a+b*b);
  return Point2D(x, y);
}

bool isLineIntersect(const Point2D &p1, const Point2D &p2, const Point2D &q1, const Point2D &q2) {
  return (((q1.x - p1.x) * (p2.y - p1.y) - (q1.y - p1.y) * (p2.x - p1.x))
          * ((q2.x - p1.x) * (p2.y - p1.y) - (q2.y - p1.y) * (p2.x - p1.x)) < 0)
         &&
         (((p1.x - q1.x) * (q2.y - q1.y) - (p1.y - q1.y) * (q2.x - q1.x))
          * ((p2.x - q1.x) * (q2.y - q1.y) - (p2.y - q1.y) * (q2.x - q1.x)) < 0);
}


// * Finds the center of the circle passing through the points p1, p2 and p3.
// * (NaN, NaN) will be returned if the points are colinear.
tf::Vector3 CircleCenterFrom3Points (tf::Vector3 p1, tf::Vector3 p2, tf::Vector3 p3) {
  double temp = p2.length2();
  double bc = (p1.length2() - temp)/2.0f;
  double cd = (temp - p3.length2())/2.0f;
  double det = (p1.x()-p2.x())*(p2.y()-p3.y())-(p2.x()-p3.x())*(p1.y()-p2.y());
//  if (fabs(det) < 1.0e-6) {
//    return { NAN, NAN, 0 };
//  }
  det = 1/det;
  return { (bc*(p2.y()-p3.y())-cd*(p1.y()-p2.y()))*det, ((p1.x()-p2.x())*cd-(p2.x()-p3.x())*bc)*det, 0 };
}

double calculatePointCurvature(const nav_msgs::Path &mapPath, int index, int stepSize) {
  auto cc = CircleCenterFrom3Points(
      tf::Vector3(mapPath.poses[index-stepSize].pose.position.x, mapPath.poses[index-stepSize].pose.position.y, 0.0),
      tf::Vector3(mapPath.poses[index].pose.position.x, mapPath.poses[index].pose.position.y, 0.0),
      tf::Vector3(mapPath.poses[index+stepSize].pose.position.x, mapPath.poses[index].pose.position.y, 0.0));
  auto radius = (cc - tf::Vector3(mapPath.poses[index].pose.position.x, mapPath.poses[index].pose.position.y, 0.0)).length();
  radius = std::max(radius, 0.0001);
  return 1 / radius;
}

std::vector<double> conv1d(const std::vector<double> &data, const std::vector<double> &kernel) {
  std::vector<double> result(data.size(), 0.0);
  int size = int((kernel.size() - 1)/2);
  for(int i = 0; i < data.size(); i++) {
    double conv = 0;
    for(int j = -size; j < size; j++) {
      if(i - j < 0 or i - j >= data.size())
        continue;
      conv += data[i-j] * kernel[size+j];
    }
    result[i] = conv;
  }
  return result;
}

std::vector<double> maximumFilter(const std::vector<double> &data, int kernelSize) {
  std::vector<double> result(data.size(), 0.0);
  int size = int((kernelSize - 1)/2);
  for(int i = 0; i < data.size(); i++) {
    double maximum = 0;
    for(int j = -size; j < size; j++) {
      if(i - j < 0 or i - j >= data.size())
        continue;
      if(data[i-j] > maximum) maximum = data[i-j];
    }
    result[i] = maximum;
  }
  return result;
}

}

#endif //SRC_MATH_H
