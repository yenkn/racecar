//
// Created by yenkn on 19-5-16.
//

#ifndef PROJECT_Point2D_H
#define PROJECT_Point2D_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class Point2D
{
public:
  static const int DIM = 2;

  Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
  Point2D(const Point2D& p) : x(p.x), y(p.y) {}
  static Point2D fromPoolarCoords(const double r, const double phi) { return Point2D(r * cos(phi), r * sin(phi)); }

  inline geometry_msgs::Point toPoint() const {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = 0;
    return pt;
  }

  inline geometry_msgs::Pose toPose() const {
    geometry_msgs::Pose pt;
    pt.position = toPoint();
    pt.orientation.w = 1.0;
    return pt;
  }

   geometry_msgs::PoseStamped toPoseStamped(const std::string &frame_id) const {
    geometry_msgs::PoseStamped pt;
    pt.pose = toPose();
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = frame_id;
    return pt;
  }

  double length()        const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
  double lengthSquared() const { return pow(x, 2.0) + pow(y, 2.0); }
  double angle()         const { return atan2(y, x); }
  double angleDeg()      const { return 180.0 * atan2(y, x) / M_PI; }
  double dot(const Point2D& p)   const { return x * p.x + y * p.y; }
  double cross(const Point2D& p) const { return x * p.y - y * p.x; }

  double distance(const Point2D &rhs) const { return sqrt(distanceSquared(rhs)); }
  double distanceSquared(const Point2D &rhs) const { return pow(x-rhs.x, 2.0) + pow(y-rhs.y, 2.0); }

  Point2D normalized() { return (length() > 0.0) ? *this / length() : *this; }
  Point2D reflected(const Point2D& normal) const { return *this - 2.0 * normal * (normal.dot(*this)); }
  Point2D perpendicular() const { return Point2D(-y, x); }
  Point2D rotate(double theta) { return Point2D(cos(theta) * x - sin(theta) * y, sin(theta) * x + cos(theta) * y); }

  inline double operator [](int idx) {
    return idx == 0 ? x : (idx == 1 ? y : 0);
  }

  inline double operator [](int idx) const {
    return idx == 0 ? x : (idx == 1 ? y : 0);
  }

  friend Point2D operator+ (const Point2D& p1, const Point2D& p2) { return Point2D(p1.x + p2.x, p1.y + p2.y); }
  friend Point2D operator- (const Point2D& p1, const Point2D& p2) { return Point2D(p1.x - p2.x, p1.y - p2.y); }
  friend Point2D operator* (const double f, const Point2D& p)  { return Point2D(f * p.x, f * p.y); }
  friend Point2D operator* (const Point2D& p, const double f)  { return Point2D(f * p.x, f * p.y); }
  friend Point2D operator/ (const Point2D& p, const double f)  { return (f != 0.0) ? Point2D(p.x / f, p.y / f) : Point2D(); }

  Point2D operator- () { return Point2D(-x, -y); }
  Point2D operator+ () { return Point2D( x,  y); }

  Point2D& operator=  (const Point2D& p) { if (this != &p) { x = p.x; y = p.y; } return *this; }
  Point2D& operator+= (const Point2D& p) { x += p.x; y += p.y; return *this; }
  Point2D& operator-= (const Point2D& p) { x -= p.x; y -= p.y; return *this; }

  friend bool operator== (const Point2D& p1, const Point2D& p2) { return (p1.x == p2.x && p1.y == p2.y); }
  friend bool operator!= (const Point2D& p1, const Point2D& p2) { return !(p1 == p2); }
  friend bool operator<  (const Point2D& p1, const Point2D& p2) { return (p1.lengthSquared() < p2.lengthSquared()); }
  friend bool operator<= (const Point2D& p1, const Point2D& p2) { return (p1.lengthSquared() <= p2.lengthSquared()); }
  friend bool operator>  (const Point2D& p1, const Point2D& p2) { return (p1.lengthSquared() > p2.lengthSquared()); }
  friend bool operator>= (const Point2D& p1, const Point2D& p2) { return (p1.lengthSquared() >= p2.lengthSquared()); }
  friend bool operator!  (const Point2D& p1) { return (p1.x == 0.0 && p1.y == 0.0); }

  friend std::ostream& operator<<(std::ostream& out, const Point2D& p)
  { out << "(" << p.x << ", " << p.y << ")"; return out; }

  double x;
  double y;
};

#endif //PROJECT_Point2D_H
