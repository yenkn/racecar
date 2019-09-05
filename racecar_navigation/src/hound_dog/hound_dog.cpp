//
// Created by yenkn on 19-8-25.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "laser_line/laser_feature_ros.h"
#include <racecar_core/point.h>

class HoundDog {
public:
  HoundDog(): nh_("~"), laserFeature_("hound_line") {
    scanSub_ = nh_.subscribe("/scan", 1, &HoundDog::scanCallback, this);
    cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

    nh_.param("speed_p", speedP_, 1.0);
    nh_.param("error_p", errorP_, 1.0);
    nh_.param("follow_distance", followDistance_, 0.5);
    nh_.param("max_distance", maxDistance_, 3.0);

  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber scanSub_;
  ros::Publisher cmdPub_;
  laser_line::LaserFeatureROS laserFeature_;
  double speedP_ = 1.0, errorP_ = 1.0, followDistance_ = 0.5, maxDistance_ = 3.0;
  Point2D lastPoint_;

  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    auto laserLines = laserFeature_.processScan(msg);

    Point2D goalPoint;
    bool detected = false;
    double mindistance = FLT_MAX;
    for(auto &line: laserLines) {
      Point2D pt((line.x1 + line.x2) / 2, (line.y1 + line.y2) / 2);
      if(pt.length() >= maxDistance_ || pt.x < 0 || pt.distance(lastPoint_) > 1.0) {
        continue;
      }
      if(pt.length() < mindistance) {
        mindistance = pt.length();
        goalPoint = pt;
        lastPoint_ = pt;
        detected = true;
      }
    }

    if(!detected) return;

    double error = atan2(goalPoint.y, goalPoint.x);
    double speedError = goalPoint.length() - 0.5;
    if(speedError < 0.0) speedError = 0.0;

    geometry_msgs::Twist cmd;
    cmd.linear.x = speedError * speedP_;
    cmd.angular.z = error * errorP_;
    cmdPub_.publish(cmd);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "hound_dog");
  HoundDog dog;
  ros::spin();
  return 0;
}