//
// Created by yenkn on 19-8-16.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <racecar_core/point.h>

ros::Publisher *scan_pub_;
double pitch_, roll_, yaw_;

void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
}

void scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
  auto scanMsg = *msg;

  for(int i = 0; i < msg->ranges.size(); i++) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] >= msg->range_max) continue;
    auto angle = msg->angle_min + i * msg->angle_increment;
    double x = msg->ranges[i]*cos(angle), y = msg->ranges[i]*sin(angle);
    scanMsg.ranges[i] = (float)hypot(x*cos(pitch_), y*cos(roll_));
    auto height = (float)(x*sin(pitch_) + y*cos(roll_));
    scanMsg.intensities[i] = height;
  }

  scan_pub_->publish(scanMsg);
}

void timerCallback(const ros::TimerEvent &evt) {
  std::cout << "roll: " << roll_ << ", pitch: " << pitch_ << ", yaw: " << yaw_ << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_calibration");
  ros::NodeHandle nh;
  auto imu_sub = nh.subscribe("/imu_data", 1, imuCallback);
  auto scan_sub = nh.subscribe("/scan", 1, scanCallback);

  auto scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_calib", 10);
  scan_pub_ = &scan_pub;

  auto timer = nh.createTimer(ros::Duration(1), timerCallback);

  ros::spin();
  return 0;
}