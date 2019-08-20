//
// Created by yenkn on 19-3-20.
//

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

#include <racecar_core/utils/laser.h>

ros::Publisher *emergencyPub = nullptr;
double stopThreshold = 0.0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  auto forwardRange = utils::cutLaser(*scan, utils::degreeToRadius(-15), utils::degreeToRadius(15));
  std::sort(forwardRange.begin(), forwardRange.end());
  auto len = int(forwardRange.size() * 0.1);
  auto distance = std::accumulate(forwardRange.cbegin(), forwardRange.cbegin() + len, 0.0) / len;

  if(distance <= scan->range_max * stopThreshold) {
    ROS_INFO("FORWARD AVG: %0.2f < %0.2f, based on %d points, doing emergency stop.", distance, scan->range_max * stopThreshold, len);
    std_msgs::Int32 mode;
    mode.data = 1;
    emergencyPub->publish(mode);
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "guardian");

  ROS_INFO("Racecar Guardian started.");

  ros::NodeHandle nh("~");
  nh.param("stop_threshold", stopThreshold, 0.03);

  ros::Subscriber scanSub = nh.subscribe("/scan", 1, scanCallback);
  ros::Subscriber imuSub = nh.subscribe("/imu_data", 1, imuCallback);
  auto pub = nh.advertise<std_msgs::Int32>("/emergency_mode", 1);
  emergencyPub = &pub;
  ros::spin();
  return 0;
}