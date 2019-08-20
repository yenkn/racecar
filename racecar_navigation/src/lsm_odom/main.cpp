//
// Created by yenkn on 19-5-14.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

ros::Publisher *odomPub;
geometry_msgs::PoseStamped lastPos;

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose = msg->pose;

  if(!lastPos.header.stamp.isValid()) {
    lastPos = *msg;
    return;
  }

  double lastYaw = tf::getYaw(lastPos.pose.orientation);
  // calculate relative position
  double dx = cos(lastYaw)*(msg->pose.position.x - lastPos.pose.position.x) + sin(lastYaw)*(msg->pose.position.y - lastPos.pose.position.y);
  double dy = -sin(lastYaw)*(msg->pose.position.x - lastPos.pose.position.x) + cos(lastYaw)*(msg->pose.position.y - lastPos.pose.position.y);

  auto dt = msg->header.stamp.toSec() - lastPos.header.stamp.toSec();
  auto vx = dx / dt;
  auto vy = dy / dt;
  auto va = (tf::getYaw(msg->pose.orientation) - lastYaw) / dt;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = va;

  odomPub->publish(odom);
  lastPos = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lsm_odom");

  ROS_INFO("laser_scan_matcher odometry started.");

  ros::NodeHandle nh("~");

  ros::Subscriber poseSub = nh.subscribe("/pose_stamped", 1, poseCallback);
  auto pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);
  odomPub = &pub;
  ros::spin();
  return 0;
}