//
// Created by yenkn on 19-3-18.
//

#ifndef PROJECT_MPC_NODE_H
#define PROJECT_MPC_NODE_H

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "mpc.h"

#include "border_adjustment_ros.h"

using namespace std;
using namespace Eigen;

class MPCNode
{
public:
  MPCNode();
  int get_thread_numbers();

private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_amcl;
  ros::Publisher carPathPub_, _pub_mpctraj, _pub_mpcstate, _pub_twist;
  ros::Timer _timer;
  tf::TransformListener _tf_listener;

  geometry_msgs::Point goal_;
  nav_msgs::Odometry odom_;
  nav_msgs::Path globalPath_;
  std::vector<tf::Vector3> carPath_;
  geometry_msgs::PoseWithCovarianceStamped amclPose_;

  string mapFrame_, _odom_frame, carFrame_;
  tf::StampedTransform mapToCar_;
  int carIndex_ = 0;

  MPC _mpc;
  map<string, double> _mpc_params;
  double _mpc_steps, _ref_cte, _ref_epsi, _ref_vel, _w_cte, _w_epsi, _w_vel,
      _w_delta, _w_accel, _w_delta_d, _w_accel_d, _max_steering, _max_throttle, _bound_value;

  double _Lf, _dt, steering_, throttle_, _speed, _max_speed;
  double pathLength_, _goalRadius, _waypointsDist;
  int _controller_freq, _downSampling, _thread_numbers;
  bool goalReceived_, goalReached_, pathComputed_, _debug_info, _delay_mode, pathRecived_ = false;

  vector<tf::Vector3> samplePath();

  void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
  void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
  void controlLoopCB(const ros::TimerEvent&);

};

#endif //PROJECT_MPC_NODE_H
