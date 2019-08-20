/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include <map>
#include <math.h>

#include "mpc_node.h"
#include <racecar_msgs/MPCState.h>
#include <racecar_core/utils/math.h>
#include <racecar_core/utils/tf.h>
#include <tf/transform_datatypes.h>

MPCNode::MPCNode()
{
  //Private parameters handler
  ros::NodeHandle pn("~");

  //Parameters for control loop
  pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
  pn.param("debug_info", _debug_info, false);
  pn.param("delay_mode", _delay_mode, true);
  pn.param("max_speed", _max_speed, 2.0); // unit: m/s
  pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
  pn.param("path_length", pathLength_, 8.0); // unit: m
  pn.param("goal_radius", _goalRadius, 0.5); // unit: m
  pn.param("controller_freq", _controller_freq, 10);
  pn.param("vehicle_Lf", _Lf, 0.25); // distance between the front of the vehicle and its center of gravity
  _dt = double(1.0/_controller_freq); // time step duration dt in s

  //Parameter for MPC solver
  pn.param("mpc_steps", _mpc_steps, 20.0);
  pn.param("mpc_ref_cte", _ref_cte, 0.0);
  pn.param("mpc_ref_epsi", _ref_epsi, 0.0);
  pn.param("mpc_ref_vel", _ref_vel, 1.5);
  pn.param("mpc_w_cte", _w_cte, 100.0);
  pn.param("mpc_w_epsi", _w_epsi, 100.0);
  pn.param("mpc_w_vel", _w_vel, 100.0);
  pn.param("mpc_w_delta", _w_delta, 100.0);
  pn.param("mpc_w_accel", _w_accel, 50.0);
  pn.param("mpc_w_delta_d", _w_delta_d, 0.0);
  pn.param("mpc_w_accel_d", _w_accel_d, 0.0);
  pn.param("mpc_max_steering", _max_steering, 0.523); // Maximal steering radian (~30 deg)
  pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
  pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

  std::string globalTopic, goalTopic;

  //Parameter for topics & Frame name
  pn.param<std::string>("global_path_topic", globalTopic, "/move_base/TrajectoryPlannerROS/global_plan" );
  pn.param<std::string>("goal_topic", goalTopic, "/move_base_simple/goal" );
  pn.param<std::string>("map_frame", mapFrame_, "map" );
  pn.param<std::string>("odom_frame", _odom_frame, "odom");
  pn.param<std::string>("car_frame", carFrame_, "base_link" );

  //Display the parameters
  cout << "\n===== Parameters =====" << endl;
  cout << "debug_info: "  << _debug_info << endl;
  cout << "delay_mode: "  << _delay_mode << endl;
  cout << "vehicle_Lf: "  << _Lf << endl;
  cout << "frequency: "   << _dt << endl;
  cout << "mpc_steps: "   << _mpc_steps << endl;
  cout << "mpc_ref_vel: " << _ref_vel << endl;
  cout << "mpc_w_cte: "   << _w_cte << endl;
  cout << "mpc_w_epsi: "  << _w_epsi << endl;
  cout << "mpc_max_steering: "  << _max_steering << endl;

  //Publishers and Subscribers
  _sub_odom   = _nh.subscribe("/odometry/filtered", 1, &MPCNode::odomCB, this);
  _sub_path   = _nh.subscribe( globalTopic, 1, &MPCNode::pathCB, this);
  _sub_goal   = _nh.subscribe( goalTopic, 1, &MPCNode::goalCB, this);
  _sub_amcl   = _nh.subscribe("/amcl_pose", 5, &MPCNode::amclCB, this);
  carPathPub_  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC
  _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
  _pub_mpcstate   = _nh.advertise<racecar_msgs::MPCState>("/mpc_state", 1);// debug MPC state output
  _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)

  //Timer
  _timer = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz

  //Init variables
  goalReceived_ = false;
  goalReached_  = false;
  pathComputed_ = false;
  throttle_ = 0.0;
  steering_ = 0.0;
  _speed = 0.0;

  //Init parameters for MPC object
  _mpc_params["DT"] = _dt;
  _mpc_params["LF"] = _Lf;
  _mpc_params["STEPS"]    = _mpc_steps;
  _mpc_params["REF_CTE"]  = _ref_cte;
  _mpc_params["REF_EPSI"] = _ref_epsi;
  _mpc_params["REF_V"]    = _ref_vel;
  _mpc_params["W_CTE"]    = _w_cte;
  _mpc_params["W_EPSI"]   = _w_epsi;
  _mpc_params["W_V"]      = _w_vel;
  _mpc_params["W_DELTA"]  = _w_delta;
  _mpc_params["W_A"]      = _w_accel;
  _mpc_params["W_DDELTA"] = _w_delta_d;
  _mpc_params["W_DA"]     = _w_accel_d;
  _mpc_params["MAXSTR"]   = _max_steering;
  _mpc_params["MAXTHR"]   = _max_throttle;
  _mpc_params["BOUND"]    = _bound_value;
  _mpc.LoadParams(_mpc_params);

}


// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
  return _thread_numbers;
}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  odom_ = *odomMsg;
  utils::getTFTransform(_tf_listener, mapFrame_, carFrame_, mapToCar_);
  if(goalReceived_ && !goalReached_) samplePath();
}

vector<tf::Vector3> MPCNode::samplePath() {
  vector<tf::Vector3> carPath;

  double minDistance = FLT_MAX;
  auto carPose = mapToCar_.inverse() * tf::Vector3(0, 0, 0);
  for(int i = std::max(0, carIndex_ - 10); i < globalPath_.poses.size(); i++) {
    double distance = carPose.distance(utils::pointToVector(globalPath_.poses[i].pose.position));
    if(distance < minDistance) {
      minDistance = distance;
      carIndex_ = i;
    }
  }

  // Cut and downsampling the path
  double downSampling = pathLength_ / 10.0; // sample 10 points
  double totalLength = 0.0;
  double sampling = 0.0;
  tf::Vector3 lastPt;

  for(int i = carIndex_; i < globalPath_.poses.size() && totalLength < pathLength_; i++)
  {
    auto pt = mapToCar_ * utils::pointToVector(globalPath_.poses[i].pose.position);
    if(i == carIndex_) {
      carPath.push_back(pt);
      lastPt = pt;
      continue;
    }

    double distance = pt.distance(lastPt);
    totalLength += distance;
    sampling += distance;

    if(sampling >= downSampling) {
      carPath.push_back(pt);
      sampling = 0.0;
    }

    lastPt = pt;
  }

  if(carPath.size() >= 6)
  {
    pathComputed_ = true;
    carPath_ = carPath;

    auto pathMsg = nav_msgs::Path();
    pathMsg.header.frame_id = carFrame_;
    pathMsg.header.stamp = ros::Time::now();
    for(auto pt : carPath) {
      geometry_msgs::PoseStamped pose;
      pose.header = pathMsg.header;
      pose.pose.position = utils::vectorToPoint(pt);
      pose.pose.orientation.w = 1;
      pathMsg.poses.push_back(pose);
    }
    carPathPub_.publish(pathMsg);
  }

  return carPath;
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
  if(goalReceived_ && !goalReached_ && !pathRecived_)
  {
    pathRecived_ = true;
    globalPath_ = *pathMsg;
    carIndex_ = 0;
    samplePath();
  }
}

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
  goal_ = goalMsg->pose.position;
  goalReceived_ = true;
  goalReached_ = false;
  ROS_INFO("Goal Received!");
}


// Callback: Check if the car is inside the goal area or not
void MPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
  amclPose_ = *amclMsg;
  if(goalReceived_)
  {
    double car2goal_x = goal_.x - amclMsg->pose.pose.position.x;
    double car2goal_y = goal_.y - amclMsg->pose.pose.position.y;
    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
    if(dist2goal < _goalRadius)
    {
      goalReached_ = true;
      goalReceived_ = false;
      pathComputed_ = false;
      ROS_INFO("Goal Reached !");
    }
  }
}


// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{
  if(goalReceived_ && !goalReached_ && pathComputed_ ) //received goal & goal not reached
  {
    /*
      Update system states: X=[x, y, psi, v]
      x, y — position.
      psi(ψ) — orientation.
      v — velocity.
    */
    const double psi = tf::getYaw(amclPose_.pose.pose.orientation);
    const double v = odom_.twist.twist.linear.x; //twist: body fixed frame
    // Update system inputs: U=[steering, throttle]
    const double steering = steering_;  // radian
    const double throttle = throttle_; // accel: >0; brake: <0
    const double dt = _dt;
    const double Lf = _Lf;

    // Waypoints related parameters
    const size_t N = carPath_.size(); // Number of waypoints

    // Convert to the vehicle coordinate system
    VectorXd x_veh(N);
    VectorXd y_veh(N);
    for(int i = 0; i < N; i++)
    {
      x_veh[i] = carPath_[i].x();
      y_veh[i] = carPath_[i].y();
    }

    /*
      Fit waypoints
      cte — cross-track error. The difference between the trajectory defined by the waypoints and the current vehicle position y in the coordinate space of the vehicle.
      epsi (eψ) — orientation error.
    */
    auto coeffs = utils::polyfit(x_veh, y_veh, 3);

    const double cte  = utils::polyeval(coeffs, 0.0);
    const double epsi = atan(coeffs[1]);
    VectorXd state(6);
    if(_delay_mode)
    {
      // Kinematic model is used to predict vehicle state at the actual
      // moment of control (current time + delay dt)
      const double px_act = v * dt;
      const double py_act = 0;
      const double psi_act = v * steering * dt / Lf;
      const double v_act = v + throttle * dt;
      const double cte_act = cte + v * sin(epsi) * dt; // Cross Track Error
      const double epsi_act = -epsi + psi_act;
      state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
    }
    else
    {
      state << 0, 0, 0, v, cte, epsi;
    }

    // Solve MPC Problem
    vector<double> mpc_results = _mpc.Solve(state, coeffs);

    // MPC result (all described in car frame)
    steering_ = mpc_results[0]; // radian
    throttle_ = mpc_results[1]; // acceleration
    _speed = v + throttle_*dt;  // speed
    // cout << "Speed: " << _speed << "\tmax: " << _max_speed << "\tv:" << v << "\tthrottle: " << throttle_ << endl;
    if (_speed >= _max_speed)
      _speed = _max_speed;
    if(_speed <= 0.0)
      _speed = 0.0;
//    if(throttle_ > 0.1 && _speed < 0.5) {
//      _speed = 0.5;
//    }

    if(_debug_info)
    {
      racecar_msgs::MPCState debug_state;
      debug_state.v = v;
      debug_state.cte = cte;
      debug_state.epsi = epsi;
      debug_state.psi = psi;
      debug_state.throttle = throttle;
      std::copy(coeffs.data(), coeffs.data() + coeffs.size(), std::back_inserter(debug_state.coeffs));
      _pub_mpcstate.publish(debug_state);
    }

    // Display the MPC predicted trajectory
    nav_msgs::Path mpc_traj;
    mpc_traj.header.frame_id = carFrame_; // points in car coordinate
    mpc_traj.header.stamp = ros::Time::now();
    for(int i=0; i<_mpc.mpc_x.size(); i++)
    {
      geometry_msgs::PoseStamped tempPose;
      tempPose.header = mpc_traj.header;
      tempPose.pose.position.x = _mpc.mpc_x[i];
      tempPose.pose.position.y = _mpc.mpc_y[i];
      tempPose.pose.orientation.w = 1.0;
      mpc_traj.poses.push_back(tempPose);
    }
    // publish the mpc trajectory
    _pub_mpctraj.publish(mpc_traj);

  }
  else
  {
    steering_ = 0.0;
    throttle_ = 0.0;
    _speed = 0.0;
    if(goalReached_ && goalReceived_)
      cout << "Goal Reached !" << endl;
  }

  // publish cmd
  geometry_msgs::Twist twistMsg;
  twistMsg.linear.x  = _speed;
  twistMsg.angular.z = steering_;
  _pub_twist.publish(twistMsg);
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "MPC_Node");
  MPCNode mpc_node;

  ROS_INFO("Waiting for global path msgs ~");
  ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
