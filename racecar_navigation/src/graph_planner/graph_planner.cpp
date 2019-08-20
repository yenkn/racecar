//
// Created by yenkn on 19-7-14.
//

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <racecar_msgs/GetCheckline.h>
#include "graph_planner.h"
#include <racecar_core/utils/graph.hpp>
#include <racecar_core/utils/serialization.h>
#include <fstream>

GraphPlanner::GraphPlanner(): nh_("~") {
  goalSub_ = nh_.subscribe("/move_base_simple/goal", 1, &GraphPlanner::goalCallback, this);
  amclSub_ = nh_.subscribe("/amcl_pose", 1, &GraphPlanner::amclCallback, this);
  obstacleSub_ = nh_.subscribe("/graph_detector/event", 100, &GraphPlanner::obstacleCallback, this);
  pointSub_ = nh_.subscribe("/clicked_point", 1, &GraphPlanner::pointCallback, this);

  checklineClient_ = nh_.serviceClient<racecar_msgs::GetCheckline>("/checkline");

  pathPub_ = nh_.advertise<nav_msgs::Path>("plan", 1, true);

  nh_.param("uturn_margin", uTurnMargin_, 0.4);
  nh_.param("sample_times", sampleTimes_, 1000);
  nh_.param("path_spacing", pathSpacing_, 0.05);
  nh_.param("spline_file", splineFile_, std::string("/home/yenkn/catkin_ws/src/racecar_navigation/config/graph_path.txt"));
  nh_.param("merge_radius", mergeRadius_, 1.0);
  nh_.param("car_width", carWidth_, 0.5);
  nh_.param("lock_side_distance", lockSideDistance_, 1.0);

  std::vector<int> obstacleStages;
  nh_.param("obstacle_stages", obstacleStages, {});
  obstacleStages_ = std::set<int>(obstacleStages.begin(), obstacleStages.end());

  std::vector<double> goal;
  nh_.param("default_goal", goal, {});
  goal_ = goal.size() < 2 ? Point2D(0.0, -2.0) : Point2D(goal[0], goal[1]);

  markerCb_ = std::bind(&GraphPlanner::markerCallback, this, std::placeholders::_1);
  server_ = new interactive_markers::InteractiveMarkerServer("control", "", false);

  getCheckline();
  initializeSpline();
}

GraphPlanner::~GraphPlanner() {
  delete server_;
}

void GraphPlanner::initializeSpline() {
  if(utils::readPoints(splineFile_, checkPoints_)) {
    pathFound_ = true;
  } else {
    pathFound_ = makePath(checkPoints_);
    flushSpline();
  }

  if(pathFound_) {
    initializeMarkers();
    refreshPath();
  }
}

void GraphPlanner::flushSpline() {
  utils::writePoints(splineFile_, checkPoints_);
}

void GraphPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  goal_ = Point2D(msg->pose.position.x, msg->pose.position.y);

  if(!pathFound_) {
    pathFound_ = makePath(checkPoints_);
    flushSpline();
    initializeMarkers();
    refreshPath();
  } else {
    checkPoints_.back().x = goal_.x;
    checkPoints_.back().y = goal_.y;
    refreshPath();
  }
}

void GraphPlanner::pointCallback(const geometry_msgs::PointStampedConstPtr &msg) {
//  auto pt = Point2D(msg->point.x, msg->point.y);
//  for(int i = 1; i < checkPoints_.size(); i++) {
//    auto start = checkPoints_[i-1];
//    auto end = checkPoints_[i];
//    Point2D leastPt;
//    if(utils::distaceToLineSegment(pt, start, end, &leastPt) < 0.5 && leastPt != start && leastPt != end) {
//      checkPoints_.insert(std::next(checkPoints_.begin(), i), pt);
//    }
//    refreshPath();
//    flushSpline();
//  }
}

void GraphPlanner::obstacleCallback(const racecar_msgs::ObstacleEventConstPtr &msg) {
  return;
  if(checkline_.empty() && !getCheckline()) {
    ROS_ERROR("no checklines, unable to add obstacle");
    return;
  }
  if(!obstacleStages_.empty() && obstacleStages_.find(msg->obstacle.stage) == obstacleStages_.end()) {
    return;
  }
  auto id = msg->obstacle.id;
  if(msg->event == racecar_msgs::ObstacleEvent::ADD) {
    ROS_INFO("obstacle [%d]", id);
    obstacles_[id] = obstacle_t(msg->obstacle);
    selectSide(id);

    createInteractiveMarker(-id, obstacles_[id].control);
    server_->applyChanges();
  } else if(msg->event == racecar_msgs::ObstacleEvent::MODIFY) {
    obstacles_[id].obstacle = msg->obstacle;
    selectSide(id);
  }

  refreshPath();
}

void GraphPlanner::amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
}

bool GraphPlanner::getCheckline() {
  racecar_msgs::GetChecklineRequest req;
  racecar_msgs::GetChecklineResponse res;
  if(!checklineClient_.call(req, res) || res.checkline.points.empty()) {
    ROS_ERROR("unable to get checklines");
    return false;
  }
  checkline_.clear();
  for(int i = 0; i < res.checkline.points.size(); i+=2) {
    checkline_.push_back({
        Point2D(res.checkline.points[i].x, res.checkline.points[i].y),
        Point2D(res.checkline.points[i+1].x, res.checkline.points[i+1].y)
    });
  }
  uturnIndex_ = res.checkline.uturn_index;
  stages_ = utils::checklinesToStages(checkline_, uturnIndex_, Point2D(0, 0), goal_);
  ROS_INFO("got checklines");
  return true;
}

void GraphPlanner::createInteractiveMarker(int id, const Point2D &point)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = "map";
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  // oss << "pt_" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = std::string("Control Point ") + std::to_string(id);
  i_marker.pose.position.x = point.x;
  i_marker.pose.position.y = point.y;
  i_marker.pose.position.z = 0.01;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::SPHERE;
  box_marker.id = id;
  box_marker.scale.x = 0.1;
  box_marker.scale.y = 0.1;
  box_marker.scale.z = 0.1;
  box_marker.color.r = id >= 0 ? 0.0f : 1.0f;
  box_marker.color.g = id >= 0 ? 1.0f : 0.0f;
  box_marker.color.b = 0.0;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = 1;
  box_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  i_marker.controls.push_back(box_control);

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.5f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.5f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  server_->insert(i_marker);
  server_->setCallback(i_marker.name, markerCb_, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
}

void GraphPlanner::initializeMarkers() {
  for(int i = 0; i < checkPoints_.size(); i++) {
    createInteractiveMarker(i, checkPoints_[i]);
  }
  server_->applyChanges();
}

bool GraphPlanner::makePath(std::vector<Point2D> &path) {
  if(checkline_.empty() && !getCheckline()) {
    ROS_ERROR("no checklines, unable to make path");
    return false;
  }
  path = { Point2D(0, 0) };

  for(int i = 0; i < checkline_.size(); i++) {
    if(i == uturnIndex_ - 1) {
      Point2D prevPt = (checkline_[i][0] + checkline_[i][1]) / 2;
      Point2D uMiddle = (checkline_[uturnIndex_][0] + checkline_[uturnIndex_][1]) / 2;
      Point2D nextPt = (checkline_[uturnIndex_+1][0] + checkline_[uturnIndex_+1][1]) / 2;

      double turnRadius = fabs(nextPt.y - prevPt.y) / 2;
      Point2D center = checkline_[i][1];

      path.emplace_back(prevPt);
      path.emplace_back(center.x + turnRadius, uMiddle.y);
      path.emplace_back(nextPt);

      i = uturnIndex_ + 1;
    } else {
      path.push_back((checkline_[i][0] + checkline_[i][1]) / 2);
    }
  }

  path.push_back(goal_);

  return true;
}

void GraphPlanner::refreshPath() {
  auto points = getPoints();
  spline_ = tinyspline::BSpline(points.size(), 2, 2, TS_CLAMPED);
  std::vector<tinyspline::real> ctrlp = spline_.controlPoints();
  for(int i = 0; i < points.size(); i++) {
    ctrlp[i*2] = points[i].x;
    ctrlp[i*2+1] = points[i].y;
  }
  spline_.setControlPoints(ctrlp);

  for(int i = 0; i < checkPoints_.size(); i++) {
    geometry_msgs::Pose pose;
    pose.position = checkPoints_[i].toPoint();
    pose.orientation.w = 1.0;
    server_->setPose(std::to_string(i), pose);
  }

  for(auto &ob: obstacles_) {
    geometry_msgs::Pose pose;
    pose.position = ob.second.control.toPoint();
    pose.orientation.w = 1.0;
    server_->setPose(std::to_string(-ob.first), pose);
  }

  server_->applyChanges();

  nav_msgs::Path msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  std::vector<tinyspline::real> lastRes;
  for(int i = 0; i <= sampleTimes_; i++) {
    auto res = spline_.eval((double)i/sampleTimes_).result();

    if(i > 0) {
      double distance = hypot(res[1] - lastRes[1], res[0] - lastRes[0]);
      if (distance > pathSpacing_) {
        double ori = atan2(res[1] - lastRes[1], res[0] - lastRes[0]);
        int count = (int) (distance / pathSpacing_);
        for (int j = 1; j < count; j++) {
          geometry_msgs::PoseStamped pose;
          pose.header = msg.header;
          pose.pose.position.x = lastRes[0] + pathSpacing_ * j * cos(ori);
          pose.pose.position.y = lastRes[1] + pathSpacing_ * j * sin(ori);
          pose.pose.orientation.w = 1;
          msg.poses.push_back(pose);
        }
      }
    }

    lastRes = res;

    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = res[0];
    pose.pose.position.y = res[1];
    pose.pose.orientation.w = 1;
    msg.poses.push_back(pose);
  }
  pathPub_.publish(msg);
}

std::vector<Point2D> GraphPlanner::getPoints() {
  auto points = checkPoints_;
  int i = 1;

  for(auto &ob : obstacles_) {
    auto next = std::next(points.begin(), ob.second.obstacle.stage+i);
    points.insert(next, ob.second.control);
    i++;
  }
  auto iter = points.begin();
  while(iter != points.end()-1) {
    auto next = std::next(iter);
    if(iter->distance(*next) < mergeRadius_) {
      next->x = (iter->x + next->x) / 2;
      next->y = (iter->y + next->y) / 2;
      iter = points.erase(iter);
    } else {
      iter++;
    }
  }
  return points;
}

int GraphPlanner::selectSide(int obstacleId) {
  auto &ob = obstacles_[obstacleId];
  int side = ob.side;
  if(side != 0) {
    // update obstacle
    if(hypot(ob.obstacle.relativeMean.x, ob.obstacle.relativeMean.y) > lockSideDistance_) {
      // can switch side
      if ((side < 0 && ob.obstacle.leftward < carWidth_) || (side > 0 && ob.obstacle.rightward < carWidth_)) {
        side *= -1;
        ROS_INFO("obstacle[%d] switch to %d", obstacleId, side);
      }
    }
  } else {
    side = ob.obstacle.leftward > ob.obstacle.rightward ? -1 : 1;
  }

  Point2D pos = Point2D(ob.obstacle.mean.x, ob.obstacle.mean.y), projection;
  if (side < 0) {
    projection = utils::lineProjectPoint(pos, stages_[ob.obstacle.stage][0], stages_[ob.obstacle.stage][3]);
  } else {
    projection = utils::lineProjectPoint(pos, stages_[ob.obstacle.stage][1], stages_[ob.obstacle.stage][2]);
  }
  ob.control = projection * (2.0 / 3.0) + pos * (1.0 / 3.0);
  ob.side = side;

  return side;
}

void GraphPlanner::markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  int index = std::stoi(feedback->marker_name);

  if(index >= 0 && index < checkPoints_.size()) {
    checkPoints_[index].x = feedback->pose.position.x;
    checkPoints_[index].y = feedback->pose.position.y;

    refreshPath();
    flushSpline();
  } else if(obstacles_.find(index) != obstacles_.end()) {

  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "graph_planner");
  GraphPlanner planner;
  ros::spin();
  return 0;

}