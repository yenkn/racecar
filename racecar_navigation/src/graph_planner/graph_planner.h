//
// Created by yenkn on 19-7-14.
//

#ifndef SRC_GRAPH_PLANNER_H
#define SRC_GRAPH_PLANNER_H

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <racecar_msgs/Checkline.h>
#include <racecar_msgs/ObstacleEvent.h>
#include <racecar_core/point.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tinysplinecpp.h>
#include <racecar_core/utils/graph.hpp>

struct obstacle_t {
  int side = 0; // -1 leftward -2 rightward
  racecar_msgs::Obstacle obstacle;
  Point2D control;

  obstacle_t() = default;
  explicit obstacle_t(racecar_msgs::Obstacle ob): obstacle(std::move(ob)) {}
};

class GraphPlanner {
public:
  GraphPlanner();
  ~GraphPlanner();

private:
  ros::NodeHandle nh_;
  ros::Publisher pathPub_;
  ros::Subscriber amclSub_, goalSub_, obstacleSub_, pointSub_;
  ros::ServiceClient checklineClient_;
  interactive_markers::InteractiveMarkerServer *server_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback markerCb_;

  Point2D goal_;
  int uturnIndex_ = 0;
  bool pathFound_;

  std::vector<Point2D> checkPoints_;

  std::map<int, obstacle_t> obstacles_;

  std::vector<std::array<Point2D, 2>> checkline_;
  std::vector<utils::polygon_t> stages_;
  tinyspline::BSpline spline_;

  std::string splineFile_;
  double uTurnMargin_, pathSpacing_, mergeRadius_, carWidth_, lockSideDistance_;
  int sampleTimes_, changeSideSurvive_;
  bool upsideDown_;
  std::set<int> obstacleStages_;

  std::map<int, std::vector<int>> obstacleSides_;

  void initializeSpline();
  void initializeMarkers();
  void flushSpline();
  void createInteractiveMarker(int id, const Point2D &point);

  void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void obstacleCallback(const racecar_msgs::ObstacleEventConstPtr &msg);
  void pointCallback(const geometry_msgs::PointStampedConstPtr &msg);
  bool getCheckline();

  int selectSide(int obstacleId);
  std::vector<Point2D> getPoints();
  bool makePath(std::vector<Point2D> &path);
  void refreshPath();
};

#endif //SRC_GRAPH_PLANNER_H
