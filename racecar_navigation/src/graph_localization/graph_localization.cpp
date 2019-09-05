//
// Created by yenkn on 19-7-1.
//
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <racecar_core/point.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <racecar_core/utils/tf.h>
#include <racecar_core/utils/color.h>
#include <racecar_core/utils/cv.h>
#include <racecar_core/point.h>
#include <racecar_core/utils/kd_tree.h>
#include <racecar_core/utils/graph.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <visualization_msgs/MarkerArray.h>
#include <racecar_msgs/Stage.h>
#include <racecar_msgs/Checkline.h>
#include <racecar_msgs/GetCheckline.h>
#include <racecar_msgs/QueryStage.h>
#include "laser_line/laser_feature_ros.h"
#include "line_merger.hpp"
#include "map_graph.hpp"

class GraphLocalizationNode {
public:
  GraphLocalizationNode();

  ~GraphLocalizationNode();

  struct beam_t {
      std::vector<Point2D> points;
      std::vector<cv::Vec4i> lines;
      double orientation = 0.0;
  };

  struct line_t {
      cv::Vec4i points;
      double orientation = 0.0;
  };

private:
  ros::NodeHandle nh_;
  ros::Subscriber mapSub_, scanSub_, odomSub_, imuSub_, amclSub_, goalSub_, pointSub_;
  ros::Publisher posePub_, scanPub_, markerPub_, stagePub_, mapPub_;
  ros::ServiceServer checklineSrv_, stageSrv_;
  tf::TransformListener tfListener_;
  tf::TransformBroadcaster tfBroadcaster_;
  nav_msgs::OccupancyGrid map_;
  nav_msgs::Odometry odom_;
  sensor_msgs::LaserScan scan_;
  laser_line::LaserFeatureROS laserFeature_;

  tf::Transform laserToBase_;
  tf::Pose estimatePose_;
  double yaw_ = 0.0, initialYaw_ = 0.0;

  std::mutex mapLock_;
  bool mapRecived_ = false;

  cv::Mat mapImage_;
  MapGraph mapGraph_;

  std::vector<MapGraph::checkline_t> checklines_;
  std::vector<utils::polygon_t> checkPolygons_;
  int uTurnIndex_;
  int currentStage_ = 0;

  bool goalRecived_ = false;
  Point2D goal_;

  cv::Ptr<cv::LineSegmentDetector> scanDetector_, mapDetector_;

  int erodeSize_ = 1; // erode on thick borders // like stageros
  int filterSize_ = 10; // filter contour area
  double orientationDiff_ = 10.0; // line merger param
  double adjacentDistance_ = 0.3;

  std::set<int> correctStages_;
  double featureDiffTolerance_;
  double maxFeatureDistance_;
  double correctThreshold_;
  double backwardBound_;

  double forwardDistance_, backwardDistance_;

  bool publishPlanMap_, firstPoint_, upsideDown_;
  double uTurnRadius_;
  int checklineXTolerance_;

  void mapCallback(const nav_msgs::OccupancyGridConstPtr &mapMsg);

  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scanMsg);
  void imuCallback(const sensor_msgs::ImuConstPtr &imuMsg);

  void pointCallback(const geometry_msgs::PointStampedConstPtr &msg);

  void amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

  void updateStage();
  void publishMarkers();

  bool getChecklineCallback(racecar_msgs::GetChecklineRequest &req, racecar_msgs::GetChecklineResponse &res);
  bool queryStageCallback(racecar_msgs::QueryStageRequest &req, racecar_msgs::QueryStageResponse &res);
};

GraphLocalizationNode::GraphLocalizationNode() : nh_("~"), laserFeature_("laser_line") {
  mapSub_ = nh_.subscribe("/map", 1, &GraphLocalizationNode::mapCallback, this);
  scanSub_ = nh_.subscribe("/scan", 1, &GraphLocalizationNode::scanCallback, this);
  odomSub_ = nh_.subscribe("/odometry/filtered", 1, &GraphLocalizationNode::odomCallback, this);
  imuSub_ = nh_.subscribe("/imu_data", 1, &GraphLocalizationNode::imuCallback, this);
  amclSub_ = nh_.subscribe("/amcl_pose", 1, &GraphLocalizationNode::amclCallback, this);
  goalSub_ = nh_.subscribe("/move_base_simple/goal", 1, &GraphLocalizationNode::goalCallback, this);

  pointSub_ = nh_.subscribe("/clicked_point", 1, &GraphLocalizationNode::pointCallback, this);

  posePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/estimate_pose", 1);
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("marker", 1, true);
  stagePub_ = nh_.advertise<racecar_msgs::Stage>("/stage", 1);
  mapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/plan_map", 1, true);
  checklineSrv_ = nh_.advertiseService("/checkline", &GraphLocalizationNode::getChecklineCallback, this);
  stageSrv_ = nh_.advertiseService("/query_stage", &GraphLocalizationNode::queryStageCallback, this);

  double lsdScale;
  nh_.param("erode_size", erodeSize_, 0);
  nh_.param("lsd_scale", lsdScale, 0.8);
  nh_.param("filter_size", filterSize_, 10);
  nh_.param("orientation_diff", orientationDiff_, 10.0);
  nh_.param("adjacent_distance", adjacentDistance_, 0.4);

  std::vector<int> stages;
  nh_.getParam("correct_stage", stages);
  nh_.param("feature_diff_tolerance", featureDiffTolerance_, 10.0);
  nh_.param("max_feature_distance", maxFeatureDistance_, 2.0);
  nh_.param("correct_threshold", correctThreshold_, 0.3);
  nh_.param("backward_bound", backwardBound_, 0.6);

  nh_.param("publish_plan_map", publishPlanMap_, true);
  nh_.param("uturn_radius", uTurnRadius_, 0.5);
  nh_.param("checkline_x_tolerance", checklineXTolerance_, 15);

  nh_.param("first_point", firstPoint_, true);
  nh_.param("upside_down", upsideDown_, false);

  std::vector<double> goal;
  nh_.param("default_goal", goal, {});
  goal_ = goal.size() < 2 ? Point2D(0.0, (upsideDown_ ? 1: -1) * 2.0) : Point2D(goal[0], goal[1]);

  estimatePose_ = utils::createTfFromXYTheta(0, 0, 0);

  scanDetector_ = cv::createLineSegmentDetector();
  mapDetector_ = cv::createLineSegmentDetector(cv::LineSegmentDetectorModes::LSD_REFINE_STD, lsdScale);

  std::cout << "correct stage: ";
  for(auto &s : stages) {
    std::cout << s << ", ";
    correctStages_.insert(s);
  }
}


GraphLocalizationNode::~GraphLocalizationNode() {
}


std::set<int> drawed;

void drawVertex(cv::Mat &colorImg, const MapGraph &graph, int vertex, const cv::Scalar &color) {
  if (drawed.find(vertex) != drawed.end()) return;

  auto index = graph.vertices_[vertex];
  auto line = graph.lines_[graph.points_[index].lineIndex];
  cv::circle(colorImg, cv::Point(graph.points_[index].x, graph.points_[index].y), 3, color, -1);
  cv::line(colorImg, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, 1, 8);
  drawed.insert(vertex);

  for (int v : graph.adjacency_[vertex]) {
    drawVertex(colorImg, graph, v, color);
  }
}

void drawGraph(cv::Mat &colorImg, const MapGraph &graph) {
  if (graph.features_.size() == 0) {
    std::cout << "no feature" << std::endl;
    return;
  }
  int colorIncrement = 360 / graph.features_.size();
  for (size_t i = 0; i < graph.features_.size(); i++) {
    int index = graph.vertices_[graph.features_[i].vertex];
    auto color = utils::Color::fromHSV(i * colorIncrement, 1.0, 1.0);
    auto cvColor = cv::Scalar(color.b_ * 255, color.g_ * 255, color.r_ * 255);
    auto pt = graph.points_[index];

    for (int j : graph.adjacency_[graph.features_[i].vertex]) {
      auto dist = (graph.points_[graph.vertices_[j]] + pt) / 2;
      cv::arrowedLine(colorImg, cv::Point(pt.x, pt.y), cv::Point(dist.x, dist.y), cvColor);
    }

    cv::circle(colorImg, cv::Point(graph.points_[index].x, graph.points_[index].y), 3, cvColor);
    std::cout << "feature detected: " << graph.features_[i].angle / M_PI * 180 << std::endl;
  }
}

void GraphLocalizationNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
  goal_ = Point2D(goal->pose.position.x, goal->pose.position.y);

  if (!mapRecived_ || goalRecived_) return;
  std::vector<utils::checkline_t> lines;

  for(auto &line : checklines_) {
    lines.push_back({ line.start, line.end });
  }

  Point2D mapGoal = (goal_ - Point2D(map_.info.origin.position.x, map_.info.origin.position.y)) / map_.info.resolution;
  checkPolygons_ = utils::checklinesToStages(lines, uTurnIndex_, mapGraph_.initial_, mapGoal);
  goalRecived_ = true;
  publishMarkers();
}

void GraphLocalizationNode::imuCallback(const sensor_msgs::ImuConstPtr &imuMsg) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(imuMsg->orientation, q);
  yaw_ = tf::getYaw(q);
}

void GraphLocalizationNode::amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
}

void GraphLocalizationNode::updateStage() {
  if (!goalRecived_) return;

  Point2D estimate((estimatePose_.getOrigin().x() - map_.info.origin.position.x) / map_.info.resolution,
                   (estimatePose_.getOrigin().y() - map_.info.origin.position.y) / map_.info.resolution);

  int stage = utils::pointStageTest(checkPolygons_, estimate, currentStage_);

//  cv::Mat colorImg;
//  cv::cvtColor(mapImage_, colorImg, cv::COLOR_GRAY2BGR);
//  cv::drawContours(colorImg, checkPolygons_, -1, cv::Scalar(0, 255, 0), 1);
//  cv::circle(colorImg, cv::Point(estimate.x, estimate.y), 3, cv::Scalar(0, 0, 255), -1);
//
//  cv::namedWindow("Match");
//  cv::imshow("Match", colorImg);
//  cv::waitKey(10);

  bool validStage = stage >= currentStage_ && stage - currentStage_ < 3;
  if (stage != currentStage_ && validStage) {
    currentStage_ = stage;
    std::cout << "new stage: " << currentStage_ << std::endl;
  }

  if(validStage) {
    racecar_msgs::Stage stageMsg;
    stageMsg.stage = currentStage_;

    utils::distance_t distances = utils::pointStageDistance(checkPolygons_[currentStage_], estimate);
    forwardDistance_ = distances.forward;
    backwardDistance_ = distances.backward;
    stageMsg.forward_distance = forwardDistance_ * map_.info.resolution;
    stageMsg.backward_distance = backwardDistance_ * map_.info.resolution;
    stagePub_.publish(stageMsg);
  }
}

void drawCircle(nav_msgs::OccupancyGrid &map, Point2D center, double radius, unsigned char color) {
  int r = int(radius / map.info.resolution);
  int r2 = r * r;
  int area = r2 << 2;
  int rr = r << 1;

  for (int i = 0; i < area; i++)
  {
    int tx = (i % rr) - r;
    int ty = (i / rr) - r;

    if (tx * tx + ty * ty <= r2) {
      int index = (center.x + tx) + (center.y + ty) * map.info.width;
      if(map.data[index] < color) map.data[index] = color;
    }
  }
}

void GraphLocalizationNode::mapCallback(const nav_msgs::OccupancyGridConstPtr &mapMsg) {
  if (mapRecived_) return;

  mapLock_.lock();
  map_ = *mapMsg;
  initialYaw_ = yaw_;

  cv::Mat mapImg = utils::mapToImage(map_);
  auto size = abs(erodeSize_);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                              cv::Size(2 * size + 1, 2 * size + 1),
                                              cv::Point(size, size));
  if(erodeSize_ > 0) cv::dilate(mapImg, mapImage_, element);
  else cv::erode(mapImg, mapImage_, element);

  std::vector<cv::Vec4i> keylines;
  mapDetector_->detect(mapImage_, keylines);

  std::cout << "before: " << keylines.size() << std::endl;
  auto mergedBeams = LineMerger::merge(keylines, orientationDiff_);
  std::cout << "beams: " << mergedBeams.size() << std::endl;

  std::vector<cv::Vec4i> afterLines;
  for (auto &beam : mergedBeams) {
    afterLines.insert(afterLines.end(), beam.lines.begin(), beam.lines.end());
  }

  std::cout << "after: " << afterLines.size() << std::endl;


  mapGraph_.initial_ = Point2D(-map_.info.origin.position.x / map_.info.resolution, -map_.info.origin.position.y / map_.info.resolution);
  mapGraph_.build(afterLines);
  checklines_ = mapGraph_.getChecklines(uTurnIndex_, upsideDown_ ? -1 : 1, checklineXTolerance_);

  std::vector<utils::checkline_t> lines;
  for(auto &line : checklines_) {
    lines.push_back({ line.start, line.end });
  }
  Point2D mapGoal = (goal_ - Point2D(map_.info.origin.position.x, map_.info.origin.position.y)) / map_.info.resolution;
  checkPolygons_ = utils::checklinesToStages(lines, uTurnIndex_, mapGraph_.initial_, mapGoal);

  publishMarkers();

  if(checklines_.empty()) {
    ROS_ERROR("unable to crate checklines.");
  }

  if(!checklines_.empty() && publishPlanMap_) {
    nav_msgs::OccupancyGrid planMap;
    std::copy(map_.data.begin(), map_.data.end(), std::back_inserter(planMap.data));
    planMap.header.stamp = ros::Time::now();
    planMap.header.frame_id = "map";
    planMap.info = map_.info;
    auto turnPoint = checklines_[uTurnIndex_-1].end;
    int radius = uTurnRadius_ / map_.info.resolution;
    drawCircle(planMap, Point2D(turnPoint.x, turnPoint.y), uTurnRadius_, 100);

    if(firstPoint_) {
      auto firstPoint = Point2D(checklines_[0].start.x - 60, checklines_[0].start.y - 22);
      drawCircle(planMap, firstPoint, 0.2, 100);
    }

    mapPub_.publish(planMap);
  }

  mapLock_.unlock();
  mapRecived_ = true;

// cv::Mat colorImg;
// cv::cvtColor(mapImage_, colorImg, cv::COLOR_GRAY2BGR);
// drawGraph(colorImg, mapGraph_);
//  for(int i = 0; i < checklines_.size(); i++) {
//    cv::arrowedLine(colorImg, cv::Point(checklines_[i].start.x, checklines_[i].start.y),
//                    cv::Point(checklines_[i].end.x, checklines_[i].end.y),
//                    i == uTurnIndex_ ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
//  }
//
//  cv::namedWindow("Map");
//  cv::imshow("Map", colorImg);
//  cv::waitKey(0);

}

void GraphLocalizationNode::pointCallback(const geometry_msgs::PointStampedConstPtr &msg) {
  Point2D point((msg->point.x - map_.info.origin.position.x) / map_.info.resolution,
                   (msg->point.y - map_.info.origin.position.y) / map_.info.resolution);

  auto stage = utils::pointStageTest(checkPolygons_, point);
  std::cout << stage << "; ";
  auto distances = stage >= 0 ? utils::pointStageDistance(checkPolygons_[stage], point) : utils::distance_t();
  std::cout << distances.forward * map_.info.resolution << ", " << distances.backward * map_.info.resolution << ", "
            << distances.leftward * map_.info.resolution << ", " << distances.rightward * map_.info.resolution << std::endl;
}

bool GraphLocalizationNode::queryStageCallback(racecar_msgs::QueryStageRequest &req,
                                               racecar_msgs::QueryStageResponse &res) {
  if(checkPolygons_.empty()) return false;

  Point2D point((req.point.x - map_.info.origin.position.x) / map_.info.resolution,
                (req.point.y - map_.info.origin.position.y) / map_.info.resolution);

  res.currentStage = currentStage_;
  res.stage = req.knownStage < 0 ? utils::pointStageTest(checkPolygons_, point) : req.knownStage;

  auto distances = res.stage >= 0 ? utils::pointStageDistance(checkPolygons_[res.stage], point) : utils::distance_t();
  res.forward = distances.forward * map_.info.resolution;
  res.backward = distances.backward * map_.info.resolution;
  res.leftward = distances.leftward * map_.info.resolution;
  res.rightward = distances.rightward * map_.info.resolution;
  return true;
}

bool GraphLocalizationNode::getChecklineCallback(racecar_msgs::GetChecklineRequest &req,
                                                 racecar_msgs::GetChecklineResponse &res) {
  ROS_INFO("sending checklines");
  racecar_msgs::Checkline msg;
  for(auto &line: checklines_) {
    msg.points.push_back(((line.start - mapGraph_.initial_) * map_.info.resolution).toPoint());
    msg.points.push_back(((line.end - mapGraph_.initial_) * map_.info.resolution).toPoint());
  }
  msg.uturn_index = uTurnIndex_;
  res.checkline = msg;
  return true;
}

void GraphLocalizationNode::publishMarkers() {
  visualization_msgs::Marker lineList, pointList;
  pointList.header.frame_id = lineList.header.frame_id = "map";
  pointList.header.stamp = lineList.header.stamp = ros::Time::now();
  pointList.ns = lineList.ns = "graph";
  pointList.action = lineList.action = visualization_msgs::Marker::ADD;
  pointList.pose.orientation.w = lineList.pose.orientation.w = 1.0;
  lineList.id = 0;
  pointList.id = 1;

  pointList.type = visualization_msgs::Marker::POINTS;
  pointList.scale.x = 0.1;
  pointList.scale.y = 0.1;

  lineList.type = visualization_msgs::Marker::LINE_LIST;
  lineList.scale.x = 0.05;

  if (mapGraph_.features_.empty()) {
    std::cout << "no feature" << std::endl;
    return;
  }

  int colorIncrement = 360 / mapGraph_.features_.size();
  for (size_t i = 0; i < mapGraph_.features_.size(); i++) {
    int index = mapGraph_.vertices_[mapGraph_.features_[i].vertex];
    auto color = utils::Color::fromHSV(i * colorIncrement, 1.0, 1.0);
    auto pt = mapGraph_.points_[index];

    std::cout << "feature detected: " << mapGraph_.features_[i].angle / M_PI * 180 << std::endl;

    for (int j : mapGraph_.adjacency_[mapGraph_.features_[i].vertex]) {
      geometry_msgs::Point pt1, pt2;
      auto dist = (mapGraph_.points_[mapGraph_.vertices_[j]] + pt) / 2;

      pt1.x = map_.info.origin.position.x + (double)pt.x * map_.info.resolution;
      pt1.y = map_.info.origin.position.y + (double)pt.y * map_.info.resolution;
      pt2.x = map_.info.origin.position.x + (double)dist.x * map_.info.resolution;
      pt2.y = map_.info.origin.position.y + (double)dist.y * map_.info.resolution;
      pt1.z = pt2.z = 0.0;

      lineList.points.push_back(pt1);
      lineList.points.push_back(pt2);
      lineList.colors.push_back(color.toColorRGBA());
      lineList.colors.push_back(color.toColorRGBA());

      pointList.points.push_back(pt1);
      pointList.colors.push_back(utils::Color::fromRGB(1, 0, 0).toColorRGBA());

    }
  }

  double vIncrement = 0.6 / checklines_.size();
  for(int i = 0; i < checklines_.size(); i++) {
    geometry_msgs::Point pt1, pt2;
    pt1.x = map_.info.origin.position.x + (double)checklines_[i].start.x * map_.info.resolution;
    pt1.y = map_.info.origin.position.y + (double)checklines_[i].start.y * map_.info.resolution;
    pt2.x = map_.info.origin.position.x + (double)checklines_[i].end.x * map_.info.resolution;
    pt2.y = map_.info.origin.position.y + (double)checklines_[i].end.y * map_.info.resolution;
    pt1.z = pt2.z = 0.0;

    lineList.points.push_back(pt1);
    lineList.points.push_back(pt2);

    auto color = i == uTurnIndex_ ? utils::Color::fromHSV(20, 1, 1) : utils::Color::fromHSV(130, 1.0, 1 - (vIncrement * i));
    lineList.colors.push_back(color.toColorRGBA());
    lineList.colors.push_back(color.toColorRGBA());
  }

  markerPub_.publish(lineList);
  markerPub_.publish(pointList);
}

void GraphLocalizationNode::odomCallback(const nav_msgs::OdometryConstPtr &odomMsg) {
  if (odom_.header.seq == 0) {
    odom_ = *odomMsg;
    return;
  }
  odom_ = *odomMsg;


  geometry_msgs::PoseStamped robot_pose, global_pose;
  robot_pose.header.frame_id = "base_footprint";
  robot_pose.header.stamp = ros::Time();
  tf::poseTFToMsg(tf::Transform::getIdentity(), robot_pose.pose);

  try {
    tfListener_.transformPose("map", ros::Time(0), robot_pose, "map", global_pose);
  } catch (tf::TransformException &ex) {
    return;
  }
  tf::poseMsgToTF(global_pose.pose, estimatePose_);
  updateStage();
}


void GraphLocalizationNode::scanCallback(const sensor_msgs::LaserScanConstPtr &scanMsg) {
  if(correctStages_.empty()) return;
  if(!mapRecived_) return;
  if(scan_.header.seq == 0) {
    tf::StampedTransform baseToLaser;
    if(!utils::getTFTransform(tfListener_, "base_footprint", scanMsg->header.frame_id, baseToLaser)) {
      ROS_ERROR("Unable to get base to laser transform");
      return;
    }
    laserToBase_ = baseToLaser.inverse();
    scan_ = *scanMsg;
    return;
  }

  if(!goalRecived_) return;
  if(correctStages_.find(currentStage_) == correctStages_.end()) return;

  auto startTime = ros::Time::now();
  int scanWidth = scan_.range_max * 2 / map_.info.resolution, scanHeight = scan_.range_max * 2 / map_.info.resolution;
  Point2D center(scanWidth / 2, scanHeight / 2);
  double scanResolution = map_.info.resolution;
  cv::Mat scanImage(scanHeight, scanWidth, CV_8UC1, cv::Scalar(0));

  double yaw = tf::getYaw(estimatePose_.getRotation());

  scan_ = *scanMsg;
  for(int i = 0; i < scan_.ranges.size(); i++) {
    double range = scan_.ranges[i];
    if (range < scan_.range_min || range >= scan_.range_max) {
      continue;
    }
    auto angle = scan_.angle_min + i * scan_.angle_increment;
    auto point = laserToBase_ * utils::polarToCartesian(range, angle);
    auto pixel = Point2D(point.x(), point.y()).rotate(yaw) / scanResolution + center;
    if(pixel.x < 0 || pixel.x >= scanWidth || pixel.y < 0 || pixel.y >= scanHeight) {
      continue;
    }
    scanImage.at<uchar>((int)pixel.y, (int)pixel.x) = 225;
  }

  cv::Mat dst = scanImage;
//  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
//                                              cv::Size(2 * 1 + 1, 2 * 1 + 1),
//                                              cv::Point(1, 1));
//  cv::dilate(scanImage, dst, element);

  std::vector<cv::Vec4i> keylines;
  // scanDetector_->detect(dst, keylines);
  // cv::HoughLinesP(scanImage, keylines, 1, CV_PI / 180.0, 4, 4, 5);
  // auto lines = LineMerger::mergeBeams(LineMerger::merge(keylines, orientationDiff_));

  cv::Mat colorScan;
  cv::cvtColor(dst, colorScan, cv::COLOR_GRAY2BGR);


  auto laserLines = laserFeature_.processScan(scanMsg);
  for(laser_line::gline &line : laserLines) {
    Point2D pt1(line.x1, line.y1), pt2(line.x2, line.y2);
    pt1 = pt1.rotate(yaw) / scanResolution + center;
    pt2 = pt2.rotate(yaw) / scanResolution + center;
    keylines.emplace_back(pt1.x, pt1.y, pt2.x, pt2.y);
    cv::line(colorScan, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), cv::Scalar(0, 255, 0), 1);
  }

  if(!keylines.empty()) {

    // std::cout << "after: " << keylines.size() << std::endl;

    MapGraph graph(keylines);
    graph.searchRadius_ = 5.0;
    graph.initial_ = Point2D(dst.cols/2, dst.rows/2);

    auto nextCheckline = checklines_[currentStage_];
//    if(currentStage_ == uTurnIndex_ - 1) {
//      nextCheckline = checklines_[currentStage_];
//    } else if(currentStage_ == uTurnIndex_ + 1) {
//      nextCheckline = checklines_[currentStage_+2];
//    }

    Point2D checklinePoint;
    double relativeX, relativeY;
    double matchedAngleDiff = featureDiffTolerance_/180*M_PI;
    bool featureMatched = false;
    int matchCount = 0;

//    drawGraph(colorScan, graph);
//
//    cv::flip(colorScan, colorScan, 0);
//
//    std::cout << "time: " << (ros::Time::now() - startTime).toSec() << std::endl;
//
//    cv::namedWindow("Scan", cv::WINDOW_NORMAL);
//    cv::resizeWindow("Scan", 600,600);
//    cv::imshow("Scan", colorScan);
//    cv::waitKey(60);

    if(graph.features_.size() > 1) {
      int upperFeature = 0;
      for (auto &f : graph.features_) {
        auto position = f.getPoint(graph) - graph.initial_;
        if(position.y > 0) upperFeature++;
      }
      if(graph.features_.size() == upperFeature || upperFeature == 0) return; // feature same side
    }


    for(auto &f : graph.features_) {
      auto position = f.getPoint(graph) - graph.initial_;
      if((currentStage_ > uTurnIndex_ && position.x > 5) || (currentStage_ <= uTurnIndex_ && position.x < 5)) continue; // filter back points
      bool side = ((position.y > 0 && currentStage_ <= uTurnIndex_) || (position.y < 0 && currentStage_ > uTurnIndex_));
      double angle = angleDiff(f.angle, (side ? nextCheckline.startAngle : nextCheckline.endAngle));
      if(angle < matchedAngleDiff && position.length() < maxFeatureDistance_ / scanResolution) {
        checklinePoint = side ? nextCheckline.start : nextCheckline.end;
        relativeX = position.x * scanResolution;
        relativeY = position.y * scanResolution;
        featureMatched = true;
        matchCount++;
      }
    }

    if(featureMatched) {
      auto checkPoint = (checklinePoint - mapGraph_.initial_) * map_.info.resolution;
      Point2D mapPos = checkPoint - Point2D(relativeX, relativeY);
//      double featureToCarAngle = atan2(checkPoint.y - mapPos.y, checkPoint.x - mapPos.x);
//      double featureToScanAngle = atan2(relativeY, relativeX);

      auto pose = utils::createTfFromXYTheta(mapPos.x, mapPos.y, tf::getYaw(estimatePose_.getRotation()));

      // && (backwardDistance_ * scanResolution > backwardBound_ || backwardDistance_ > forwardDistance_)
      if(pose.getOrigin().distance(estimatePose_.getOrigin()) > correctThreshold_) {
        geometry_msgs::PoseWithCovarianceStamped estimate;
        estimate.header.stamp = ros::Time::now();
        estimate.header.frame_id = "map";
        estimate.pose.covariance = {
            0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942 };
        tf::poseTFToMsg(pose, estimate.pose.pose);

        posePub_.publish(estimate);
        std::cout << "pose corrected, distance: " << hypot(relativeX, relativeY) << "; " << " diff: " << pose.getOrigin().distance(estimatePose_.getOrigin()) << std::endl;
      }

      visualization_msgs::Marker checkPointMsg, scanFeatureMsg;
      checkPointMsg.header.frame_id = "map";
      scanFeatureMsg.header.frame_id = scan_.header.frame_id;
      scanFeatureMsg.header.stamp = checkPointMsg.header.stamp = ros::Time::now();
      scanFeatureMsg.pose.orientation.w = checkPointMsg.pose.orientation.w = 1;
      scanFeatureMsg.scale.x = scanFeatureMsg.scale.y = scanFeatureMsg.scale.z = checkPointMsg.scale.x = checkPointMsg.scale.y = checkPointMsg.scale.z = 0.2;
      scanFeatureMsg.type = checkPointMsg.type = visualization_msgs::Marker::SPHERE;
      scanFeatureMsg.action = checkPointMsg.action = visualization_msgs::Marker::ADD;
      scanFeatureMsg.ns = checkPointMsg.ns = "graph";
      scanFeatureMsg.lifetime = ros::Duration(3);

      checkPointMsg.color = utils::Color::fromRGB(255, 0, 0).toColorRGBA();
      checkPointMsg.id = 3;
      checkPointMsg.pose.position = checkPoint.toPoint();

      scanFeatureMsg.color = utils::Color::fromRGB(0, 255, 0).toColorRGBA();
      scanFeatureMsg.id = 4;
      scanFeatureMsg.pose.position.x = relativeX;
      scanFeatureMsg.pose.position.y = relativeY;

      markerPub_.publish(checkPointMsg);
      markerPub_.publish(scanFeatureMsg);
    }


//    Point2D estimate(mapGraph_.initial_.x + estimatePose_.getOrigin().x() / map_.info.resolution,
//                     mapGraph_.initial_.y + estimatePose_.getOrigin().y() / map_.info.resolution);
//    auto matchTf = mapGraph_.match(mapImage_, graph, estimate);
//    if(matchTf.distance < 3.0) {
//      geometry_msgs::PoseStamped msg;
//      msg.header.stamp = ros::Time::now();
//      msg.header.frame_id = "map";
//      auto pose = utils::createTfFromXYTheta((matchTf.x - mapImage_.cols / 2) * map_.info.resolution,
//          (matchTf.y - mapImage_.rows / 2) * map_.info.resolution,
//          0);
//      tf::poseTFToMsg(pose, msg.pose);
//
//      std::cout << "trans: " << pose.getOrigin().x()
//                << ", " << pose.getOrigin().y()
//                << ", " << (matchTf.orientaion * 180 / M_PI) << std::endl;
//     posePub_.publish(msg);
//    }


  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "graph_localization");
  GraphLocalizationNode node;
  ros::spin();
  return 0;
}