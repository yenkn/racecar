//
// Created by yenkn on 19-7-10.
//

#ifndef SRC_MAP_GRAPH_H
#define SRC_MAP_GRAPH_H

#include <racecar_core/point.h>
#include <racecar_core/utils/kd_tree.h>
#include <racecar_core/utils/math.h>
#include <stack>

class LinePoint : public Point2D {
public:
  int lineIndex = 0;
  int pairIndex = 0;
  int graph = -1;
  int vertex = -1;
  bool processed = false;

  LinePoint(double x_, double y_, int lineIndex_, int pairIndex_): Point2D(x_, y_), lineIndex(lineIndex_), pairIndex(pairIndex_) {}
};

double angleDiff(double a, double b) {
  double diff = fabs(a - b);
  if(diff > M_PI) diff = 2 * M_PI - diff;
  return diff;
}

class MapGraph {
public:
  struct feature_t {
      int vertex;
      double angle;

      feature_t() = default;
      feature_t(int v, double a): vertex(v), angle(a) {}

      template<int N = -1>
      inline Point2D getPoint(const MapGraph &graph) const {
        if(N < 0) return graph.points_[graph.vertices_[vertex]];
        return graph.points_[graph.vertices_[graph.adjacency_[vertex][N]]];
      }
  };

  struct match_t {
      int ref;
      int align;
      double angle;

      match_t(int referenceFeature, int alignFeature, double angleDiff): ref(referenceFeature), align(alignFeature), angle(angleDiff) {}
  };

  struct transform_t {
      int x;
      int y;
      int testX;
      int testY;
      double orientaion;
      double distance = FLT_MAX;

      transform_t() = default;
      transform_t(int x_, int y_, double ori_): x(x_), y(y_), orientaion(ori_) {}
  };

  typedef std::pair<Point2D, Point2D> edge_t;

  struct checkline_t {
      Point2D start;
      double startAngle;
      Point2D end;
      double endAngle;

      checkline_t() = default;
      checkline_t(const Point2D &s, double startAng, const Point2D &e, double endAng): start(s), end(e), startAngle(startAng), endAngle(endAng) {}
  };

  MapGraph() = default;
  explicit MapGraph(const std::vector<cv::Vec4i> &lines) {
    build(lines);
  }

  void build(const std::vector<cv::Vec4i> &lines) {
    lines_ = lines;

    for(auto &line : lines_) {
      if(line[0] > line[2]) {
        std::swap(line[0], line[2]);
        std::swap(line[1], line[3]);
      }
    }

    std::sort(lines_.begin(), lines_.end(), [](cv::Vec4i &a, cv::Vec4i &b) {
      return a[0] < b[0];
    });

    for(int i = 0; i < lines_.size(); i++) {
      points_.emplace_back(lines_[i][0], lines_[i][1], i, i*2+1);
      points_.emplace_back(lines_[i][2], lines_[i][3], i, i*2);
    }

    tree_.build(points_);

    for(int i = 0; i < lines_.size(); i++) {
      int vertex = addPoint(-1, i*2);
      if(vertex >= 0) subGraph_[vertex] = 0;
    }

    for(const auto &pt : points_) {
      if(pt.graph >= 0) subGraph_[pt.graph]++; // count sub graph point count
    }

    // extract feature points
    for(int id = 0; id < vertices_.size(); id++) {
      if(adjacency_[id].size() == 2) {
        auto pt = points_[vertices_[id]];
        auto pt1 = points_[vertices_[adjacency_[id][0]]];
        auto pt2 = points_[vertices_[adjacency_[id][1]]];
        if(pt2.x < pt1.x) {
          // ensure pt1.x < pt2.x
          std::swap(adjacency_[id][0], adjacency_[id][1]);
          std::swap(pt1, pt2);
        }
        auto angle = atan2(pt2.y - pt.y, pt2.x - pt.x) - atan2(pt1.y - pt.y, pt1.x - pt.x);
        if(angle < -M_PI) angle += 2 * M_PI;
        else if(angle > M_PI) angle -= 2 * M_PI;
        features_.emplace_back(id, angle);
      }

      // extract edges
      std::vector<bool> processed(vertices_.size(), false);
      for(int adjId : adjacency_[id]) {
        if(processed[adjId]) continue;
        edges_.emplace_back(points_[vertices_[id]], points_[vertices_[adjId]]);
        processed[adjId] = true;
      }
      processed[id] = true;
    }

    std::sort(features_.begin(), features_.end(), [](feature_t &a, feature_t &b) {
      return a.angle < b.angle;
    });
  }

  std::vector<checkline_t> getChecklines(int &uTurnIndex, double clusterTolerance = 10) {
    // find sub graph with maximum points
    using pair_t = decltype(subGraph_)::value_type;
    auto pair = std::max_element(subGraph_.begin(), subGraph_.end(), [](const pair_t &a, const pair_t &b) { return a.second < b.second; });
    decltype(points_) graphPoints;
    std::copy_if(points_.begin(), points_.end(), std::back_inserter(graphPoints), [pair](const LinePoint &pt) {
      return pt.graph == pair->first;
    });

    // sort points by x
    std::sort(graphPoints.begin(), graphPoints.end(), [](const LinePoint &a, const LinePoint &b) {
      return a.x < b.x;
    });

    //correct points because of map rotation
    auto lastPoint = graphPoints[graphPoints.size()-1];
    auto prevLastPoint = graphPoints[graphPoints.size()-2];
    auto correctedPoints = graphPoints;
    auto theta = atan2(lastPoint.x - prevLastPoint.x, lastPoint.y - prevLastPoint.y);
    if(fabs(theta) < 10/180*M_PI) {
      for (auto &pt : correctedPoints) {
        auto relate = (pt - lastPoint).rotate(theta);
        pt.x = lastPoint.x + relate.x;
        pt.y = lastPoint.y + relate.y;
      }
    }

    // exclude points before initial pose
    int start = std::distance(correctedPoints.begin(),
        std::find_if(correctedPoints.begin(), correctedPoints.end(), [&](LinePoint &a) { return a.x > initial_.x; }));

    // cluster points by x
    std::vector<std::vector<LinePoint>> groups({ {} });
    auto yComparer = [](const LinePoint &a, const LinePoint &b) {
      return a.y > b.y; // upside down
    };
    for(int i = start, g = 0; i < graphPoints.size(); i++) {
      if((i > start && correctedPoints[i].x - correctedPoints[i-1].x > clusterTolerance)) {
        std::sort(groups[g].begin(), groups[g].end(), yComparer); // ensure top -> middle -> bottom
        groups.emplace_back();
        g++;
      }
      groups[g].push_back(graphPoints[i]);
    }
    std::sort(groups.back().begin(), groups.back().end(), yComparer);

    std::map<int, feature_t> featureMap = {};
    for(auto &f: features_) {
      featureMap[f.vertex] = f;
    }

    bool foundTurnPoint = false;
    checkline_t uTurnCheckline;
    std::vector<checkline_t> beforeCheckLines, afterCheckLines;
    for(int i = 0; i < groups.size(); i++) {
      if(groups[i].size() == 1 && adjacency_[groups[i][0].vertex].size() == 1) { // not feature point
        // u-turn point
        if(i == groups.size() - 1 || groups[i+1].size() != 2) {
          ROS_ERROR("No u-turn after u-turn point");
          return {};
        }
        beforeCheckLines.emplace_back(Point2D(groups[i][0].x, (groups[i+1][0].y + groups[i-1][0].y)/2), 0.0, groups[i][0], 0.0);
        afterCheckLines.emplace(afterCheckLines.begin(), Point2D(groups[i][0].x, (groups[i+1][1].y + groups[i-1][2].y)/2), 0.0, groups[i][0], 0.0);
        foundTurnPoint = true;
      } else if(groups[i].size() == 2 && foundTurnPoint) {
        uTurnCheckline = checkline_t(groups[i][0], featureMap[groups[i][0].vertex].angle, groups[i][1], featureMap[groups[i][1].vertex].angle);
      } else if(groups[i].size() == 3) {
        // two lanes
        beforeCheckLines.emplace_back(groups[i][0], featureMap[groups[i][0].vertex].angle, groups[i][1], featureMap[groups[i][1].vertex].angle);
        afterCheckLines.emplace(afterCheckLines.begin(), groups[i][2], featureMap[groups[i][2].vertex].angle, groups[i][1], featureMap[groups[i][1].vertex].angle);
      } else if(groups[i].size() == 4) {
        // two path
        beforeCheckLines.emplace_back(groups[i][0], featureMap[groups[i][0].vertex].angle, groups[i][1], featureMap[groups[i][1].vertex].angle);
        afterCheckLines.emplace(afterCheckLines.begin(), groups[i][3], featureMap[groups[i][3].vertex].angle, groups[i][2], featureMap[groups[i][2].vertex].angle);
      }
    }
    if(uTurnIndex == -1) {
      ROS_ERROR("No u-turn found");
      return {};
    }

    std::vector<checkline_t> result;
    result.insert(result.begin(), beforeCheckLines.begin(), beforeCheckLines.end());
    uTurnIndex = result.size();
    result.push_back(uTurnCheckline);
    result.insert(result.end(), afterCheckLines.begin(), afterCheckLines.end());
    return result;
  }

  transform_t match(const cv::Mat &mapImage, const MapGraph &subGraph, const Point2D &estimatePosition, size_t iterations = 10) {
    transform_t ret;
    if(subGraph.features_.size() < 2) return ret;

    auto allowFeatures = filterFeatures(estimatePosition);
    std::vector<std::pair<Point2D, Point2D>> allowLines;
    for(int f : allowFeatures) {
      allowLines.emplace_back(features_[f].getPoint(*this), features_[f].getPoint<0>(*this));
      allowLines.emplace_back(features_[f].getPoint(*this), features_[f].getPoint<1>(*this));
    }

    auto matches = matchFeatures(allowFeatures, subGraph);
    iterations = std::min(iterations, matches.size());
    double minDistance = FLT_MAX;
    int minMatch = -1;

    for(int i = 0; i < iterations; i++) {
      // align sub graph to local feature point
      transform_t tf;
      auto transformedPoints = transformPoints(subGraph, matches[i].ref, matches[i].align, tf);

       auto distance = getMatchDistance(subGraph, allowLines, transformedPoints);
      if(distance > 0 && distance < minDistance) {
        minDistance = distance;
        minMatch = i;
        ret = tf;
//        cv::Mat colorImg;
//        cv::cvtColor(mapImage, colorImg, cv::COLOR_GRAY2BGR);
//        for(int f : allowFeatures) {
//          cv::circle(colorImg, cv::Point(points_[vertices_[features_[f].vertex]].x, points_[vertices_[features_[f].vertex]].y), 3, cv::Scalar(255, 0, f == matches[i].ref ? 255 : 0), -1);
//        }
//        for(auto &edge : allowLines) {
//          cv::line(colorImg, cv::Point(edge.first.x, edge.first.y), cv::Point(edge.second.x, edge.second.y), cv::Scalar(255, 126, 0), 1, 8);
//        }
//        cv::circle(colorImg, cv::Point(estimatePosition.x, estimatePosition.y), 5, cv::Scalar(0, 0, 255), -1);
//        for(auto &pt : transformedPoints) {
//          cv::circle(colorImg, cv::Point(pt.x, pt.y), 2, cv::Scalar(0, 255, 0), -1);
//        }
//        cv::namedWindow( "Map", 1 );
//        cv::imshow( "Map", colorImg);
//        cv::waitKey(30);
      }
    }
    std::cout << "min distance: " << minDistance << std::endl;

    ret.distance = minDistance;
    return ret;
  }

  double getMatchDistance(const MapGraph &graph, const std::vector<std::pair<Point2D, Point2D>> &allowLines, const std::vector<LinePoint> &points) {
    double score = 0.0;
    double den = 0.0;
    std::vector<bool> processed(points.size(), false);

    for(auto &feature : graph.features_) {
      if(fabs(feature.angle) < 10/180*M_PI || fabs(feature.angle) > 170/180*M_PI) continue;

      int alignIndex = graph.vertices_[feature.vertex];
      auto alignPoint = points[alignIndex];
      processed[alignIndex] = true; // processed feature point

      double minDist;
      int localPoint = tree_.nnSearch(alignPoint, &minDist);
      if(localPoint < 0 || localPoint >= points_.size()) continue;

      auto localVertex = std::find(vertices_.begin(), vertices_.end(), localPoint);
      if(localVertex == vertices_.end()) continue;
      if(adjacency_[*localVertex].size() != 2) continue; // is feature vertex?

      score += points_[localPoint].distance(alignPoint);
      den += 0.5;

      // score feature edges
      auto leftPoint = points_[vertices_[adjacency_[*localVertex][0]]];
      auto rightPoint = points_[vertices_[adjacency_[*localVertex][1]]];

      int leftIndex = graph.vertices_[graph.adjacency_[feature.vertex][0]];
      int rightIndex = graph.vertices_[graph.adjacency_[feature.vertex][1]];
      score += utils::distaceToLineSegment(points[leftIndex], points_[localPoint], leftPoint);
      score += utils::distaceToLineSegment(points[graph.vertices_[graph.adjacency_[feature.vertex][1]]], points_[localPoint],
                                    rightPoint);
      den += 2;
      processed[leftIndex] = true;
      processed[rightIndex] = true;
    }

    for(int i = 0; i < points.size(); i++) {
      if(processed[i]) continue;
      // find nearest line
      double minDistance = FLT_MAX;
      for(auto &line : allowLines) {
        double distance = utils::distaceToLineSegment(points[i], line.first, line.second);
        if(distance < minDistance) minDistance = distance;
      }
      score += minDistance;
      den += 1 + minDistance / 5;
    }

    return score / den;
  }

  std::vector<int> filterFeatures(const Point2D &estimatePosition) {
    std::vector<int> indexes;
    for(int i = 0; i < features_.size(); i++) {
      auto pt = points_[vertices_[features_[i].vertex]];
      bool intersected = false;
      for(auto &line: edges_) {
        if(utils::isLineIntersect(pt, estimatePosition, line.first, line.second)) {
          intersected = true;
          break;
        }
      }
      if(!intersected && pt.distance(estimatePosition) < 10/0.05) indexes.push_back(i);
    }
    return indexes;
  }

  std::vector<match_t> matchFeatures(const std::vector<int> &features, const MapGraph &other) {
    std::vector<match_t> matches;

    for(int i : features) {
      for(int j = 0; j < other.features_.size(); j++) {
        matches.emplace_back(i, j, angleDiff(features_[i].angle, other.features_[j].angle));
      }
    }

    std::sort(matches.begin(), matches.end(), [](match_t &a, match_t &b) {
      return a.angle < b.angle;
    });

    return matches;
  }

  std::vector<LinePoint> transformPoints(const MapGraph &graph, int referenceFeature, int alignFeature, transform_t &transform) {
    auto srcPoint = points_[vertices_[features_[referenceFeature].vertex]];
    auto alignPoint = graph.points_[graph.vertices_[graph.features_[alignFeature].vertex]];

    Point2D translation = srcPoint - alignPoint;
    double orientation = (features_[referenceFeature].getPoint<0>(*this) - srcPoint).angle()
        - (graph.features_[alignFeature].getPoint<0>(graph) - alignPoint).angle();

    auto points = graph.points_;

    for(auto &pt : points) {
      auto relative = (pt - alignPoint).rotate(orientation);
      pt.x = srcPoint.x + relative.x;
      pt.y = srcPoint.y + relative.y;
    }

    auto pt = srcPoint + (initial_ - alignPoint).rotate(orientation);

    transform = transform_t(pt.x, pt.y, orientation);
    return points;
  }

  /***
   *
   * @param srcVertex
   * @param index
   * @param topGraph: top layer (sub graph) id
   * @return
   */
  int addPoint(int srcVertex, int index, int topGraph = -1) {
    if(points_[index].processed) return -1;

    vertices_.push_back(index);
    int vertexId = vertices_.size() - 1;
    if(topGraph == -1) topGraph = vertexId; // current vertex is top

    if(srcVertex >= 0) {
      adjacency_.push_back({ srcVertex });
      adjacency_[srcVertex].push_back(vertexId);
    } else {
      adjacency_.emplace_back();
    }

    auto &pt = points_[index];
    pt.processed = true;
    pt.graph = topGraph;
    pt.vertex = vertexId;
    addPoint(vertexId, pt.pairIndex, topGraph);

    auto indices = tree_.radiusSearch(pt, searchRadius_);
    indices.erase(std::remove_if(indices.begin(), indices.end(), [this](int x) { return points_[x].processed; }), indices.end());
    if(indices.empty()) {
      return vertexId;
    }

    // connect another line
    auto concatIndex = indices[0];
    points_[concatIndex].processed = true;
    addPoint(vertexId, points_[concatIndex].pairIndex, topGraph);

    return vertexId;
  }

  std::vector<cv::Vec4i> lines_;
  std::vector<LinePoint> points_;
  utils::KDTree<LinePoint> tree_;

  std::map<int, int> subGraph_;

  std::vector<int> vertices_;
  std::vector<std::vector<int>> adjacency_;
  std::vector<feature_t> features_;
  std::vector<edge_t> edges_;

  Point2D initial_;
  double searchRadius_ = 20.0;

};

#endif //SRC_MAP_GRAPH_H
