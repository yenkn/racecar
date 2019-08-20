//
// Created by yenkn on 19-7-9.
//

#ifndef SRC_LINE_MERGER_HPP
#define SRC_LINE_MERGER_HPP

#include <opencv2/opencv.hpp>

#include <racecar_core/point.h>
#include <racecar_core/utils/kd_tree.h>

class LineMerger {
public:
  struct beam_t {
      std::vector<Point2D> points;
      std::vector<cv::Vec4i> lines;
      double orientation = 0.0;
  };

  static std::vector<beam_t> merge(const std::vector<cv::Vec4i> &lines, double orientation_diff = 15.0) {
    std::vector<cv::Vec4i> lines_ = lines;
    std::vector<line_t> sort_lines;

    for(auto &pt : lines_) {
      if(pt[0] > pt[2]) { std::swap(pt[1], pt[3]), std::swap(pt[0], pt[2]); }
      auto orientation = atan2((pt[3] - pt[1]), (pt[2] - pt[0])) * 180 / M_PI;
      if(orientation < -15) orientation += 180;
      line_t l;
      l.points = pt;
      l.orientation = orientation;
      sort_lines.push_back(l);
    }
    std::sort(sort_lines.begin(), sort_lines.end(), [](const line_t &a, const line_t &b) {
      return a.orientation < b.orientation;
    });

    std::vector<beam_t> beams;
    beam_t beam;
    double total_orientation = 0.0;
    for (size_t i = 0, io = 0; i < sort_lines.size(); i++, io++) {
      if ((i > 0 && fabs(sort_lines[i].orientation - sort_lines[i - 1].orientation) > orientation_diff) || i == sort_lines.size() - 1) {
        beam.orientation = total_orientation / io;
        total_orientation = io = 0;
        extractLine(beam);
        beams.push_back(beam);
        beam = beam_t();
      }
      total_orientation += sort_lines[i].orientation;
      fillLine(sort_lines[i].points, beam.points, 5);
    }

    return beams;
  }

  static std::vector<cv::Vec4i> mergeBeams(const std::vector<beam_t> &beams) {
    std::vector<cv::Vec4i> afterLines;
    for (auto &beam : beams) {
      afterLines.insert(afterLines.end(), beam.lines.begin(), beam.lines.end());
    }
    return afterLines;
  }

private:

  struct line_t {
      cv::Vec4i points;
      double orientation = 0.0;
  };

  static void fillLine(const cv::Vec4i &pt, std::vector<Point2D> &points, double step) {
    points.emplace_back(pt[0], pt[1]);
    double length = hypot(pt[2] - pt[0], pt[3] - pt[1]);
    int iterations = (int)(length / step);
    double ori = atan2((pt[3] - pt[1]), (pt[2] - pt[0]));
    for(int i = 1; i < iterations; i++) {
      points.emplace_back(pt[0] + step * i * cos(ori), pt[1] + step * i * sin(ori));
    }
    points.emplace_back(pt[2], pt[3]);
  }

  static void extractLine(beam_t &beam) {
    utils::KDTree<Point2D> tree(beam.points);
    auto clusters = tree.cluster(10);

    for(auto &cluster : clusters) {
      if(cluster.size() < 5) continue;
      for(auto &pt : cluster)
        pt = pt.rotate(-beam.orientation/180*M_PI);

      std::sort(cluster.begin(), cluster.end(), [](Point2D &a, Point2D &b) {
        return a.x < b.x;
      });

      auto first = ((*cluster.begin() + *(cluster.begin()+1))/2).rotate(beam.orientation/180*M_PI);
      auto last = ((*(cluster.end()-1) + *(cluster.end()-2))/2).rotate(beam.orientation/180*M_PI);
      cv::Vec4i line(first.x, first.y, last.x, last.y);
      beam.lines.push_back(line);
    }
  }
};

#endif //SRC_LINE_MERGER_HPP
