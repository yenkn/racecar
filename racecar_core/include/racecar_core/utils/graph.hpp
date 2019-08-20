//
// Created by yenkn on 19-8-9.
//

#ifndef PROJECT_GRAPH_HPP
#define PROJECT_GRAPH_HPP

#include <vector>
#include "math.h"

#define STAGE_POINTS 4

namespace utils {

typedef std::array<Point2D, STAGE_POINTS> polygon_t;
typedef std::array<Point2D, 2> checkline_t;

struct distance_t {
  double forward, backward, leftward, rightward;

  distance_t(): forward(0.0), backward(0.0), leftward(0.0), rightward(0.0) {}
  distance_t(double f, double b, double l, double r): forward(f), backward(b), leftward(l), rightward(r) {}
};

double pointPolygonTest(const polygon_t &_contour, Point2D pt, bool measureDist)
{
  double result = 0;
  int i, total = STAGE_POINTS, counter = 0;
  double min_dist_num = FLT_MAX, min_dist_denom = 1;

  Point2D v0, v = _contour[total-1];
  if(!measureDist)
  {
    for(i = 0; i < total; i++)
    {
      double dist;
      v0 = v;
      v = _contour[i];

      if( (v0.y <= pt.y && v.y <= pt.y) ||
          (v0.y > pt.y && v.y > pt.y) ||
          (v0.x < pt.x && v.x < pt.x) )
      {
        if( pt.y == v.y && (pt.x == v.x || (pt.y == v0.y && ((v0.x <= pt.x && pt.x <= v.x) || (v.x <= pt.x && pt.x <= v0.x)))) )
          return 0;
        continue;
      }

      dist = (double)(pt.y - v0.y)*(v.x - v0.x) - (double)(pt.x - v0.x)*(v.y - v0.y);
      if( dist == 0 )
        return 0;
      if( v.y < v0.y )
        dist = -dist;
      counter += dist > 0;
    }

    result = counter % 2 == 0 ? -1 : 1;
  }
  else
  {
    for( i = 0; i < total; i++ )
    {
      double dx, dy, dx1, dy1, dx2, dy2, dist_num, dist_denom = 1;

      v0 = v;
      v = _contour[i];

      dx = v.x - v0.x; dy = v.y - v0.y;
      dx1 = pt.x - v0.x; dy1 = pt.y - v0.y;
      dx2 = pt.x - v.x; dy2 = pt.y - v.y;

      if( dx1*dx + dy1*dy <= 0 )
        dist_num = dx1*dx1 + dy1*dy1;
      else if( dx2*dx + dy2*dy >= 0 )
        dist_num = dx2*dx2 + dy2*dy2;
      else
      {
        dist_num = (dy1*dx - dx1*dy);
        dist_num *= dist_num;
        dist_denom = dx*dx + dy*dy;
      }

      if( dist_num*min_dist_denom < min_dist_num*dist_denom )
      {
        min_dist_num = dist_num;
        min_dist_denom = dist_denom;
        if( min_dist_num == 0 )
          break;
      }

      if( (v0.y <= pt.y && v.y <= pt.y) ||
          (v0.y > pt.y && v.y > pt.y) ||
          (v0.x < pt.x && v.x < pt.x) )
        continue;

      dist_num = dy1*dx - dx1*dy;
      if( dy < 0 )
        dist_num = -dist_num;
      counter += dist_num > 0;
    }

    result = std::sqrt(min_dist_num/min_dist_denom);
    if( counter % 2 == 0 )
      result = -result;
  }

  return result;
}

std::vector<polygon_t> checklinesToStages(std::vector<checkline_t> lines, int uTurnIndex, const Point2D &start, const Point2D &goal) {
  std::vector<polygon_t> stages;
  if(lines.empty()) return stages;

  lines.insert(lines.begin(), {
      Point2D(start.x, lines[0][0].y),
      Point2D(start.x, lines[0][1].y)
  });
  uTurnIndex++;
  lines.push_back({
    Point2D(goal.x, lines.back()[0].y),
    Point2D(goal.x, lines.back()[1].y)
  });

  auto uturnMiddle = (lines[uTurnIndex][0] + lines[uTurnIndex][1]) / 2;
  for(int i = 0; i < lines.size()-1; i++) {
    if (i == uTurnIndex - 1) {
      // enter u-turn
      polygon_t polygon = { lines[i][0], lines[i][1], uturnMiddle, lines[uTurnIndex][0] };
      stages.push_back(polygon);
    } else if(i == uTurnIndex) {
      polygon_t polygon = { lines[i][1], uturnMiddle, lines[i+1][1], lines[i+1][0] };
      stages.push_back(polygon);
    } else {
      polygon_t polygon = { lines[i][0], lines[i][1], lines[i + 1][1], lines[i + 1][0] };
      stages.push_back(polygon);
    }
  }
  return stages;
}

int pointStageTest(const std::vector<polygon_t> &stages, const Point2D &point, int currentStage = 0) {
  int stage = -1;
  double minDistance = -FLT_MAX;

  for (int i = currentStage; i < stages.size(); i++) {
    auto result = pointPolygonTest(stages[i], point, true);
    if (result >= 0) {
      stage = i;
      break;
    } else if(result > minDistance) {
      minDistance = result;
    }
  }

  return stage;
}

distance_t pointStageDistance(const polygon_t &stage, const Point2D &point) {
  distance_t distances;
  distances.backward = distanceToLine(point, stage[0], stage[1]);
  distances.forward = distanceToLine(point, stage[2], stage[3]);
  distances.leftward = distanceToLine(point, stage[0], stage[3]);
  distances.rightward = distanceToLine(point, stage[1], stage[2]);
  return distances;
}


}

#endif //PROJECT_GRAPH_HPP
