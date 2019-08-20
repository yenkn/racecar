//
// Created by yenkn on 19-3-17.
//

#ifndef PROJECT_TABLE_INTERPOLATION_H
#define PROJECT_TABLE_INTERPOLATION_H

#include <map>
#include <list>
#include "spline.h"

enum class InterpolateMode {
    Linear,
    Spline,
};

enum class BoundaryType {
  Clamp,
  Linear,
  Continuous,
};

class TableInterpolation {
public:
  explicit TableInterpolation(InterpolateMode mode, BoundaryType boundary)
    : mode_(mode), boundaryType_(boundary) {}

  size_t size() { return table_.size(); }
  void setPoints(const std::map<double, double> &table);
  double interpolate(double value);

private:
  std::map<double, double> table_;
  InterpolateMode mode_;
  BoundaryType boundaryType_;
  tk::spline spline_;
};

#endif //PROJECT_LINEAR_INTERPOLATION_H
