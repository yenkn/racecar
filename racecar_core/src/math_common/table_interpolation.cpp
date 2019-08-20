//
// Created by yenkn on 19-3-17.
//

#include "racecar_core/math_common/table_interpolation.h"

#include <vector>
#include <iterator>

void TableInterpolation::setPoints(const std::map<double, double> &table) {
  table_ = table;

  std::vector<double> xs;
  std::vector<double> ys;
  for (auto const &pair : table_) {
    xs.push_back(pair.first);
    ys.push_back(pair.second);
  }
  spline_ = tk::spline();
  if(boundaryType_ != BoundaryType::Clamp) {
    auto bd_type = boundaryType_ == BoundaryType::Linear ? tk::spline::second_deriv : tk::spline::first_deriv;
    spline_.set_boundary(bd_type, 0.0, bd_type, 0.0);
  }
  spline_.set_points(xs, ys, mode_ == InterpolateMode::Spline);
}

double TableInterpolation::interpolate(double key) {
  if(boundaryType_ == BoundaryType::Clamp) {
    auto firstKey = table_.cbegin()->first;
    auto lastKey = std::prev(table_.cend())->first;
    if(firstKey > key) {
      return table_[firstKey];
    } else if(lastKey < key) {
      return table_[lastKey];
    }
  }

  return spline_(key);
}
