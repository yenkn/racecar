//
// Created by yenkn on 19-4-27.
//

#ifndef PROJECT_LINEAR_INTERPOLATOR_H
#define PROJECT_LINEAR_INTERPOLATOR_H

#include <map>

template <class T>
class LinearInterpolator {
public:
  LinearInterpolator() = default;
  void clear() {
    interpolateMap_.clear();
  }

  void addPoint(T x, T y) {
    interpolateMap_[x] = y;
  }

  T interpolate(T x) {
    if(interpolateMap_.empty()) {
      return 0.0;
    }
    auto iter = interpolateMap_.upper_bound(x); // first element greater than x
    T p1x, p1y, p2x, p2y;
    if(iter == interpolateMap_.end()) {
      p2x = interpolateMap_.rbegin()->first;
      p2y  = interpolateMap_.rbegin()->second;
      p1x = std::prev(interpolateMap_.rbegin())->first;
      p1y = std::prev(interpolateMap_.rbegin())->second;
    } else {
      p1x = iter->first;
      p1y = iter->second;
      p2x = std::next(iter)->first;
      p2y = std::next(iter)->second;
    }
    auto k = (p2y - p1y) / (p2x - p1x);
    return (x - p1x) * k + p1y;
  }

private:
  std::map<T, T> interpolateMap_;
};

#endif //PROJECT_LINEAR_INTERPOLATOR_H
