//
// Created by yenkn on 19-5-6.
//

#ifndef PROJECT_GRID_PATH_H
#define PROJECT_GRID_PATH_H

#include <vector>

class GridPath {
protected:
  int xs_, ys_;
  unsigned char lethal_cost_;
public:
  inline int getIndex(int x, int y) {
    return x + y * xs_;
  }

  void setSize(int xs, int ys) {
    xs_ = xs;
    ys_ = ys;
  }

  GridPath() {}
  bool getPath(int* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
};

#endif //PROJECT_GRID_PATH_H
