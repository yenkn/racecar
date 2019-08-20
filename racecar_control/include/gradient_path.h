//
// Created by yenkn on 19-5-5.
//

#ifndef PROJECT_GRADIENT_PATH_H
#define PROJECT_GRADIENT_PATH_H

#include <math.h>
#include <vector>

#define POT_HIGH INT_MAX

class GradientPath {
public:
  GradientPath();
  ~GradientPath();

  void setSize(int xs, int ys);

  //
  // Path construction
  // Find gradient at array points, interpolate path
  // Use step size of pathStep, usually 0.5 pixel
  //
  // Some sanity checks:
  //  1. Stuck at same index position
  //  2. Doesn't get near goal
  //  3. Surrounded by high potentials
  //
  bool getPath(int* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);

  inline int getIndex(int x, int y) {
    return x + y * xs_;
  }
  void setLethalCost(unsigned char lethal_cost) {
    lethal_cost_ = lethal_cost;
  }
protected:
  int xs_, ys_;
  unsigned char lethal_cost_;
private:
  inline int getNearestPoint(int stc, float dx, float dy) {
    int pt = stc + (int)round(dx) + (int)(xs_ * round(dy));
    return std::max(0, std::min(xs_ * ys_ - 1, pt));
  }
  float gradCell(int* potential, int n);

  float *gradx_, *grady_; /**< gradient arrays, size of potential array */

  float pathStep_; /**< step size for following gradient */
};


#endif //PROJECT_GRADIENT_PATH_H
