//
// Created by yenkn on 19-3-17.
//

#ifndef PROJECT_MATH_UTILS_H
#define PROJECT_MATH_UTILS_H

#include <utility>

/**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Calculate the difference between angle from and to
 * @param from the start angle
 * @param from the end angle
 * @return The difference between from and to. The range is between [0, PI).
 */
double AngleDiff(const double from, const double to);

/**
 * @brief Get a random integer between two integer values by a random seed.
 * @param s The lower bound of the random integer.
 * @param t The upper bound of the random integer.
 * @param random_seed The random seed.
 * @return A random integer between s and t based on the input random_seed.
 */
int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

/**
 * @brief Get a random double between two integer values by a random seed.
 * @param s The lower bound of the random double.
 * @param t The upper bound of the random double.
 * @param random_seed The random seed.
 * @return A random double between s and t based on the input random_seed.
 */
double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

// Gaussian
double Gaussian(const double u, const double std, const double x);

// Sigmoid
double Sigmoid(const double x);

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);

#endif //PROJECT_MATH_UTILS_H
