//
// Created by yenkn on 19-7-24.
//

#ifndef PROJECT_CUBIC_BEZIER_HPP
#define PROJECT_CUBIC_BEZIER_HPP

namespace utils {
class CubicBezier {
public:
  double mX1, mY1, mX2, mY2;

  CubicBezier(double X1, double Y1, double X2, double Y2): mX1(X1), mY1(Y1), mX2(X2), mY2(Y2) {}

  double get(double aX) {
  if (mX1 == mY1 && mX2 == mY2) return aX; // linear
  return CalcBezier(GetTForX(aX), mY1, mY2);
  }

  inline double A(double aA1, double aA2) { return 1.0 - 3.0 * aA2 + 3.0 * aA1; }

  inline double B(double aA1, double aA2) { return 3.0 * aA2 - 6.0 * aA1; }

  inline double C(double aA1) { return 3.0 * aA1; }

  // Returns x(t) given t, x1, and x2, or y(t) given t, y1, and y2.
  double CalcBezier(double aT, double aA1, double aA2) {
    return ((A(aA1, aA2) * aT + B(aA1, aA2)) * aT + C(aA1)) * aT;
  }

  // Returns dx/dt given t, x1, and x2, or dy/dt given t, y1, and y2.
  double GetSlope(double aT, double aA1, double aA2) {
    return 3.0 * A(aA1, aA2) * aT * aT + 2.0 * B(aA1, aA2) * aT + C(aA1);
  }

  double GetTForX(double aX) {
    // Newton raphson iteration
    double aGuessT = aX;
    for (int i = 0; i < 4; ++i) {
      double currentSlope = GetSlope(aGuessT, mX1, mX2);
      if (currentSlope == 0.0) return aGuessT;
      double currentX = CalcBezier(aGuessT, mX1, mX2) - aX;
      aGuessT -= currentX / currentSlope;
    }
    return aGuessT;
  }
};

}

#endif //PROJECT_CUBIC_BEZIER_HPP
