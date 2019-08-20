//
// Created by yenkn on 19-7-8.
//

#ifndef SRC_COLOR_H
#define SRC_COLOR_H

#include <std_msgs/ColorRGBA.h>
namespace utils {
class Color {
public:
  Color() = default;

  Color(float r, float g, float b) : r_(r), g_(g), b_(b), a_(1.0) {}

  inline std_msgs::ColorRGBA toColorRGBA() const {
    std_msgs::ColorRGBA color;
    color.r = r_;
    color.g = g_;
    color.b = b_;
    color.a = a_;
    return color;
  }

  static Color fromRGB(float r, float g, float b) {
    return Color(std::min(r, 1.0f), std::min(g, 1.0f), std::min(b, 1.0f));
  }

  /*
   * H(Hue): 0 - 360 degree (integer)
   * S(Saturation): 0 - 1.00 (double)
   * V(Value): 0 - 1.00 (double)
   */
  static Color fromHSV(int H, double S, double V) {
    double C = S * V;
    double X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
    double m = V - C;
    double Rs, Gs, Bs;

    if (H >= 0 && H < 60) {
      Rs = C;
      Gs = X;
      Bs = 0;
    } else if (H >= 60 && H < 120) {
      Rs = X;
      Gs = C;
      Bs = 0;
    } else if (H >= 120 && H < 180) {
      Rs = 0;
      Gs = C;
      Bs = X;
    } else if (H >= 180 && H < 240) {
      Rs = 0;
      Gs = X;
      Bs = C;
    } else if (H >= 240 && H < 300) {
      Rs = X;
      Gs = 0;
      Bs = C;
    } else {
      Rs = C;
      Gs = 0;
      Bs = X;
    }

    return Color(Rs + m, Gs + m, Bs + m);
  }

  double r_, g_, b_, a_;

};
}

#endif //SRC_COLOR_H
