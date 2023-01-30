#pragma once

#ifndef COLOR_UTIL
#define COLOR_UTIL

#include <vector>
#include <Eigen/Dense>

struct Color {
  int r, g, b;
};


class ColorUtil {
 protected:
  std::vector<Color> colors;
  virtual void load_colors();

public:
  ColorUtil();
  void setColor(Eigen::Vector3d &color, int &label);
  void setColor_DL(Eigen::Vector3d &color, int &label);

};

class ColorUtil_SemKITTI : public ColorUtil {
protected:
  void load_colors();

public:
  ColorUtil_SemKITTI();
};

class ColorUtil_NuSC : public ColorUtil {
protected:
  void load_colors();

public:
  ColorUtil_NuSC();
};

#endif // COLOR_UTIL