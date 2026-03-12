#pragma once

#include <string>

#include "libmavnetics/ui/core.h"
#include "libmavnetics/utils/rotation2d.hpp"

namespace libmavnetics {
namespace gui {

class SwerveVisualizer {
public:
  SwerveVisualizer(std::string name = "Swerve Drive");

  void focus();

private:
  std::array<lv_point_precise_t, 5> transformArrow(int length, Rotation2D angle);

  rd_view_t *view;
  std::string name;
};

} // namespace gui
} // namespace libmavnetics
