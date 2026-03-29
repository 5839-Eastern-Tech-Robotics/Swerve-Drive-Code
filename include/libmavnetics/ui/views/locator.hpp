#pragma once

#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_types.h"
#include "libmavnetics/ui/core.h"
#include "libmavnetics/utils/pose2d.hpp"
#include <array>
#include <optional>
#include <string>

extern lv_image_t *empty_field_small;

namespace libmavnetics {
namespace gui {

class OdomLocator {
public:
  OdomLocator(std::optional<void *> field_path = std::nullopt,
              std::string name = "Odometry");

  void update(libmavnetics::Pose2D pose);
  void focus();

private:
  rd_view_t *view;

  lv_obj_t *robot;
  lv_obj_t *label_x;
  lv_obj_t *label_y;
  lv_obj_t *label_0;

  std::array<lv_point_precise_t, 11> robot_points;
  libmavnetics::Pose2D robot_pose{0_in, 0_in, 0_deg};
  std::string name;

  static void update_robot(lv_timer_t *timer);
  static void mode_cb(lv_event_t *event);

  bool mode = false; // false = position, true = angle
  lv_obj_t *mode_label;
};

} // namespace gui
} // namespace libmavnetics
