#include "libmavnetics/ui/views/swerve.hpp"
#include "globals.hpp"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/misc/lv_style_gen.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/line/lv_line.h"
#include "libmavnetics/ui/core.h"
#include "libmavnetics/ui/utils.h"
#include "libmavnetics/utils/rotation2d.hpp"
#include "units/angle.h"
#include <array>
#include <string>

namespace libmavnetics {
namespace gui {

std::array<lv_point_precise_t, 5>
SwerveVisualizer::transformArrow(int length, Rotation2D angle) {
  double cos_theta = (-angle).cos();
  double sin_theta = (-angle).sin();
  std::array<lv_point_precise_t, 5> points{
      {{0, 0}, {0, length}, {-5, length - 5}, {0, length}, {5, length - 5}}};
  for (int i = 0; i < 5; i++) {
    int x = points[i].x;
    int y = points[i].y;
    int new_x = (x - 12) * cos_theta - (y - 12) * sin_theta + 12;
    int new_y = (x - 12) * sin_theta + (y - 12) * cos_theta + 12;
    // std::cout << "(" << new_x << ", " << new_y << "), ";
    points[i] = {new_x, new_y};
  }
  return points;
}

SwerveVisualizer::SwerveVisualizer(
    std::string name, units::meters_per_second_t maxModuleSpeed,
    units::radians_per_second_t maxRotationalVelocity)
    : maxModuleVelocity(maxModuleSpeed),
      maxRotationalVelocity(maxRotationalVelocity) {

  this->name = name;
  this->view = rd_view_create(name.c_str());

  lv_obj_set_style_bg_color(view->obj, color_bg, 0);
  lv_obj_remove_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *chassis = lv_line_create(view->obj);
  lv_obj_set_style_line_width(chassis, 4, 0);
  lv_obj_set_style_line_rounded(chassis, true, 0);
  lv_obj_set_style_line_color(chassis, LV_COLOR_MAKE(50, 205, 50), 0);
  lv_line_set_points(chassis, chassis_frame.data(), 5);
  lv_obj_center(chassis);

  return;

  arrow_points_fl = transformArrow(40, 135_deg);
  arrow_points_fr = transformArrow(40, 45_deg);
  arrow_points_bl = transformArrow(40, 225_deg);
  arrow_points_br = transformArrow(40, 315_deg);
  arrow_points_chassis = transformArrow(40, 315_deg);

  arrow_fl = lv_line_create(view->obj);
  lv_obj_set_style_line_width(arrow_fl, 4, 0);
  lv_obj_set_style_line_rounded(arrow_fl, true, 0);
  lv_obj_set_style_line_color(arrow_fl, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_obj_align(arrow_fl, LV_ALIGN_CENTER, -75, -75);
  lv_line_set_points(arrow_fl, arrow_points_fl.begin(), 5);

  arrow_fr = lv_line_create(view->obj);
  lv_obj_set_style_line_width(arrow_fr, 4, 0);
  lv_obj_set_style_line_rounded(arrow_fr, true, 0);
  lv_obj_set_style_line_color(arrow_fr, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_obj_align(arrow_fr, LV_ALIGN_CENTER, 75, -75);
  lv_line_set_points(arrow_fr, arrow_points_fr.begin(), 5);

  arrow_bl = lv_line_create(view->obj);
  lv_obj_set_style_line_width(arrow_bl, 4, 0);
  lv_obj_set_style_line_rounded(arrow_bl, true, 0);
  lv_obj_set_style_line_color(arrow_bl, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_obj_align(arrow_bl, LV_ALIGN_CENTER, 75, 75);
  lv_line_set_points(arrow_bl, arrow_points_bl.begin(), 5);

  arrow_br = lv_line_create(view->obj);
  lv_obj_set_style_line_width(arrow_br, 4, 0);
  lv_obj_set_style_line_rounded(arrow_br, true, 0);
  lv_obj_set_style_line_color(arrow_br, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_obj_align(arrow_br, LV_ALIGN_CENTER, -75, 75);
  lv_line_set_points(arrow_br, arrow_points_br.begin(), 5);

  arrow_chassis = lv_line_create(view->obj);
  lv_obj_set_style_line_width(arrow_chassis, 4, 0);
  lv_obj_set_style_line_rounded(arrow_chassis, true, 0);
  lv_obj_align(arrow_chassis, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_line_color(arrow_chassis, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_line_set_points(arrow_chassis, arrow_points_chassis.begin(), 5);
}

void SwerveVisualizer::focus() { rd_view_focus(this->view); }

} // namespace gui
} // namespace libmavnetics
