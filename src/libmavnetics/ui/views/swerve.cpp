#include "libmavnetics/ui/views/swerve.hpp"
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

std::array<lv_point_precise_t, 5> SwerveVisualizer::transformArrow(int length, Rotation2D angle) {
  double cos_theta = (-angle).cos();
  double sin_theta = (-angle).sin();
  std::array<lv_point_precise_t, 5> points{{ {0, 0}, {0, length}, {-5, length - 5}, {0, length}, {5, length - 5} }};
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

SwerveVisualizer::SwerveVisualizer(std::string name) {
  this->name = name;
  this->view = rd_view_create(name.c_str());

  lv_obj_set_style_bg_color(view->obj, color_bg, 0);
  lv_obj_remove_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);

  lv_style_t style_line;
  lv_style_init(&style_line);
  lv_style_set_line_width(&style_line, 4);
  lv_style_set_line_rounded(&style_line, true);

  lv_obj_t *chassis = lv_line_create(view->obj);
  lv_obj_add_style(chassis, &style_line, 0);
  lv_obj_set_style_line_width(chassis, 4, 0);
  lv_obj_set_style_line_color(chassis, LV_COLOR_MAKE(50, 205, 50), 0);
  lv_point_precise_t points[] = {
      {0, 0}, {0, 150}, {150, 150}, {150, 0}, {0, 0}};
  lv_line_set_points(chassis, points, 5);
  lv_obj_center(chassis);

  std::array<lv_point_precise_t, 5> arrow_points = transformArrow(40, 30_deg);

  lv_obj_t* arrow_1 = lv_line_create(view->obj);
  lv_obj_add_style(arrow_1, &style_line, 0);
  lv_obj_align(arrow_1, LV_ALIGN_CENTER, -75, -75);
  lv_obj_set_style_line_color(arrow_1, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_line_set_points(arrow_1, arrow_points.begin(), 5);

  lv_obj_t* arrow_2 = lv_line_create(view->obj);
  lv_obj_add_style(arrow_2, &style_line, 0);
  lv_obj_align(arrow_2, LV_ALIGN_CENTER, 75, -75);
  lv_obj_set_style_line_color(arrow_2, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_line_set_points(arrow_2, arrow_points.begin(), 5);

  lv_obj_t* arrow_3 = lv_line_create(view->obj);
  lv_obj_add_style(arrow_3, &style_line, 0);
  lv_obj_align(arrow_3, LV_ALIGN_CENTER, 75, 75);
  lv_obj_set_style_line_color(arrow_3, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_line_set_points(arrow_3, arrow_points.begin(), 5);

  lv_obj_t* arrow_4 = lv_line_create(view->obj);
  lv_obj_add_style(arrow_4, &style_line, 0);
  lv_obj_align(arrow_4, LV_ALIGN_CENTER, -75, 75);
  lv_obj_set_style_line_color(arrow_4, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_line_set_points(arrow_4, arrow_points.begin(), 5);

  lv_obj_t* arrow_5 = lv_line_create(view->obj);
  lv_obj_add_style(arrow_5, &style_line, 0);
  lv_obj_align(arrow_5, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_line_color(arrow_5, LV_COLOR_MAKE(65, 105, 225), 0);
  lv_line_set_points(arrow_5, arrow_points.begin(), 5);
}

void SwerveVisualizer::focus() { rd_view_focus(this->view); }

} // namespace gui
} // namespace libmavnetics
