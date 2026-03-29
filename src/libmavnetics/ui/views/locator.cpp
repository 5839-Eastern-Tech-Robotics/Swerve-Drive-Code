#include "libmavnetics/ui/views/locator.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_event.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/layouts/flex/lv_flex.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/widgets/line/lv_line.h"
#include "libmavnetics/ui/core.h"
#include "libmavnetics/ui/utils.h"
#include "libmavnetics/utils/pose2d.hpp"
#include "units/angle.h"
#include "units/base.h"
#include "units/length.h"
#include <optional>
#include <string>

namespace libmavnetics {
namespace gui {

void OdomLocator::mode_cb(lv_event_t *event) {
  OdomLocator *locator =
      (OdomLocator *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  locator->mode = !(locator->mode);
  if (locator->mode) {
    lv_label_set_text(locator->mode_label, "Mode:\nAngle");
  }
  if (!(locator->mode)) {
    lv_label_set_text(locator->mode_label, "Mode:\nPosition");
  }
}

void OdomLocator::update_robot(lv_timer_t *timer) {
  OdomLocator *locator = (OdomLocator *)lv_timer_get_user_data(timer);

  int x_offset = 16 * locator->robot_pose.x().convert<units::foot>().value();
  int y_offset = -16 * locator->robot_pose.y().convert<units::foot>().value();
  // std::cout << "X: " << x_offset << ", Y: " << y_offset << ", Points: ";

  double cos_theta = (-locator->robot_pose.rotation()).cos();
  double sin_theta = (-locator->robot_pose.rotation()).sin();

  lv_point_precise_t original_points[] = {{12, 18}, {18, 18}, {18, 6}, {6, 6},
                                          {6, 18},  {12, 18}, {9, 14}, {12, 18},
                                          {15, 14}, {12, 18}, {12, 9}};
  for (int i = 0; i < 11; i++) {
    int x = original_points[i].x;
    int y = original_points[i].y;
    int new_x = (x - 12) * cos_theta - (y - 12) * sin_theta + 12;
    int new_y = (x - 12) * sin_theta + (y - 12) * cos_theta + 12;
    // std::cout << "(" << new_x << ", " << new_y << "), ";
    locator->robot_points[i] = {new_x, new_y};
  }

  // std::cout << std::endl;
  lv_line_set_points(locator->robot, locator->robot_points.begin(),
                     locator->robot_points.size());
  lv_obj_align(locator->robot, LV_ALIGN_CENTER, x_offset, y_offset);

  lv_label_set_text(
      locator->label_x,
      std::format("X: {:.2f} in",
                  locator->robot_pose.x().convert<units::inch>().value())
          .c_str());

  lv_label_set_text(
      locator->label_y,
      std::format("Y: {:.2f} in",
                  locator->robot_pose.y().convert<units::inch>().value())
          .c_str());

  lv_label_set_text(
      locator->label_0,
      std::format("0: {:.1f} deg",
                  locator->robot_pose.rotation().degrees().value())
          .c_str());
}

OdomLocator::OdomLocator(std::optional<void *> field_path, std::string name) {
  this->name = name;
  this->view = rd_view_create(name.c_str());

  lv_obj_set_style_bg_color(view->obj, color_bg, 0);
  lv_obj_remove_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *info_cont = lv_obj_create(view->obj);
  lv_obj_add_style(info_cont, &style_transp, 0);
  lv_obj_set_size(info_cont, 228, 192);
  lv_obj_align(info_cont, LV_ALIGN_TOP_LEFT, 8, 40);
  lv_obj_remove_flag(info_cont, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(info_cont, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(info_cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(info_cont, LV_FLEX_ALIGN_SPACE_EVENLY,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  label_x = lv_label_create(info_cont);
  lv_label_set_text(label_x, "X: 0.000000 in");
  lv_obj_add_style(label_x, &style_text_mono, 0);
  lv_obj_add_style(label_x, &style_text_medium, 0);

  label_y = lv_label_create(info_cont);
  lv_label_set_text(label_y, "Y: 0.000000 in");
  lv_obj_add_style(label_y, &style_text_mono, 0);
  lv_obj_add_style(label_y, &style_text_medium, 0);

  label_0 = lv_label_create(info_cont);
  lv_label_set_text(label_0, "0: 0.000000 deg");
  lv_obj_add_style(label_0, &style_text_mono, 0);
  lv_obj_add_style(label_0, &style_text_medium, 0);

  lv_obj_t *button_list = lv_obj_create(info_cont);
  lv_obj_add_style(button_list, &style_transp, 0);
  lv_obj_set_size(button_list, 176, 32);
  lv_obj_remove_flag(button_list, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(button_list, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(button_list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(button_list, LV_FLEX_ALIGN_SPACE_EVENLY,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  // lv_obj_t *mode_btn = lv_button_create(button_list);
  // lv_obj_set_size(mode_btn, 67, 41);
  // lv_obj_set_flex_align(mode_btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
  //                       LV_FLEX_ALIGN_CENTER);
  // lv_obj_add_event_cb(mode_btn, &mode_cb, LV_EVENT_CLICKED, NULL);
  // lv_obj_set_user_data(mode_btn, this);
  // mode_label = lv_label_create(mode_btn);
  // lv_label_set_text(mode_label, "Mode:\nPosition");

  lv_obj_t *field_cont = lv_obj_create(view->obj);
  lv_obj_add_style(field_cont, &style_transp, 0);
  lv_obj_set_style_pad_all(field_cont, 0, 0);
  lv_obj_set_size(field_cont, 194, 194);
  lv_obj_align(field_cont, LV_ALIGN_TOP_RIGHT, -8, 40);
  lv_obj_remove_flag(field_cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *field_img = lv_image_create(field_cont);
  lv_obj_set_size(field_img, 192, 192);
  lv_obj_align(field_img, LV_ALIGN_TOP_RIGHT, 0, 0);
  if (field_path.has_value()) {
    lv_image_set_src(field_img, field_path.value());
  } else {
    lv_image_set_src(field_img, &empty_field_small);
  }

  this->robot_points = {{{12, 18},
                         {18, 18},
                         {18, 6},
                         {6, 6},
                         {6, 18},
                         {12, 18},
                         {9, 14},
                         {12, 18},
                         {15, 14},
                         {12, 18},
                         {12, 9}}};
  robot = lv_line_create(field_cont);
  lv_obj_set_style_line_width(robot, 2, 0);
  lv_obj_set_style_line_color(robot, LV_COLOR_MAKE(0x32, 0xCD, 0x32), 0);
  lv_obj_set_style_line_rounded(robot, true, 0);
  lv_line_set_y_invert(robot, true);
  lv_line_set_points(robot, robot_points.begin(), robot_points.size());
  lv_obj_align(robot, LV_ALIGN_CENTER, 0, 0);

  lv_timer_create(update_robot, 50, this);
}

void OdomLocator::update(libmavnetics::Pose2D pose) { this->robot_pose = pose; }

void OdomLocator::focus() { rd_view_focus(this->view); }

} // namespace gui
} // namespace libmavnetics
