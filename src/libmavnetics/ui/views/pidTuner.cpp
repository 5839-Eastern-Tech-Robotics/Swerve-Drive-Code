#include "libmavnetics/ui/views/pidTuner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <iostream>
#include <string>

#include "globals.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_event.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/layouts/flex/lv_flex.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/chart/lv_chart.h"
#include "liblvgl/widgets/dropdown/lv_dropdown.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/widgets/scale/lv_scale.h"
#include "liblvgl/widgets/textarea/lv_textarea.h"
#include "libmavnetics/ui/utils.h"
#include "libmavnetics/utils/pid.hpp"
#include "libmavnetics/utils/util.hpp"

namespace libmavnetics {
namespace gui {

// ============================== UI Callbacks ============================== //

void PIDTuner::p_selected_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_keyboard_set_textarea(tuner->numpad, tuner->ta_p);
  lv_obj_remove_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
}

void PIDTuner::i_selected_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_keyboard_set_textarea(tuner->numpad, tuner->ta_i);
  lv_obj_remove_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
}

void PIDTuner::d_selected_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_keyboard_set_textarea(tuner->numpad, tuner->ta_d);
  lv_obj_remove_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
}

void PIDTuner::run_test_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));

  if (tuner->running_test) {
    lv_label_set_text(tuner->test_label, "Test");
    tuner->running_test = false;
  } else {
    lv_label_set_text(tuner->test_label, "Stop");
    float kP = std::stof(lv_textarea_get_text(tuner->ta_p));
    float kI = std::stof(lv_textarea_get_text(tuner->ta_i));
    float kD = std::stof(lv_textarea_get_text(tuner->ta_d));
    lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
    tuner->selected_pid->pid->setGains(kP, kI, kD);
    tuner->running_test = true;
  }
}

void PIDTuner::reset_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_label_set_text(tuner->test_label, "Test");
  tuner->running_test = false;
  lv_chart_set_all_value(tuner->graph, tuner->error_series, 0);
  lv_chart_set_range(tuner->graph, LV_CHART_AXIS_PRIMARY_Y, -20, 20);
  lv_scale_set_range(tuner->scale, -20, 20);
  lv_chart_refresh(tuner->graph);
}

void PIDTuner::select_cb(lv_event_t *event) {
  lv_obj_t *pid_list = lv_event_get_target_obj(event);
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  int index = lv_dropdown_get_selected(pid_list);
  tuner->selectPID(index);
}

void PIDTuner::numpad_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (lv_keyboard_get_selected_button(tuner->numpad) == 3) {
    lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  } else if (lv_keyboard_get_selected_button(tuner->numpad) == 7) {
    lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
    // check button - set constants
    // float kP = atof(lv_textarea_get_text(tuner->ta_p));
    // float kI = atof(lv_textarea_get_text(tuner->ta_i));
    // float kD = atof(lv_textarea_get_text(tuner->ta_d));
    // tuner->selected_pid->pid->setGains(kP, kI, kD);
  }
  // lv_textarea_set_text(tuner->ta_p,
  // std::to_string(lv_keyboard_get_selected_button(tuner->numpad)).c_str());
}

void PIDTuner::update_graph(lv_timer_t *timer) {
  PIDTuner *tuner = (PIDTuner *)lv_timer_get_user_data(timer);
  if (!tuner->running_test)
    return;

  tuner->selected_pid->update();
  float error = tuner->selected_pid->getError == nullptr
                    ? tuner->selected_pid->pid->getError()
                    : tuner->selected_pid->getError();

  std::rotate(tuner->error_values.begin(), tuner->error_values.begin() + 1,
              tuner->error_values.end());
  tuner->error_values.back() = error;
  double max =
      *std::max_element(tuner->error_values.begin(), tuner->error_values.end());
  // lv_chart_set_next_value(tuner->graph, tuner->error_series, error);

  double scale = 1;
  double graphScale = 1;
  if (max < 1)
    scale = std::pow(10, std::floor(std::log10(max)));
  if (max < 100)
    graphScale = std::pow(10, std::floor(std::log10(max) - 2));

  std::transform(tuner->error_values.begin(), tuner->error_values.end(),
                 tuner->scaled_error_values.begin(),
                 [graphScale](double value) {
                   return static_cast<std::int32_t>(value / graphScale);
                 });

  std::cout << "max: " << max << ", scale: " << scale
            << ", graph scale: " << graphScale << ", ";
  std::cout << "error: " << error << ", scaled error: " << error / graphScale
            << std::endl;
  // std::cout << "values: ";
  for (auto &value : tuner->scaled_error_values) {
    lv_chart_set_next_value(tuner->graph, tuner->error_series, value);
    // std::cout << value << ", ";
  }
  // std::cout << std::endl;

  lv_label_set_text(tuner->scale_label,
                    std::format("Scale: {}", scale).c_str());
  lv_chart_set_range(tuner->graph, LV_CHART_AXIS_PRIMARY_Y, -max / graphScale,
                     max / graphScale);
  lv_scale_set_range(tuner->scale, -max / scale, max / scale);

  lv_chart_refresh(tuner->graph);
}

// ============================== Constructor ============================== //

PIDTuner::PIDTuner(std::vector<PID_t> pids) : PIDTuner("PID Tuner", pids) {}

PIDTuner::PIDTuner(std::string name, std::vector<PID_t> pids) {
  this->name = name;
  this->pids = pids;
  selected_pid = nullptr;

  this->view = rd_view_create(name.c_str());
  lv_obj_set_style_bg_color(view->obj, color_bg, 0);

  // ----------------------------- Left Side UI ----------------------------- //
  pid_cont = lv_obj_create(view->obj);
  lv_obj_add_style(pid_cont, &style_transp, 0);
  lv_obj_set_layout(pid_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(pid_cont, 180, 220);
  lv_obj_align(pid_cont, LV_ALIGN_LEFT_MID, 10, 0);
  lv_obj_set_flex_flow(pid_cont, LV_FLEX_FLOW_COLUMN);

  pid_dropdown = lv_dropdown_create(pid_cont);
  lv_dropdown_clear_options(pid_dropdown);
  lv_obj_set_size(pid_dropdown, 140, 40);
  lv_obj_add_event_cb(pid_dropdown, select_cb, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_set_user_data(pid_dropdown, this);
  lv_obj_set_flex_align(pid_dropdown, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  for (int i = 0; i < pids.size(); i++)
    lv_dropdown_add_option(pid_dropdown, pids[i].name.c_str(), i);

  ta_p = lv_textarea_create(pid_cont);
  ta_i = lv_textarea_create(pid_cont);
  ta_d = lv_textarea_create(pid_cont);
  lv_obj_set_size(ta_p, 140, 40);
  lv_obj_set_size(ta_i, 140, 40);
  lv_obj_set_size(ta_d, 140, 40);

  lv_obj_set_flex_align(ta_p, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_flex_align(ta_i, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_flex_align(ta_d, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_add_event_cb(ta_p, &p_selected_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(ta_i, &i_selected_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(ta_d, &d_selected_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_user_data(ta_p, this);
  lv_obj_set_user_data(ta_i, this);
  lv_obj_set_user_data(ta_d, this);

  lv_textarea_set_placeholder_text(ta_p, "0.0");
  lv_textarea_set_placeholder_text(ta_i, "0.0");
  lv_textarea_set_placeholder_text(ta_d, "0.0");
  lv_textarea_set_text(ta_p, "0");
  lv_textarea_set_text(ta_i, "0");
  lv_textarea_set_text(ta_d, "0");

  lv_textarea_set_accepted_chars(ta_p, "+-.1234567890");
  lv_textarea_set_accepted_chars(ta_i, "+-.1234567890");
  lv_textarea_set_accepted_chars(ta_d, "+-.1234567890");

  selectPID(0);

  // ----------------------------- Right Side UI -----------------------------
  lv_obj_t *graph_cont = lv_obj_create(view->obj);
  lv_obj_add_style(graph_cont, &style_transp, 0);
  lv_obj_set_layout(graph_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(graph_cont, 280, 220);
  lv_obj_align(graph_cont, LV_ALIGN_CENTER, 55, 0);
  lv_obj_set_flex_flow(graph_cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_remove_flag(graph_cont, LV_OBJ_FLAG_SCROLLABLE);

  scale_label = lv_label_create(graph_cont);
  lv_obj_add_style(scale_label, &style_text_small, 0);
  lv_obj_set_style_text_align(scale_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_size(scale_label, lv_pct(100), 10);
  lv_obj_align(scale_label, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(scale_label, "Scale: 1");

  lv_obj_t *graph_wrapper = lv_obj_create(graph_cont);
  lv_obj_remove_style_all(graph_wrapper);
  lv_obj_set_size(graph_wrapper, lv_pct(100), 150);
  lv_obj_set_flex_flow(graph_wrapper, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(graph_wrapper, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  scale = lv_scale_create(graph_wrapper);
  lv_scale_set_mode(scale, LV_SCALE_MODE_VERTICAL_LEFT);
  lv_obj_set_size(scale, 50, lv_pct(90));
  lv_obj_align(scale, LV_ALIGN_LEFT_MID, 0, 0);
  lv_scale_set_total_tick_count(scale, 9);
  lv_scale_set_major_tick_every(scale, 1);
  lv_scale_set_range(scale, -20, 20);
  lv_obj_set_style_pad_hor(
      scale, lv_chart_get_first_point_center_offset(graph_wrapper) + 5, 0);

  graph = lv_chart_create(graph_wrapper);
  lv_obj_set_height(graph, lv_pct(90));
  lv_obj_set_flex_grow(graph, 1);
  lv_chart_set_type(graph, LV_CHART_TYPE_LINE);
  lv_chart_set_update_mode(graph, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(graph, 50);
  lv_chart_set_range(graph, LV_CHART_AXIS_PRIMARY_Y, -20, 20);
  lv_obj_set_style_size(graph, 0, 0, LV_PART_INDICATOR);
  error_series = lv_chart_add_series(graph, LV_COLOR_MAKE(34, 139, 34),
                                     LV_CHART_AXIS_PRIMARY_Y);
  // lv_chart_set_ext_y_array(graph, error_series, scaled_error_values.begin());

  lv_obj_t *btn_cont = lv_obj_create(graph_cont);
  lv_obj_add_style(btn_cont, &style_transp, 0);
  lv_obj_set_layout(btn_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(btn_cont, lv_pct(100), 30);
  lv_obj_align(btn_cont, LV_ALIGN_CENTER, -20, 0);
  lv_obj_remove_flag(btn_cont, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);

  lv_obj_t *test_btn = lv_button_create(btn_cont);
  lv_obj_t *reset_btn = lv_button_create(btn_cont);
  lv_obj_set_size(test_btn, 80, 30);
  lv_obj_set_size(reset_btn, 80, 30);
  lv_obj_set_flex_align(test_btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_flex_align(reset_btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_add_event_cb(test_btn, &run_test_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(reset_btn, &reset_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_user_data(test_btn, this);
  lv_obj_set_user_data(reset_btn, this);

  test_label = lv_label_create(test_btn);
  lv_label_set_text(test_label, "Test");
  lv_obj_t *reset_label = lv_label_create(reset_btn);
  lv_label_set_text(reset_label, "Reset");

  // ----------------------------- Numpad ----------------------------- //
  numpad = lv_keyboard_create(view->obj);
  lv_keyboard_set_mode(numpad, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_add_event_cb(numpad, &numpad_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_user_data(numpad, this);
  lv_obj_set_size(numpad, 240, 120);
  lv_obj_align_to(numpad, graph, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(numpad, LV_OBJ_FLAG_HIDDEN);

  lv_timer_create(update_graph, 100, this);
}

void PIDTuner::selectPID(int index) {
  this->selected_pid = &pids[index];
  lv_dropdown_set_text(this->pid_dropdown, this->selected_pid->name.c_str());
  lv_textarea_set_text(ta_p, std::to_string(selected_pid->pid->getP()).c_str());
  lv_textarea_set_text(ta_i, std::to_string(selected_pid->pid->getI()).c_str());
  lv_textarea_set_text(ta_d, std::to_string(selected_pid->pid->getD()).c_str());
}

void PIDTuner::focus() { rd_view_focus(this->view); }

} // namespace gui
} // namespace libmavnetics
