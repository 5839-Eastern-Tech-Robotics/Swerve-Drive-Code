#include "libmavnetics/ui/views/pidTuner.hpp"

#include <cstring>

#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/widgets/chart/lv_chart.h"
#include "liblvgl/widgets/dropdown/lv_dropdown.h"
#include "liblvgl/widgets/textarea/lv_textarea.h"
#include "libmavnetics/ui/utils.h"
#include "libmavnetics/utils/pid.hpp"

namespace libmavnetics {
namespace gui {

// ============================== UI Callbacks ============================== //

void PIDTuner::p_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  lv_keyboard_set_textarea(tuner->numpad, tuner->ta_p);
  lv_obj_remove_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
}

void PIDTuner::i_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  lv_keyboard_set_textarea(tuner->numpad, tuner->ta_i);
  lv_obj_remove_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
}

void PIDTuner::d_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  lv_keyboard_set_textarea(tuner->numpad, tuner->ta_d);
  lv_obj_remove_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
}
void PIDTuner::test_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  float kP = std::stof(lv_textarea_get_text(tuner->ta_p));
  float kI = std::stof(lv_textarea_get_text(tuner->ta_i));
  float kD = std::stof(lv_textarea_get_text(tuner->ta_d));
  lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  // set kP, kI, kD to take effect
  tuner->selected_pid->pid->setGains(kP, kI, kD);
  // run_cb and graph
  for (int i = 0; i < 10; i++) { // TODO: stop button?
    tuner->run_cb();             // should include updating PID
    float error = tuner->selected_pid->pid->getError();
    lv_chart_set_next_value(tuner->graph, tuner->ser, error);
    lv_chart_refresh(tuner->graph);
  }
}

void PIDTuner::autotune_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  tuner->frozen = true;
  // tuner->selected_pid->pid->start_autotune(); // i might re-add this but i
  // dont think i have time lmao

  // convert this into a timer, disable all the buttons and then in the timer do
  // the inside of the loop and then do all the updates
  for (int i = 0; i < 10; i++) {
    // while (!(tuner->selected_pid->pid->finished_autotune())) {
    // tuner->run_cb(); // should include updating PID
    float error = tuner->selected_pid->pid->getError();
    lv_chart_set_next_value(tuner->graph, tuner->ser, error);
    lv_chart_refresh(tuner->graph);
  }
  tuner->frozen = false;
}

void PIDTuner::reset_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  lv_chart_set_all_value(tuner->graph, tuner->ser, LV_CHART_POINT_NONE);
  lv_chart_refresh(tuner->graph);
}

void PIDTuner::select_cb(lv_event_t *event) {
  lv_obj_t *pid_list = lv_event_get_target_obj(event);
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  if (tuner->frozen) {
    return;
  }
  lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  char buf[64];
  lv_dropdown_get_selected_str(pid_list, buf, sizeof(buf));
  // lv_textarea_set_text(tuner->ta_p, ("6" + std::string(buf)).c_str());
  // lv_textarea_set_text(tuner->ta_i, ("7" + std::string(buf)).c_str());
  // lv_textarea_set_text(tuner->ta_d, ("67" + std::string(buf)).c_str());
  for (PID_t pid : tuner->pids) {
    if (pid.name == std::string(buf)) {
      tuner->selected_pid = &pid;
      lv_textarea_set_text(tuner->ta_d, pid.name.c_str());
      lv_dropdown_set_text(pid_list, pid.name.c_str());
      break;
    }
  }
  // update with new pid constants
  lv_textarea_set_text(
      tuner->ta_p, std::to_string(tuner->selected_pid->pid->getP()).c_str());
  lv_textarea_set_text(
      tuner->ta_i, std::to_string(tuner->selected_pid->pid->getI()).c_str());
  lv_textarea_set_text(
      tuner->ta_d, std::to_string(tuner->selected_pid->pid->getD()).c_str());
}

void PIDTuner::numpad_cb(lv_event_t *event) {
  PIDTuner *tuner =
      (PIDTuner *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  // if (lv_event_get_code(event) == LV_EVENT_READY ||
  //     lv_event_get_code(event) == LV_EVENT_CANCEL) {
  //   lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  // }
  if (lv_keyboard_get_selected_button(tuner->numpad) == 3) {
    // keyboard button - hide numpad
    lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
  } else if (lv_keyboard_get_selected_button(tuner->numpad) == 7 &&
             tuner->selected_pid != nullptr) {
    lv_obj_add_flag(tuner->numpad, LV_OBJ_FLAG_HIDDEN);
    // check button - set constants
    float kP = atof(lv_textarea_get_text(tuner->ta_p));
    float kI = atof(lv_textarea_get_text(tuner->ta_i));
    float kD = atof(lv_textarea_get_text(tuner->ta_d));
    tuner->selected_pid->pid->setGains(kP, kI, kD);
  }
  // lv_textarea_set_text(tuner->ta_p,
  // std::to_string(lv_keyboard_get_selected_button(tuner->numpad)).c_str());
}

// ============================== Fonstructor ============================== //

PIDTuner::PIDTuner(std::vector<PID_t> pids, std::function<float()> run_cb)
    : PIDTuner("PID Tuner", pids, run_cb) {}

PIDTuner::PIDTuner(std::string name, std::vector<PID_t> pids,
                   std::function<float()> run_cb) {
  this->name = name;
  this->run_cb = run_cb;
  this->pids = pids;
  selected_pid = nullptr;

  // ----------------------------- Create UI ----------------------------- //

  this->view = rd_view_create(name.c_str());

  lv_obj_set_style_bg_color(view->obj, color_bg, 0);

  pid_cont = lv_obj_create(view->obj);
  lv_obj_add_style(pid_cont, &style_transp, 0);
  lv_obj_set_layout(pid_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(pid_cont, 200, 200);
  lv_obj_align(pid_cont, LV_ALIGN_LEFT_MID, 8, 0);
  lv_obj_set_flex_flow(pid_cont, LV_FLEX_FLOW_COLUMN);
  // lv_obj_set_flex_align(pid_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
  //                       LV_FLEX_ALIGN_CENTER);

  pid_list = lv_dropdown_create(pid_cont);
  lv_dropdown_clear_options(pid_list);
  lv_obj_set_size(pid_list, 160, 40);
  // lv_obj_set_flex_flow(pid_list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(pid_list, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  // lv_obj_align(pid_list, LV_ALIGN_TOP_LEFT, 8, 40);
  for (PID_t pid : pids) {
    lv_dropdown_add_option(pid_list, pid.name.c_str(), LV_DROPDOWN_POS_LAST);
  }

  lv_dropdown_set_text(pid_list, "Select PID");
  lv_dropdown_set_selected_highlight(pid_list, false);
  lv_obj_add_event_cb(pid_list, &select_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_set_user_data(pid_list, this);

  selected_cont = lv_obj_create(view->obj);
  lv_obj_add_style(selected_cont, &style_transp, 0);
  lv_obj_set_layout(selected_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(selected_cont, 240, 240);
  lv_obj_align(selected_cont, LV_ALIGN_CENTER, 120, 0);
  lv_obj_set_flex_align(selected_cont, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_flex_flow(selected_cont, LV_FLEX_FLOW_COLUMN);

  numpad = lv_keyboard_create(view->obj);
  lv_keyboard_set_mode(numpad, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_add_style(numpad, &style_list, 0);
  lv_obj_add_event_cb(numpad, &numpad_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_user_data(numpad, this);
  lv_obj_set_size(numpad, 140, 100);
  lv_obj_add_flag(numpad, LV_OBJ_FLAG_HIDDEN);
  lv_obj_align(numpad, LV_ALIGN_BOTTOM_RIGHT, -8, -8);

  graph_cont = lv_obj_create(view->obj);
  lv_obj_add_style(graph_cont, &style_transp, 0);
  lv_obj_set_layout(graph_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(graph_cont, 280, 220);
  lv_obj_align(graph_cont, LV_ALIGN_CENTER, 90, 0);
  lv_obj_set_flex_flow(graph_cont, LV_FLEX_FLOW_COLUMN);
  // lv_obj_set_flex_align(graph_cont, LV_FLEX_ALIGN_CENTER,
  // LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  lv_obj_t *graph = lv_chart_create(graph_cont);
  lv_obj_set_size(graph, 267, 150);
  lv_chart_set_type(graph, LV_CHART_TYPE_LINE);
  lv_chart_set_update_mode(graph, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(graph, 50);
  lv_obj_set_style_size(graph, 0, 0, LV_PART_INDICATOR);
  lv_chart_series_t *setpoint = lv_chart_add_series(
      graph, LV_COLOR_MAKE(255, 215, 0), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_series_t *value = lv_chart_add_series(
      graph, LV_COLOR_MAKE(34, 139, 34), LV_CHART_AXIS_PRIMARY_Y);

  // Testing Values
  // int current = 0;
  // int target = 0;
  // for (int i = 0; i < 50; i++) {
  //   if (i == 20)
  //     target = 50;
  //
  //   current += 0.4 * (target - current) + lv_rand(0, 5);
  //   lv_chart_set_next_value(graph, setpoint, target);
  //   lv_chart_set_next_value(graph, value, current);
  // }

  btn_cont = lv_obj_create(graph_cont);
  lv_obj_add_style(btn_cont, &style_transp, 0);
  lv_obj_set_layout(btn_cont, LV_LAYOUT_FLEX);
  lv_obj_set_size(btn_cont, 267, 60);
  lv_obj_align(btn_cont, LV_ALIGN_CENTER, 120, 0);
  lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);

  lv_obj_t *test_btn = lv_button_create(btn_cont);
  lv_obj_t *autotune_btn = lv_button_create(btn_cont);
  lv_obj_t *reset_btn = lv_button_create(btn_cont);
  lv_obj_set_size(test_btn, 67, 40);
  lv_obj_set_size(autotune_btn, 67, 40);
  lv_obj_set_size(reset_btn, 67, 40);

  // lv_obj_set_flex_flow(test_btn, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(test_btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  // lv_obj_set_flex_flow(autotune_btn, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(autotune_btn, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  // lv_obj_set_flex_flow(reset_btn, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(reset_btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_add_event_cb(test_btn, &test_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(autotune_btn, &autotune_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(reset_btn, &reset_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_user_data(test_btn, this);
  lv_obj_set_user_data(autotune_btn, this);
  lv_obj_set_user_data(reset_btn, this);

  lv_obj_t *test_label = lv_label_create(test_btn);
  lv_label_set_text(test_label, "Test");
  lv_obj_t *autotune_label = lv_label_create(autotune_btn);
  lv_label_set_text(autotune_label, "Auto\nTune");
  lv_obj_t *reset_label = lv_label_create(reset_btn);
  lv_label_set_text(reset_label, "Reset");

  // lv_obj_set_style_text_opa(test_btn, 128, LV_STATE_PRESSED);
  // lv_obj_set_flex_grow(test_btn, 1);

  // Numpad
  numpad = lv_keyboard_create(view->obj);
  lv_keyboard_set_mode(numpad, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_add_event_cb(numpad, &numpad_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_user_data(numpad, this);
  lv_obj_set_size(numpad, 240, 120);
  lv_obj_align_to(numpad, graph, LV_ALIGN_CENTER, 0, 0);

  // kP,kI,kD fields (textareas)
  ta_p = lv_textarea_create(pid_cont);
  ta_i = lv_textarea_create(pid_cont);
  ta_d = lv_textarea_create(pid_cont);
  lv_obj_set_size(ta_p, 160, 40);
  lv_obj_set_size(ta_i, 160, 40);
  lv_obj_set_size(ta_d, 160, 40);

  // lv_obj_set_flex_flow(ta_p, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(ta_p, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  // lv_obj_set_flex_flow(ta_i, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(ta_i, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  // lv_obj_set_flex_flow(ta_d, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(ta_d, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_add_event_cb(ta_p, &p_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(ta_i, &i_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(ta_d, &d_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_user_data(ta_p, this);
  lv_obj_set_user_data(ta_i, this);
  lv_obj_set_user_data(ta_d, this);

  lv_textarea_set_placeholder_text(ta_p, "0.0");
  lv_textarea_set_placeholder_text(ta_i, "0.0");
  lv_textarea_set_placeholder_text(ta_d, "0.0");
  lv_textarea_set_text(ta_p, "0.0");
  lv_textarea_set_text(ta_i, "0.0");
  lv_textarea_set_text(ta_d, "0.0");

  lv_obj_t *title = lv_label_create(view->obj);
  lv_label_set_text(title, "Tune PID");
  lv_obj_add_style(title, &style_text_large, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 8, 12);

  lv_textarea_set_accepted_chars(ta_p, "+-.1234567890");
  lv_textarea_set_accepted_chars(ta_i, "+-.1234567890");
  lv_textarea_set_accepted_chars(ta_d, "+-.1234567890");

  lv_textarea_set_placeholder_text(ta_p, "0.0");
  lv_textarea_set_placeholder_text(ta_i, "0.0");
  lv_textarea_set_placeholder_text(ta_d, "0.0");

  lv_textarea_set_text(ta_p, "0.0");
  lv_textarea_set_text(ta_i, "0.0");
  lv_textarea_set_text(ta_d, "0.0");
}

void PIDTuner::focus() { rd_view_focus(this->view); }

} // namespace gui
} // namespace libmavnetics
