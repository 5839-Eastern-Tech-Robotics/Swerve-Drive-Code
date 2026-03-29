#pragma once

#include "liblvgl/misc/lv_types.h"
#include "libmavnetics/ui/apix.h"
#include "libmavnetics/utils/pid.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace libmavnetics {
namespace gui {

class PIDTuner {
public:
  typedef std::function<void()> routine_action_t;

  typedef struct PID_s {
    std::string name;
    PID *pid;
    routine_action_t update;
    std::function<double()> getError = nullptr;
  } PID_t;
  // std::vector<PID_t> *test = nullptr;
  // typedef std::function<void(std::optional<routine_t>)> select_action_t;

  /// @name Selector Functions

  /**
   * @brief Create PID tuner
   * @param name Name of the PID tuner
   * @param pids Vector of pid names/objects TODO
   * @param run_cb Callback to run for testing
   */
  PIDTuner(std::string name, std::vector<PID_t> pids);

  /**
   * @brief Create PID tuner
   * @param pids Vector of pid names/objects TODO decide
   * @param run_cb Callback to run for testing
   */
  PIDTuner(std::vector<PID_t> pids);

  /**
   * @brief select the pid
   * @param index the index of the pid, as ordered by the vector when
   * constructed
   */
  void selectPID(int index);

  /**
   * @brief Set this view to the active view
   */
  void focus();

  /// @}

private:
  rd_view_t *view;

  lv_obj_t *pid_cont;
  lv_obj_t *pid_dropdown;

  lv_obj_t *numpad;
  lv_obj_t *ta_p;
  lv_obj_t *ta_i;
  lv_obj_t *ta_d;

  lv_obj_t *graph;
  lv_obj_t *scale;
  lv_chart_series_t *error_series;

  lv_obj_t *scale_label;
  lv_obj_t *test_label;

  std::vector<PID_t> pids;
  std::string name;
  std::array<double, 50> error_values{};
  std::array<std::int32_t, 50> scaled_error_values{};
  PID_t *selected_pid;
  bool running_test = false;

  static void p_selected_cb(lv_event_t *event);
  static void i_selected_cb(lv_event_t *event);
  static void d_selected_cb(lv_event_t *event);
  static void run_test_cb(lv_event_t *event);
  static void reset_cb(lv_event_t *event);
  static void select_cb(lv_event_t *event);
  static void numpad_cb(lv_event_t *event);
  static void update_graph(lv_timer_t *timer);

};

} // namespace gui
} // namespace libmavnetics
