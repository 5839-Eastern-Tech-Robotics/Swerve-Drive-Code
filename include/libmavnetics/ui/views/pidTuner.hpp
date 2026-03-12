#pragma once

#include "libmavnetics/ui/apix.h"
#include "libmavnetics/utils/pid.hpp"
#include <functional>
#include <string>
#include <vector>

namespace libmavnetics {
namespace gui {

class PIDTuner {
  /// @addtogroup selector
  /// @{
public:
  /// @name Selector Typedefs
  typedef std::function<void()> routine_action_t;

  typedef struct PID_s {
    std::string name;
    PID *pid;
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
  PIDTuner(std::string name, std::vector<PID_t> pids,
           std::function<float()> run_cb);

  /**
   * @brief Create PID tuner
   * @param pids Vector of pid names/objects TODO decide
   * @param run_cb Callback to run for testing
   */
  PIDTuner(std::vector<PID_t> pids, std::function<float()> run_cb);

  /**
   * @brief Set this view to the active view
   */
  void focus();

  /// @}

private:
  rd_view_t *view;

  lv_obj_t *pid_cont;
  lv_obj_t *pid_list;
  lv_obj_t *selected_cont;
  lv_obj_t *selected_label;
  lv_obj_t *selected_img;

  lv_obj_t *numpad;
  lv_obj_t *ta_p;
  lv_obj_t *ta_i;
  lv_obj_t *ta_d;

  lv_obj_t *btn_cont;
  lv_obj_t *graph_cont;
  lv_obj_t *graph;
  lv_chart_series_t *ser;

  std::string name;
  std::vector<PID_t> pids;
  std::function<float()> run_cb;
  // std::vector<select_action_t> select_callbacks;
  PID_t *selected_pid;

  // void sd_save();
  // void sd_load();

  // void run_callbacks();

  static void p_cb(lv_event_t *event);
  static void i_cb(lv_event_t *event);
  static void d_cb(lv_event_t *event);
  static void test_cb(lv_event_t *event);
  static void autotune_cb(lv_event_t *event);
  static void reset_cb(lv_event_t *event);
  static void select_cb(lv_event_t *event);
  static void numpad_cb(lv_event_t *event);

  bool frozen = false;
};

} // namespace gui
} // namespace libmavnetics
