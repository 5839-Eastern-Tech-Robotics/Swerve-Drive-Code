#pragma once

#include <cstddef>
#include <queue>
#include <string>
#include <vector>

#include "liblvgl/misc/lv_types.h"
#include "libmavnetics/ui/apix.h"
#include "pros/rtos.hpp"

namespace libmavnetics {
namespace gui {

class DebugConsole {
public:
  DebugConsole();
  DebugConsole(std::string name);

  void print(size_t line, std::string str);
  size_t print(std::string str);

  template <typename... Args>
  void printf(int line, std::string format, Args... args) {
    auto size =
        std::snprintf(nullptr, 0, format.c_str(), std::forward<Args>(args)...);
    std::string output(size + 1, '\0');
    std::sprintf(&output[0], format.c_str(), std::forward<Args>(args)...);
    print(line, output);
  }

  template <typename... Args> size_t printf(std::string format, Args... args) {
    auto size =
        std::snprintf(nullptr, 0, format.c_str(), std::forward<Args>(args)...);
    std::string output(size + 1, '\0');
    std::sprintf(&output[0], format.c_str(), std::forward<Args>(args)...);
    return print(output);
  }

  void clear(size_t line);
  void clear();

  void focus();

private:
  enum action_type_e { PRINT, CLEAR };
  struct console_action_t {
    action_type_e action_type;
    int line_number;
    std::string str;
  };

  rd_view_t *view;
  lv_obj_t *line_list;

  std::string name;
  std::vector<lv_obj_t *> lines;
  std::queue<console_action_t> action_queue;
  pros::Mutex queue_mut{};

  void add_blank_line();

  static void update_console(lv_timer_t *timer);
  static void clear_screen_cb(lv_event_t *event);
  static void up_cb(lv_event_t *event);
  static void down_cb(lv_event_t *event);
};

} // namespace gui
} // namespace libmavnetics
