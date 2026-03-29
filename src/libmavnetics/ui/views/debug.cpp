#include "libmavnetics/ui/views/debug.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_event.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/core/lv_obj_tree.h"
#include "liblvgl/misc/lv_anim.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/widgets/list/lv_list.h"
#include "libmavnetics/ui/core.h"
#include "libmavnetics/ui/utils.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <mutex>
#include <string>

namespace libmavnetics {
namespace gui {

void DebugConsole::update_console(lv_timer_t* timer) {
  DebugConsole* console = (DebugConsole*) lv_timer_get_user_data(timer);

  while (!console->action_queue.empty()) {
    console_action_t action = console->action_queue.front();
    console->action_queue.pop();

    if (action.action_type == action_type_e::PRINT) {
      if (action.line_number == -1) { // print at the end
        console->add_blank_line();
        lv_label_set_text(console->lines.back(), action.str.c_str());
      } else { // print on a specific line
        while (action.line_number >= console->lines.size()) {
          console->add_blank_line();
        }

        lv_label_set_text(console->lines.at(action.line_number), action.str.c_str());
      }
    } else { // action_type_e::CLEAR
      if (action.line_number == -1) { // clear all lines
        for (lv_obj_t *line : console->lines) {
          lv_obj_delete(line);
        }
        console->lines.clear();
      } else { // clear a specific line
        if (action.line_number >= console->lines.size())
          return;

        if (action.line_number == console->lines.size() - 1) {
          lv_obj_delete(console->lines.back());
          console->lines.pop_back();
        } else {
          lv_label_set_text(console->lines.at(action.line_number), "");
        }
      }
    }
  }
}

void DebugConsole::clear_screen_cb(lv_event_t *event) {
  DebugConsole *console = (DebugConsole *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  console->clear();
}

void DebugConsole::up_cb(lv_event_t *event) {
  DebugConsole *console = (DebugConsole *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_coord_t scroll_y = lv_obj_get_height(console->line_list) / 2;
  lv_obj_scroll_by_bounded(console->line_list, 0, scroll_y, LV_ANIM_ON);
}

void DebugConsole::down_cb(lv_event_t *event) {
  DebugConsole *console = (DebugConsole *)lv_obj_get_user_data(lv_event_get_target_obj(event));
  lv_coord_t scroll_y = lv_obj_get_height(console->line_list) / -2;
  lv_obj_scroll_by_bounded(console->line_list, 0, scroll_y, LV_ANIM_ON);
}

void DebugConsole::add_blank_line() {
  lv_obj_t *new_line = lv_list_add_text(line_list, "");
  lv_obj_add_style(new_line, &style_text_mono, 0);
  lv_obj_set_style_bg_color(new_line, LV_COLOR_MAKE(13, 21, 26), 0);
  lv_obj_set_style_pad_all(new_line, 5, 0);
  lv_obj_scroll_to_view(new_line, LV_ANIM_OFF);
  lines.push_back(new_line);
}

void DebugConsole::print(size_t line, std::string str) {
  std::lock_guard<pros::Mutex> lock{queue_mut};
  action_queue.push({action_type_e::PRINT, (int) line, str});
}

size_t DebugConsole::print(std::string str) {
  std::lock_guard<pros::Mutex> lock{queue_mut};
  action_queue.push({action_type_e::PRINT, -1, str});
  return 0;
}

void DebugConsole::clear() {
  std::lock_guard<pros::Mutex> lock{queue_mut};
  action_queue.push({action_type_e::CLEAR, -1, ""});
}

void DebugConsole::clear(size_t line) {
  std::lock_guard<pros::Mutex> lock{queue_mut};
  action_queue.push({action_type_e::CLEAR, (int) line, ""});
}

DebugConsole::DebugConsole() : DebugConsole("Debug Console") {}

DebugConsole::DebugConsole(std::string name) {
  this->name = name;
  this->view = rd_view_create(name.c_str());

  lv_obj_set_style_bg_color(view->obj, color_bg, 0);
  lv_obj_remove_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);

  this->line_list = lv_list_create(view->obj);
  lv_obj_set_size(line_list, 422, 192);
  lv_obj_align(line_list, LV_ALIGN_TOP_LEFT, 8, 40);
  lv_obj_add_style(line_list, &style_list, 0);

  lv_obj_t *list_btns = lv_obj_create(view->obj);
  lv_obj_add_style(list_btns, &style_transp, 0);
  lv_obj_set_size(list_btns, 32, 192);
  lv_obj_align(list_btns, LV_ALIGN_TOP_RIGHT, -8, 40);
  lv_obj_remove_flag(list_btns, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(list_btns, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(list_btns, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(list_btns, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);

  lv_obj_t *up_btn = lv_button_create(list_btns);
  lv_obj_add_style(up_btn, &style_transp, 0);
  lv_obj_set_size(up_btn, 32, 32);
  lv_obj_add_event_cb(up_btn, &up_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_user_data(up_btn, this);
  lv_obj_set_style_text_opa(up_btn, 128, LV_STATE_PRESSED);
  lv_obj_set_flex_grow(up_btn, 1);

  lv_obj_t *up_img = lv_image_create(up_btn);
  lv_obj_align(up_img, LV_ALIGN_CENTER, 0, 0);
  lv_image_set_src(up_img, LV_SYMBOL_UP);

  lv_obj_t *down_btn = lv_button_create(list_btns);
  lv_obj_add_style(down_btn, &style_transp, 0);
  lv_obj_set_size(down_btn, 32, 32);
  lv_obj_add_event_cb(down_btn, &down_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_user_data(down_btn, this);
  lv_obj_set_style_text_opa(down_btn, 128, LV_STATE_PRESSED);
  lv_obj_set_flex_grow(down_btn, 1);

  lv_obj_t *down_img = lv_image_create(down_btn);
  lv_obj_align(down_img, LV_ALIGN_CENTER, 0, 0);
  lv_image_set_src(down_img, LV_SYMBOL_DOWN);

  lv_obj_t *clear_btn = lv_button_create(view->obj);
  lv_obj_add_style(clear_btn, &style_core_button, 0);
  lv_obj_set_size(clear_btn, 50, 32);
  lv_obj_add_event_cb(clear_btn, &clear_screen_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_user_data(clear_btn, this);
  lv_obj_add_style(clear_btn, &style_core_button_pr, LV_STATE_PRESSED);
  lv_obj_align(clear_btn, LV_ALIGN_TOP_LEFT, 8, 4);

  lv_obj_t *clear_txt = lv_label_create(clear_btn);
  lv_label_set_text(clear_txt, "clear");
  lv_obj_add_style(clear_txt, &style_text_medium, 0);
  lv_obj_align(clear_txt, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t *title = lv_label_create(view->obj);
  lv_label_set_text(title, "Console");
  lv_obj_add_style(title, &style_text_large, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  lv_timer_create(update_console, 50, this);
}

void DebugConsole::focus() { rd_view_focus(this->view); }

} // namespace gui
} // namespace libmavnetics
