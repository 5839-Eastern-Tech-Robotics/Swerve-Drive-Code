#include "libmavnetics/ui/views/image.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "libmavnetics/ui/core.h"
#include <cstdint>
#include <string>

namespace libmavnetics {
namespace gui {

ImageView::ImageView(std::string name, const void* image_src) {
  this->name = name;
  this->view = rd_view_create(name.c_str());

  lv_obj_set_style_bg_color(view->obj, color_bg, 0);
  lv_obj_remove_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* image = lv_image_create(view->obj);
  lv_obj_set_size(image, 480, 240);
  lv_image_set_src(image, image_src);
  lv_image_set_inner_align(image, lv_image_align_t::LV_IMAGE_ALIGN_CENTER);

  const lv_image_dsc_t* src = lv_image_get_bitmap_map_src(image);
  uint32_t src_h = src->header.h;
  uint32_t src_w = src->header.w;
  double zoom;
  if (2 * src_h > src_w) {
    zoom = 240.0 / src_h;
  } else {
    zoom = 480.0 / src_w;
  }
  lv_image_set_scale(image, (int)(zoom * 256));
}

}
}
