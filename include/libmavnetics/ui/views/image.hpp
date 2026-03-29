#pragma once

#include "libmavnetics/ui/api.h"

namespace libmavnetics {
namespace gui {

class ImageView {
public:
  ImageView(std::string name, const lv_image_dsc_t *image_src);

  void focus();

private:
  rd_view_t *view;
  std::string name;
};

} // namespace gui
} // namespace libmavnetics
