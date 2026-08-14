#include "EZ-Template/display.hpp"

#include "liblvgl/lvgl.h"
#include <cstdio>

namespace ez {

namespace {
bool g_flipped = false;
}

void screen_flip_set(bool flipped) {
  lv_disp_t* disp = lv_disp_get_default();
  if (disp == nullptr || disp->driver == nullptr) {
    printf("[display] No LVGL display yet; initialize the screen first.\n");
    return;
  }
  disp->driver->sw_rotate = 1;
  lv_disp_set_rotation(disp, flipped ? LV_DISP_ROT_180 : LV_DISP_ROT_NONE);
  g_flipped = flipped;
  printf("[display] Screen rotation %s.\n", flipped ? "180" : "normal");
}

bool screen_flipped() { return g_flipped; }

}  // namespace ez
