#include "EZ-Template/display.hpp"

#include "liblvgl/lvgl.h"
#include <cstdio>

namespace ez {

namespace {
int g_rotation = 0;
}

void screen_rotation_set(int degrees) {
  lv_disp_rot_t rot;
  switch (degrees) {
    case 0:   rot = LV_DISP_ROT_NONE; break;
    case 90:  rot = LV_DISP_ROT_90; break;
    case 180: rot = LV_DISP_ROT_180; break;
    case 270: rot = LV_DISP_ROT_270; break;
    default:
      printf("[display] Invalid rotation %d; use 0, 90, 180, or 270.\n", degrees);
      return;
  }

  lv_disp_t* disp = lv_disp_get_default();
  if (disp == nullptr || disp->driver == nullptr) {
    printf("[display] No LVGL display yet; initialize the screen first.\n");
    return;
  }
  disp->driver->sw_rotate = 1;
  lv_disp_set_rotation(disp, rot);
  g_rotation = degrees;
  printf("[display] Screen rotation set to %d degrees.\n", degrees);
}

int screen_rotation_get() { return g_rotation; }

}  // namespace ez
