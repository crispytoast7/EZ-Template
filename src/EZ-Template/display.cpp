#include "EZ-Template/display.hpp"

#include "liblvgl/lvgl.h"
#include <cstdio>

namespace ez {

namespace {
int g_rotation = 0;
}

#if LVGL_VERSION_MAJOR >= 9

void screen_rotation_set(int degrees) {
  lv_display_rotation_t rot;
  switch (degrees) {
    case 0:   rot = LV_DISPLAY_ROTATION_0; break;
    case 90:  rot = LV_DISPLAY_ROTATION_90; break;
    case 180: rot = LV_DISPLAY_ROTATION_180; break;
    case 270: rot = LV_DISPLAY_ROTATION_270; break;
    default:
      printf("[display] Invalid rotation %d; use 0, 90, 180, or 270.\n", degrees);
      return;
  }

  lv_display_t* disp = lv_display_get_default();
  if (disp == nullptr) {
    printf("[display] No LVGL display yet; initialize the screen first.\n");
    return;
  }
  lv_display_set_rotation(disp, rot);
  g_rotation = degrees;
  printf("[display] Screen rotation set to %d degrees.\n", degrees);
}

#else  // LVGL 8

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
  // No indev hook here: LVGL 8.3.4 already rotates pointer input off
  // disp->driver->rotated (lv_indev.c:347-354, inlined into
  // lv_indev_read_timer_cb in the vendored liblvgl.a). Wrapping read_cb with
  // ez::screen_touch_rotate would transform the point a second time.
  disp->driver->sw_rotate = 1;
  lv_disp_set_rotation(disp, rot);
  g_rotation = degrees;
  printf("[display] Screen rotation set to %d degrees.\n", degrees);
}

#endif

int screen_rotation_get() { return g_rotation; }

}  // namespace ez
