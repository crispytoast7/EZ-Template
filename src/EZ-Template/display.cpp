#include "EZ-Template/display.hpp"

#include <cstdint>
#include <cstdio>

#include "liblvgl/llemu.hpp"
#include "liblvgl/lvgl.h"
#include "pros/llemu.hpp"

namespace ez {

namespace {
constexpr int LINE_COUNT = 8;

int g_rotation = 0;
bool g_portrait_on = false;
std::string g_line_text[LINE_COUNT];
}  // namespace

#if LVGL_VERSION_MAJOR >= 9

namespace {
// Read out of the vendored liblvgl.a (llemu.c.o, lcd_initialize) so the
// portrait screen is LLEMU's theme at a different size, not a lookalike. LLEMU
// was rewritten against the LVGL 9 API on this kernel, but every color it sets
// is unchanged from the LVGL 8 build; COL_TEXT doubles as the button fill,
// which LLEMU names COL_BTN_NORMAL.
constexpr uint32_t COL_SCREEN = 0x404040;
constexpr uint32_t COL_FRAME = 0x808080;
constexpr uint32_t COL_PANEL_BORDER = 0x606060;
constexpr uint32_t COL_PANEL = 0x5ABC03;
constexpr uint32_t COL_TEXT = 0x202020;
constexpr uint32_t COL_BAR = 0xA0A0A0;
constexpr uint32_t COL_BTN_PRESSED = 0x808080;

// LLEMU's own dimensions: 8 labels on an 18px pitch, each 14px narrower than
// its panel, in a 30px tall button bar of three 80px buttons.
constexpr int32_t LINE_PITCH = 18;
constexpr int32_t PANEL_INSET = 14;
constexpr int32_t BAR_HEIGHT = 30;
constexpr int32_t BTN_WIDTH = 76;
constexpr int32_t MARGIN = 11;

lv_obj_t* g_portrait_screen = nullptr;
lv_obj_t* g_portrait_lines[LINE_COUNT] = {nullptr};
lv_obj_t* g_llemu_screen = nullptr;

lv_obj_t* llemu_button_search(lv_obj_t* parent, int index, int& found) {
  uint32_t children = lv_obj_get_child_count(parent);
  for (uint32_t i = 0; i < children; i++) {
    lv_obj_t* child = lv_obj_get_child(parent, i);
    if (lv_obj_check_type(child, &lv_button_class)) {
      if (found == index) return child;
      found++;
      continue;
    }
    lv_obj_t* match = llemu_button_search(child, index, found);
    if (match != nullptr) return match;
  }
  return nullptr;
}

// LLEMU's three buttons are the only lv_button objects it builds, in btn0/btn1/
// btn2 order. Resolved per click rather than cached because pros::lcd::shutdown
// deletes the tree they live in.
lv_obj_t* llemu_button_get(int index) {
  if (g_llemu_screen == nullptr) return nullptr;
  int found = 0;
  return llemu_button_search(g_llemu_screen, index, found);
}

// Replays the press onto LLEMU's own button so whatever is registered through
// pros::lcd::register_btn*_cb runs with the context it already expects,
// including the center button EZ leaves to the user.
void portrait_button_cb(lv_event_t* event) {
  lv_event_code_t code = lv_event_get_code(event);
  if (code != LV_EVENT_PRESSED && code != LV_EVENT_RELEASED && code != LV_EVENT_CLICKED) return;

  lv_obj_t* target = llemu_button_get((int)(intptr_t)lv_event_get_user_data(event));
  if (target != nullptr) lv_obj_send_event(target, code, nullptr);
}

void portrait_create() {
  int32_t w = lv_display_get_horizontal_resolution(nullptr);
  int32_t h = lv_display_get_vertical_resolution(nullptr);
  int32_t panel_w = w - (MARGIN * 2);
  int32_t panel_h = h - BAR_HEIGHT - (MARGIN * 4);

  g_portrait_screen = lv_obj_create(nullptr);
  lv_obj_set_size(g_portrait_screen, w, h);
  lv_obj_set_style_pad_all(g_portrait_screen, 0, 0);
  lv_obj_set_scrollbar_mode(g_portrait_screen, LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(g_portrait_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(g_portrait_screen, lv_color_hex(COL_SCREEN), 0);

  lv_obj_t* frame = lv_obj_create(g_portrait_screen);
  lv_obj_set_size(frame, w, h);
  lv_obj_set_style_pad_all(frame, 0, 0);
  lv_obj_remove_flag(frame, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(frame, lv_color_hex(COL_FRAME), 0);
  lv_obj_set_style_border_color(frame, lv_color_hex(COL_FRAME), 0);

  lv_obj_t* panel = lv_obj_create(frame);
  lv_obj_set_size(panel, panel_w, panel_h);
  lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, MARGIN);
  lv_obj_set_style_pad_all(panel, 2, 0);
  lv_obj_set_style_pad_left(panel, 7, 0);
  lv_obj_set_style_pad_row(panel, 0, 0);
  lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_border_color(panel, lv_color_hex(COL_PANEL_BORDER), 0);
  lv_obj_set_style_bg_color(panel, lv_color_hex(COL_PANEL), 0);
  lv_obj_set_style_text_color(panel, lv_color_hex(COL_TEXT), 0);
  lv_obj_set_style_text_font(panel, &pros_font_dejavu_mono_18, 0);
  // LLEMU pins each label to a fixed y; the wrapped lines have to push the ones
  // under them down instead, which is what the extra height is for.
  lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);

  for (int i = 0; i < LINE_COUNT; i++) {
    lv_obj_t* label = lv_label_create(panel);
    lv_obj_set_width(label, panel_w - PANEL_INSET);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    // Empty labels collapse to nothing under flex, so hold LLEMU's pitch.
    lv_obj_set_style_min_height(label, LINE_PITCH, 0);
    lv_label_set_text(label, "");
    g_portrait_lines[i] = label;
  }

  lv_obj_t* bar = lv_obj_create(frame);
  lv_obj_set_size(bar, panel_w, BAR_HEIGHT);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -MARGIN);
  lv_obj_set_style_bg_color(bar, lv_color_hex(COL_BAR), 0);
  lv_obj_set_style_border_color(bar, lv_color_hex(COL_FRAME), 0);
  lv_obj_set_style_border_width(bar, 0, 0);
  lv_obj_set_style_pad_all(bar, 0, 0);
  lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_AROUND);
  lv_obj_remove_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  for (int i = 0; i < 3; i++) {
    lv_obj_t* btn = lv_button_create(bar);
    lv_obj_set_width(btn, BTN_WIDTH);
    lv_obj_add_event_cb(btn, portrait_button_cb, LV_EVENT_ALL, (void*)(intptr_t)i);
    lv_obj_set_style_bg_color(btn, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(COL_BTN_PRESSED), LV_STATE_PRESSED);
  }
}

// Screens are built and loaded from whatever task calls screen_rotation_set,
// while PROS's disp_daemon is already inside lv_timer_handler; the load-time
// full-screen repaint races that flush and lands only partly on a rotated
// display. Repainting once more from a timer puts the redraw on the daemon's
// own task, where nothing is in flight.
void repaint_schedule() {
  lv_timer_t* timer = lv_timer_create([](lv_timer_t* t) { lv_obj_invalidate(lv_screen_active()); }, 60, nullptr);
  lv_timer_set_repeat_count(timer, 1);
}

void portrait_load() {
  if (g_portrait_screen == nullptr) portrait_create();

  if (!g_portrait_on) {
    g_llemu_screen = lv_screen_active();
    g_portrait_on = true;
  }

  for (int i = 0; i < LINE_COUNT; i++)
    lv_label_set_text(g_portrait_lines[i], g_line_text[i].c_str());

  lv_screen_load(g_portrait_screen);
  repaint_schedule();
}

void portrait_unload() {
  if (!g_portrait_on) return;

  g_portrait_on = false;
  // The portrait screen is kept, not deleted, so repeated calls reuse it.
  if (g_llemu_screen != nullptr) lv_screen_load(g_llemu_screen);
  repaint_schedule();
}

void portrait_line_set(int line, const char* text) { lv_label_set_text(g_portrait_lines[line], text); }
}  // namespace

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

  // Built after the rotation so the screen picks up the swapped resolution.
  if (degrees == 90 || degrees == 270)
    portrait_load();
  else
    portrait_unload();

  printf("[display] Screen rotation set to %d degrees.\n", degrees);
}

#else  // LVGL 8

namespace {
// The portrait screen is built against the LVGL 9 API, so on LVGL 8 every
// rotation keeps stock LLEMU and the print lines route to it unchanged.
void portrait_load() {}
void portrait_unload() {}
void portrait_line_set(int line, const char* text) {}
}  // namespace

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

  if (degrees == 90 || degrees == 270)
    portrait_load();
  else
    portrait_unload();

  printf("[display] Screen rotation set to %d degrees.\n", degrees);
}

#endif

int screen_rotation_get() { return g_rotation; }

bool screen_portrait_enabled() { return g_portrait_on; }

void screen_line_set(int line, std::string text) {
  if (line >= 0 && line < LINE_COUNT) g_line_text[line] = text;

  if (g_portrait_on) {
    if (line >= 0 && line < LINE_COUNT) portrait_line_set(line, text.c_str());
    return;
  }
  pros::lcd::set_text(line, text);
}

void screen_line_clear(int line) {
  if (line >= 0 && line < LINE_COUNT) g_line_text[line].clear();

  if (g_portrait_on) {
    if (line >= 0 && line < LINE_COUNT) portrait_line_set(line, "");
    return;
  }
  pros::lcd::clear_line(line);
}

void screen_lines_clear() {
  for (int i = 0; i < LINE_COUNT; i++) g_line_text[i].clear();

  if (g_portrait_on) {
    for (int i = 0; i < LINE_COUNT; i++) portrait_line_set(i, "");
    return;
  }
  pros::lcd::clear();
}

}  // namespace ez
