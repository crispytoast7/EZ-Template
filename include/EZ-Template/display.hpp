#pragma once

namespace ez {

/// Rotates the brain screen in 90-degree steps for brains mounted sideways or
/// upside down. Accepts 0, 90, 180, or 270 (measured clockwise); other values
/// are rejected with a printed message. Call after the screen is initialized
/// (ez::as::initialize or pros::lcd::initialize).
///
/// Experimental: uses LVGL's software rotation on the live display driver.
/// Known limitations: touch coordinates are not rotated with the pixels, so
/// on-screen buttons respond in their pre-rotation positions, and 90/270
/// swap the screen's width and height. The physical LLEMU buttons are
/// unaffected.
void screen_rotation_set(int degrees);

/// Current rotation in degrees (0, 90, 180, or 270).
int screen_rotation_get();

}  // namespace ez
