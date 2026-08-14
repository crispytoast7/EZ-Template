#pragma once

namespace ez {

/// Rotates the brain screen 180 degrees, for brains mounted upside down.
/// Call after the screen is initialized (ez::as::initialize or
/// pros::lcd::initialize).
///
/// Experimental: uses LVGL's software rotation on the live display driver.
/// Known limitation: touch coordinates are not rotated with the pixels, so
/// on-screen buttons respond mirrored (left and right swap). The physical
/// LLEMU buttons are unaffected.
void screen_flip_set(bool flipped);

/// Current flip state.
bool screen_flipped();

}  // namespace ez
