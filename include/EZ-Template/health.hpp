#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/misc.hpp"

#if __has_include("hardware/Device.hpp")
#define EZ_HEALTH_LEMLIB_HARDWARE 1
#include "hardware/Device.hpp"
#endif

namespace ez {
namespace health {

struct Report {
  bool imu_ok = true;
  int motors_bad = 0;        ///< drive motors not responding
  int trackers_bad = 0;      ///< configured odom trackers not responding
  int distance_bad = 0;      ///< registered DSR sensors not responding
  int lemlib_bad = 0;        ///< registered lemlib devices not connected
  bool all_ok() const {
    return imu_ok && motors_bad == 0 && trackers_bad == 0 && distance_bad == 0 &&
           lemlib_bad == 0;
  }
};

/// Checks that the IMU, every drive motor, every configured odom tracker, and
/// every DSR sensor responds. Prints each failure with its port and rumbles
/// the controller when anything is wrong. Safe to call from initialize() and
/// again at the start of autonomous.
Report preflight(ez::Drive& chassis, pros::Controller& controller);

#ifdef EZ_HEALTH_LEMLIB_HARDWARE
/// Registers a LemLib hardware device (Motor, MotorGroup, encoder, IMU, or
/// anything implementing lemlib::Device) for inclusion in preflight checks.
void device_add(lemlib::Device* device, const char* name);
#endif

}  // namespace health
}  // namespace ez
