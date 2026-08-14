#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/misc.hpp"

namespace ez {
namespace health {

struct Report {
  bool imu_ok = true;
  int motors_bad = 0;        ///< drive motors not responding
  int trackers_bad = 0;      ///< configured odom trackers not responding
  int distance_bad = 0;      ///< registered DSR sensors not responding
  bool all_ok() const {
    return imu_ok && motors_bad == 0 && trackers_bad == 0 && distance_bad == 0;
  }
};

/// Checks that the IMU, every drive motor, every configured odom tracker, and
/// every DSR sensor responds. Prints each failure with its port and rumbles
/// the controller when anything is wrong. Safe to call from initialize() and
/// again at the start of autonomous.
Report preflight(ez::Drive& chassis, pros::Controller& controller);

}  // namespace health
}  // namespace ez
