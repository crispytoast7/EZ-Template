#include "EZ-Template/health.hpp"

#include "EZ-Template/dsr.hpp"
#include "pros/error.h"
#include <cstdio>

namespace ez {
namespace health {

Report preflight(ez::Drive& chassis, pros::Controller& controller) {
  Report r;

  if (!chassis.imu.is_installed()) {
    r.imu_ok = false;
    printf("[health] IMU on port %d not responding\n", chassis.imu.get_port());
  }

  auto check_motors = [&](std::vector<pros::Motor>& motors) {
    for (auto& m : motors) {
      if (m.get_temperature() == PROS_ERR_F) {
        r.motors_bad++;
        printf("[health] Drive motor on port %d not responding\n", m.get_port());
      }
    }
  };
  check_motors(chassis.left_motors);
  check_motors(chassis.right_motors);

  auto check_tracker = [&](ez::tracking_wheel* t, const char* name) {
    if (t == nullptr) return;
    if (t->get_raw() == PROS_ERR_F || t->get_raw() == PROS_ERR) {
      r.trackers_bad++;
      printf("[health] %s tracker not responding\n", name);
    }
  };
  check_tracker(chassis.odom_tracker_left, "left");
  check_tracker(chassis.odom_tracker_right, "right");
  check_tracker(chassis.odom_tracker_front, "front");
  check_tracker(chassis.odom_tracker_back, "back");

  for (const auto& s : dsr::sensors()) {
    if (s.device->get() == PROS_ERR) {
      r.distance_bad++;
      printf("[health] DSR sensor on port %d not responding\n", s.device->get_port());
    }
  }

  if (!r.all_ok()) {
    controller.rumble("---");
    printf("[health] PREFLIGHT FAILED: imu %s, %d motor(s), %d tracker(s), %d distance sensor(s)\n",
           r.imu_ok ? "ok" : "BAD", r.motors_bad, r.trackers_bad, r.distance_bad);
  } else {
    printf("[health] Preflight OK.\n");
  }
  return r;
}

}  // namespace health
}  // namespace ez
