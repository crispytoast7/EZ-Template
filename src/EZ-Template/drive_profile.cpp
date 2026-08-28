#include "EZ-Template/drive_profile.hpp"

#include "EZ-Template/auton.hpp"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/tuner.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

namespace ez {

namespace {

DriveProfilePair g_profile;
bool g_enabled = false;
bool g_battery_enabled = false;

// Command levels stepped through per side: 10 through 127, evenly spaced.
constexpr int LEVELS = 10;
constexpr int LEVEL_START = 10;
constexpr int LEVEL_STEP = 13;

constexpr int SAMPLE_MS = 20;
constexpr int SETTLE_MS = 500;    // how long the velocity has to hold still
constexpr int MAX_HOLD_MS = 1500;  // give up waiting and take what we have
constexpr int AVERAGE_MS = 150;    // steady-state window once settled
constexpr int REST_MS = 300;       // between levels, so nothing is measured coasting
constexpr double SETTLE_RPM = 4.0;  // velocity is "still" inside this band

}  // namespace

DriveProfilePair& drive_profile() { return g_profile; }

void drive_profile_enable(bool enable) {
  g_enabled = enable;
  if (enable && !g_profile.valid())
    printf("[profile] Compensation enabled but no profile is loaded; drive is unchanged.\n");
}
bool drive_profile_enabled() { return g_enabled; }

void drive_profile_battery_enable(bool enable) { g_battery_enabled = enable; }
bool drive_profile_battery_enabled() { return g_battery_enabled; }

/////
//
// Math
//
/////

double drive_profile_deadband_map(double cmd, double ks) {
  if (cmd == 0.0) return 0.0;
  if (ks <= 0.0 || ks >= 127.0) return cmd;

  double sign = cmd < 0.0 ? -1.0 : 1.0;
  double mag = std::fabs(cmd);
  if (mag > 127.0) mag = 127.0;
  return sign * (ks + mag * (127.0 - ks) / 127.0);
}

double drive_profile_balance_scale(double kv_this, double kv_other) {
  if (kv_this <= 0.0 || kv_other <= 0.0) return 1.0;
  double ratio = kv_this / kv_other;
  return ratio < 1.0 ? ratio : 1.0;
}

double drive_profile_battery_scale(double battery_mv) {
  if (battery_mv <= 0.0) return 1.0;
  double scale = 12500.0 / battery_mv;
  if (scale > 1.25) return 1.25;
  if (scale < 0.85) return 0.85;
  return scale;
}

int drive_profile_apply(double cmd, const DriveProfile& side, double balance_scale,
                        double battery_scale) {
  if (cmd == 0.0) return 0;

  // Balance first: it scales how much motion is being asked for, and the
  // deadband map is what turns that ask into a command that actually moves.
  double out = drive_profile_deadband_map(cmd * balance_scale, side.ks) * battery_scale;
  if (out > 127.0) out = 127.0;
  if (out < -127.0) out = -127.0;
  return (int)std::lround(out);
}

void drive_profile_transform(int& left, int& right) {
  if (!g_enabled || !g_profile.valid()) return;

  double battery = g_battery_enabled ? drive_profile_battery_scale(pros::battery::get_voltage()) : 1.0;
  double left_balance = drive_profile_balance_scale(g_profile.left.kv, g_profile.right.kv);
  double right_balance = drive_profile_balance_scale(g_profile.right.kv, g_profile.left.kv);

  left = drive_profile_apply(left, g_profile.left, left_balance, battery);
  right = drive_profile_apply(right, g_profile.right, right_balance, battery);
}

DriveProfileFit drive_profile_fit(const double* commands, const double* rpms, int count,
                                  double moving_rpm) {
  DriveProfileFit fit;
  if (commands == nullptr || rpms == nullptr || count <= 0) return fit;

  double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
  double lowest_moving = 0.0;
  int n = 0;

  for (int i = 0; i < count; i++) {
    double rpm = std::fabs(rpms[i]);
    double cmd = std::fabs(commands[i]);
    if (rpm < moving_rpm) continue;

    if (n == 0 || cmd < lowest_moving) lowest_moving = cmd;
    sum_x += rpm;
    sum_y += cmd;
    sum_xx += rpm * rpm;
    sum_xy += rpm * cmd;
    n++;
  }

  fit.points = n;
  if (n == 0) return fit;  // nothing moved: no profile, compensation stays off

  if (n == 1) {
    // No slope to fit. kv is only ever read as a ratio against the other side,
    // so the crude command / rpm is still the right shape of number.
    fit.kv = sum_y / sum_x;
    fit.ks = lowest_moving;
    fit.r2 = 0.0;
    fit.ok = true;
  } else {
    double denom = n * sum_xx - sum_x * sum_x;
    double slope = 0.0, intercept = 0.0;
    if (std::fabs(denom) > 1e-9) {
      slope = (n * sum_xy - sum_x * sum_y) / denom;
      intercept = (sum_y - slope * sum_x) / n;
    }

    // A flat or falling slope means the samples were noise, not a line; fall
    // back to the same ratio the single-point case uses.
    if (slope <= 0.0) {
      slope = sum_y / sum_x;
      intercept = 0.0;
      fit.r2 = 0.0;
    } else {
      double mean_y = sum_y / n;
      double ss_tot = 0.0, ss_res = 0.0;
      for (int i = 0; i < count; i++) {
        double rpm = std::fabs(rpms[i]);
        double cmd = std::fabs(commands[i]);
        if (rpm < moving_rpm) continue;
        double predicted = intercept + slope * rpm;
        ss_res += (cmd - predicted) * (cmd - predicted);
        ss_tot += (cmd - mean_y) * (cmd - mean_y);
      }
      fit.r2 = ss_tot > 1e-9 ? 1.0 - ss_res / ss_tot : 0.0;
    }

    fit.kv = slope;
    // Higher of the two estimates: erring high means the drive pushes a little
    // harder than it strictly has to, erring low means it doesn't move at all.
    fit.ks = intercept > lowest_moving ? intercept : lowest_moving;
    fit.ok = true;
  }

  if (fit.ks < 0.0) fit.ks = 0.0;
  if (fit.ks > 126.0) fit.ks = 126.0;
  if (fit.kv <= 0.0) fit.ok = false;
  return fit;
}

/////
//
// Characterization
//
/////

namespace {

// Motor velocity averaged over the side's motors, skipping anything in the pto
// list or not answering. With the wheels in the air this is the only thing on
// the robot that turns, which is why the drive encoders aren't used here.
double side_velocity(ez::Drive& chassis, std::vector<pros::Motor>& motors) {
  double sum = 0.0;
  int n = 0;
  for (auto& m : motors) {
    if (chassis.pto_check(m)) continue;
    double v = m.get_actual_velocity();
    if (v == PROS_ERR_F) continue;
    sum += v;
    n++;
  }
  return n > 0 ? sum / n : 0.0;
}

// Same guard the PID tuner runs between trials: a hot motor's velocity sags,
// which would read as a slower side that isn't.
double motor_max_temp(ez::Drive& chassis) {
  double max_t = 0.0;
  for (auto& m : chassis.left_motors) {
    double t = m.get_temperature();
    if (t > 0.0 && t < 150.0) max_t = std::max(max_t, t);
  }
  for (auto& m : chassis.right_motors) {
    double t = m.get_temperature();
    if (t > 0.0 && t < 150.0) max_t = std::max(max_t, t);
  }
  return max_t;
}

void wait_for_temp(ez::Drive& chassis, double max_c) {
  while (true) {
    double t = motor_max_temp(chassis);
    if (t <= max_c) return;
    printf("[profile] Motor at %.0f C, waiting for <= %.0f C\n", t, max_c);
    pros::delay(5000);
  }
}

void side_command(ez::Drive& chassis, bool left_side, int cmd) {
  if (left_side)
    chassis.drive_set(cmd, 0);
  else
    chassis.drive_set(0, cmd);
}

// Holds one command level until the velocity stops changing, then averages the
// steady state. Returns the measured rpm and fills battery_mv.
double hold_and_measure(ez::Drive& chassis, bool left_side, int cmd, double& battery_mv) {
  std::vector<pros::Motor>& motors = left_side ? chassis.left_motors : chassis.right_motors;

  side_command(chassis, left_side, cmd);

  uint32_t start = pros::millis();
  uint32_t stable_since = start;
  double reference = side_velocity(chassis, motors);

  while (pros::millis() - start < (uint32_t)MAX_HOLD_MS) {
    pros::delay(SAMPLE_MS);
    double v = side_velocity(chassis, motors);
    if (std::fabs(v - reference) > SETTLE_RPM) {
      reference = v;
      stable_since = pros::millis();
    } else if (pros::millis() - stable_since >= (uint32_t)SETTLE_MS) {
      break;
    }
  }

  double sum = 0.0, battery_sum = 0.0;
  int n = 0;
  uint32_t avg_start = pros::millis();
  while (pros::millis() - avg_start < (uint32_t)AVERAGE_MS) {
    sum += side_velocity(chassis, motors);
    battery_sum += pros::battery::get_voltage();
    n++;
    pros::delay(SAMPLE_MS);
  }

  side_command(chassis, left_side, 0);
  battery_mv = n > 0 ? battery_sum / n : pros::battery::get_voltage();
  return n > 0 ? sum / n : 0.0;
}

// Steps one side through every level and prints the table it measured.
DriveProfileFit characterize_side(ez::Drive& chassis, bool left_side) {
  const char* name = left_side ? "left" : "right";
  double commands[LEVELS] = {0.0};
  double rpms[LEVELS] = {0.0};

  wait_for_temp(chassis, 50.0);
  printf("[profile] %s side, %d levels:\n", name, LEVELS);
  printf("[profile]    cmd      rpm   battery mV\n");

  for (int i = 0; i < LEVELS; i++) {
    int cmd = LEVEL_START + i * LEVEL_STEP;
    double battery_mv = 0.0;
    double rpm = hold_and_measure(chassis, left_side, cmd, battery_mv);

    commands[i] = cmd;
    rpms[i] = std::fabs(rpm);
    printf("[profile]    %3d   %6.1f        %5.0f\n", cmd, rpms[i], battery_mv);
    pros::delay(REST_MS);
  }

  DriveProfileFit fit = drive_profile_fit(commands, rpms, LEVELS);
  if (!fit.ok) {
    printf("[profile] %s side never moved; check the pto list, the port list, and that\n", name);
    printf("[profile] the wheels are free to spin.\n");
    return fit;
  }
  printf("[profile] %s: kS = %.1f, kV = %.4f cmd/rpm (r2 = %.3f over %d moving points)\n",
         name, fit.ks, fit.kv, fit.r2, fit.points);
  return fit;
}

}  // namespace

DriveProfilePair drive_characterize(ez::Drive& chassis, pros::Controller& controller,
                                    const char* path) {
  DriveProfilePair measured;

  printf("[profile] Drive characterization: put the robot on a stand with the\n");
  printf("[profile] WHEELS OFF THE FLOOR, then press A. B cancels.\n");
  controller.set_text(0, 0, "Wheels up: A=go B=x");

  while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) break;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      printf("[profile] Characterization cancelled.\n");
      controller.set_text(0, 0, "Cancelled          ");
      return measured;
    }
    pros::delay(50);
  }

  // Measure the drive as it actually is, not as it is already being corrected.
  bool was_enabled = g_enabled;
  g_enabled = false;
  controller.set_text(0, 0, "Running left...    ");

  DriveProfileFit left = characterize_side(chassis, true);
  chassis.drive_set(0, 0);
  pros::delay(1000);

  controller.set_text(0, 0, "Running right...   ");
  DriveProfileFit right = characterize_side(chassis, false);
  chassis.drive_set(0, 0);

  g_enabled = was_enabled;

  measured.left.ks = left.ks;
  measured.left.kv = left.kv;
  measured.right.ks = right.ks;
  measured.right.kv = right.kv;

  if (!measured.valid()) {
    printf("[profile] Characterization failed, nothing saved.\n");
    controller.set_text(0, 0, "Failed             ");
    return measured;
  }

  g_profile = measured;
  internal::sd_line_save("dp-ls", measured.left.ks, path);
  internal::sd_line_save("dp-lv", measured.left.kv, path);
  internal::sd_line_save("dp-rs", measured.right.ks, path);
  internal::sd_line_save("dp-rv", measured.right.kv, path);

  double left_balance = drive_profile_balance_scale(measured.left.kv, measured.right.kv);
  double right_balance = drive_profile_balance_scale(measured.right.kv, measured.left.kv);
  printf("[profile] Balance: left x%.3f, right x%.3f (the faster side is the scaled one).\n",
         left_balance, right_balance);
  printf("[profile] Saved. Compensation is %s; ez::drive_profile_enable(true) turns it on,\n",
         g_enabled ? "ON" : "still OFF");
  printf("[profile] and re-tune your PIDs afterward, it changes how the drive responds.\n");
  controller.set_text(0, 0, "Done, saved        ");
  return measured;
}

/////
//
// Straightness self-test
//
/////

namespace {

constexpr int STRAIGHT_POWER = 60;        // open loop, both sides, same number
constexpr double STRAIGHT_TARGET_IN = 72.0;  // 6 ft of the 8 ft runway
constexpr uint32_t STRAIGHT_CAP_MS = 4000;   // give up if the target never arrives
constexpr double STRAIGHT_ABORT_IN = 80.0;   // past this the runway is gone
constexpr double STRAIGHT_ABORT_DEG = 30.0;  // past this it is not driving straight
constexpr int STRAIGHT_SAMPLE_MS = 10;
constexpr int STRAIGHT_SETTLE_MS = 400;  // let the robot stop rocking before reading

// Every sensor here can hand back PROS_ERR_F, and drive_sensor_* divides by a
// tick count that a misconfigured chassis can leave at zero. Anything that is
// not a real number becomes 0.0 so the comparisons below stay meaningful: a
// NaN loses every comparison it is in, which would quietly disable the abort
// checks instead of tripping them.
double finite_or_zero(double v) { return std::isfinite(v) ? v : 0.0; }

// A side that is not answering reads PROS_ERR raw, and drive_sensor_* divides
// that by the tick count and hands back a finite but enormous number of
// inches - far enough "out" to trip the runway abort before the robot has
// moved at all. Checking the raw value is the only way to tell that apart from
// a real reading, so dead sides are dropped from the average the same way
// characterization skips a motor whose velocity reads PROS_ERR_F.
bool straight_side_live(ez::Drive& chassis, bool left_side) {
  int raw = left_side ? chassis.drive_sensor_left_raw() : chassis.drive_sensor_right_raw();
  return raw != PROS_ERR;
}

int straight_live_sides(ez::Drive& chassis) {
  return (straight_side_live(chassis, true) ? 1 : 0) + (straight_side_live(chassis, false) ? 1 : 0);
}

// Average travel over whichever sides are still reporting. With none left this
// is 0.0 forever, which reads as "not moving": the leg then ends on its time
// cap instead of its distance target, and the abort limits stay clear.
double straight_position(ez::Drive& chassis) {
  double sum = 0.0;
  int n = 0;
  if (straight_side_live(chassis, true)) {
    sum += finite_or_zero(chassis.drive_sensor_left());
    n++;
  }
  if (straight_side_live(chassis, false)) {
    sum += finite_or_zero(chassis.drive_sensor_right());
    n++;
  }
  return n > 0 ? sum / n : 0.0;
}

double straight_heading(ez::Drive& chassis) { return finite_or_zero(chassis.drive_imu_get()); }

// Zeroes everything the run is measured against. The sensor reset also clears
// the tracking task's "last" values, so the pose is zeroed after a short delay
// once that task has taken a clean cycle and cannot fold the reset step into
// the drift being measured.
void straight_reset(ez::Drive& chassis) {
  chassis.drive_imu_reset(0);
  chassis.drive_sensor_reset();
  pros::delay(150);
  if (chassis.odom_enabled()) chassis.odom_xy_set(0.0, 0.0);
}

// Drives open loop until the average drive sensor covers `target_in` of travel
// from where it is now, and stops. `power` carries the direction. Returns
// false if a limit cut the run short.
bool straight_leg(ez::Drive& chassis, int power, double target_in, double& travelled) {
  double start = straight_position(chassis);
  uint32_t begin = pros::millis();
  bool aborted = false;

  chassis.drive_set(power, power);
  while (true) {
    double position = straight_position(chassis);
    double moved = std::fabs(position - start);
    double total = std::fabs(position);
    double heading = std::fabs(straight_heading(chassis));

    if (total > STRAIGHT_ABORT_IN) {
      printf("[straight] ABORT: drive is %.1f in out, past the %.0f in runway limit.\n",
             total, STRAIGHT_ABORT_IN);
      aborted = true;
      break;
    }
    if (heading > STRAIGHT_ABORT_DEG) {
      printf("[straight] ABORT: turned %.1f deg, past the %.0f deg limit.\n",
             heading, STRAIGHT_ABORT_DEG);
      aborted = true;
      break;
    }
    if (moved >= target_in) break;
    if (pros::millis() - begin >= STRAIGHT_CAP_MS) {
      // Not an abort: a slow drive, or one whose encoders never moved, still
      // has to hand the motors back rather than sit here pushing.
      printf("[straight] %u ms cap hit after %.1f in of %.0f in.\n",
             (unsigned)STRAIGHT_CAP_MS, moved, target_in);
      break;
    }
    pros::delay(STRAIGHT_SAMPLE_MS);
  }

  chassis.drive_set(0, 0);
  travelled = std::fabs(straight_position(chassis) - start);
  pros::delay(STRAIGHT_SETTLE_MS);
  return !aborted;
}

// One full there-and-back run with compensation pinned to `enabled`.
DriveStraightnessRun straight_phase(ez::Drive& chassis, bool enabled, const char* label) {
  DriveStraightnessRun run;
  g_enabled = enabled;

  wait_for_temp(chassis, 50.0);
  printf("[straight] %s: driving out at power %d.\n", label, STRAIGHT_POWER);
  straight_reset(chassis);

  int live = straight_live_sides(chassis);
  if (live == 0)
    printf("[straight] WARNING: neither drive sensor is answering; this run measures\n"
           "[straight] nothing and will stop on its time cap.\n");
  else if (live == 1)
    printf("[straight] WARNING: only one drive sensor is answering; distance is from\n"
           "[straight] that side alone.\n");

  double out = 0.0;
  bool clean = straight_leg(chassis, STRAIGHT_POWER, STRAIGHT_TARGET_IN, out);

  run.ran = true;
  run.aborted = !clean;
  run.distance_in = out;
  run.heading_deg = straight_heading(chassis);
  if (chassis.odom_enabled()) {
    // Forward from heading 0 increases odom y, so x is the sideways axis and
    // is the drift this test is after (tracking.cpp solve_xy_vert).
    run.lateral_in = finite_or_zero(chassis.odom_x_get());
    run.lateral_valid = true;
  }

  printf("[straight] %s: %.1f in out, %.1f deg drift", label, run.distance_in, run.heading_deg);
  if (run.lateral_valid)
    printf(", %.1f in sideways\n", run.lateral_in);
  else
    printf(", odom off so no lateral\n");

  // Back it up whether or not the run was clean, so the robot ends near where
  // it started; the return trip is not measured.
  printf("[straight] %s: backing up %.1f in.\n", label, run.distance_in);
  double back = 0.0;
  straight_leg(chassis, -STRAIGHT_POWER, run.distance_in, back);
  return run;
}

void straight_report(const DriveStraightnessResult& result) {
  const DriveStraightnessRun& off = result.off;
  const DriveStraightnessRun& on = result.on;

  if (on.ran) {
    printf("[straight] profile off: %.1f deg, %.1f in | profile on: %.1f deg, %.1f in\n",
           off.heading_deg, off.lateral_in, on.heading_deg, on.lateral_in);
  } else {
    printf("[straight] profile off: %.1f deg, %.1f in | profile on: not run\n",
           off.heading_deg, off.lateral_in);
  }
  if (!off.lateral_valid || (on.ran && !on.lateral_valid))
    printf("[straight] (odom is off, so the inch column is not a measurement)\n");

  if (!on.ran) {
    printf("[straight] Verdict: no profile, nothing to compare. Run Drive Characterize first.\n");
    return;
  }

  double before = std::fabs(off.heading_deg);
  double after = std::fabs(on.heading_deg);
  double delta = before - after;
  if (std::fabs(delta) < 0.5) {
    printf("[straight] Verdict: no change, %.1f deg either way.\n", before);
  } else if (delta > 0.0) {
    printf("[straight] Verdict: improved, %.1f deg less drift (%.1f -> %.1f).\n",
           delta, before, after);
  } else {
    printf("[straight] Verdict: WORSE, %.1f deg more drift (%.1f -> %.1f). Re-characterize.\n",
           -delta, before, after);
  }
  if (off.aborted || on.aborted)
    printf("[straight] A run was cut short by a limit, so treat these numbers as a floor.\n");
}

}  // namespace

DriveStraightnessResult drive_straightness_test(ez::Drive& chassis, pros::Controller& controller) {
  DriveStraightnessResult result;

  printf("[straight] Straightness test: the robot drives itself %.0f in forward and\n",
         STRAIGHT_TARGET_IN);
  printf("[straight] back, twice. Clear 8 ft ahead of it, then press A. B cancels.\n");
  if (!chassis.drive_imu_calibrated())
    printf("[straight] WARNING: the imu is not calibrated, so heading drift will read 0.\n");
  controller.set_text(0, 0, "8ft clear! A=go B=x");

  while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) break;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      printf("[straight] Straightness test cancelled.\n");
      controller.set_text(0, 0, "Cancelled          ");
      result.cancelled = true;
      return result;
    }
    pros::delay(50);
  }

  bool was_enabled = g_enabled;

  controller.set_text(0, 0, "Phase 1: profile off");
  result.off = straight_phase(chassis, false, "profile off");
  pros::delay(1000);

  if (g_profile.valid()) {
    controller.set_text(0, 0, "Phase 2: profile on ");
    result.on = straight_phase(chassis, true, "profile on");
  } else {
    printf("[straight] Phase 2 skipped: no valid drive profile is loaded, so turning\n");
    printf("[straight] compensation on would change nothing. Run Drive Characterize first.\n");
  }

  chassis.drive_set(0, 0);
  g_enabled = was_enabled;

  straight_report(result);
  controller.set_text(0, 0, "Done, see terminal ");
  return result;
}

void drive_straightness_register(ez::Drive& chassis) {
  // Appended the same way drive_characterize_register does it: autons_add()
  // assigns over the list, so anything already registered would be lost.
  ez::as::auton_selector.Autons.push_back(
      Auton("Straightness Test\n\n8ft clear ahead first!", [&chassis]() {
        pros::Controller controller(pros::E_CONTROLLER_MASTER);
        drive_straightness_test(chassis, controller);
      }));
  ez::as::auton_selector.auton_count++;
  printf("[straight] Straightness Test added to the auton selector as page %d.\n",
         ez::as::auton_selector.auton_count);
}

void drive_characterize_register(ez::Drive& chassis) {
  // autons_add() assigns over the list instead of appending, so registering
  // through it would drop every auton already added. Push straight onto the
  // public members instead.
  ez::as::auton_selector.Autons.push_back(
      Auton("Drive Characterize\n\nWheels OFF the floor first!", [&chassis]() {
        pros::Controller controller(pros::E_CONTROLLER_MASTER);
        drive_characterize(chassis, controller);
      }));
  ez::as::auton_selector.auton_count++;
  printf("[profile] Drive Characterize added to the auton selector as page %d.\n",
         ez::as::auton_selector.auton_count);
}

}  // namespace ez
