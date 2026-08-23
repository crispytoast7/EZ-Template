#include "EZ-Template/tuner.hpp"

#include "EZ-Template/drive_profile.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ez {

// Left, right, back, front.
static constexpr int TRACKER_SLOTS = 4;

PIDAutoTuner::PIDAutoTuner(ez::Drive& chassis) : chassis(chassis) {}

void PIDAutoTuner::runway_set(double inches) {
  runway_in_ = std::max(24.0, inches);
  printf("[tuner] Runway set to %.0f in.\n", runway_in_);
}

const PIDAutoTuner::MotionSpec& PIDAutoTuner::spec(Motion m) {
  // refine_scales: Kp must pass at base x each scale (drive 12/24/48 in,
  // turns 45/90/135 deg) so the constants hold across move sizes.
  static constexpr MotionSpec drive   = {"drive",   "Drive",   "in",  false, true,  false, TEST_TIMEOUT_MS,         4000,                    0.5, 1.0, DRIVE_OVERSHOOT_IN,    0.15, {1.0, 2.0, 4.0}, 3};
  static constexpr MotionSpec turn    = {"turn",    "Turn",    "deg", true,  false, false, TEST_TIMEOUT_MS,         4000,                    1.0, 3.0, ANGLE_OVERSHOOT_DEG,   0.75, {0.5, 1.0, 1.5}, 3};
  static constexpr MotionSpec swing   = {"swing",   "Swing",   "deg", true,  false, false, TEST_TIMEOUT_MS,         4000,                    1.0, 3.0, ANGLE_OVERSHOOT_DEG,   0.75, {0.5, 1.0, 1.5}, 3};
  static constexpr MotionSpec heading = {"heading", "Heading", "deg", true,  true,  true,  HEADING_TEST_TIMEOUT_MS, HEADING_TEST_TIMEOUT_MS, 0.5, 2.0, HEADING_OVERSHOOT_DEG, 0.3,  {1.0, 0.0, 0.0}, 1};
  switch (m) {
    case Motion::TURN:    return turn;
    case Motion::SWING:   return swing;
    case Motion::HEADING: return heading;
    default:              return drive;
  }
}

double PIDAutoTuner::read_pos() {
  return (chassis.drive_sensor_left() + chassis.drive_sensor_right()) * 0.5;
}

double PIDAutoTuner::read(Motion m) {
  return (m == Motion::DRIVE) ? read_pos() : chassis.drive_imu_get();
}

void PIDAutoTuner::reset_sensors(Motion m) {
  chassis.drive_sensor_reset();
  if (spec(m).use_imu) chassis.drive_imu_reset(0);
  pros::delay(150);
}

void PIDAutoTuner::set_output(Motion m, double cmd, int forward_speed) {
  switch (m) {
    case Motion::DRIVE:
      chassis.drive_set(static_cast<int>(cmd), static_cast<int>(cmd));
      break;
    case Motion::TURN:
      chassis.drive_set(static_cast<int>(cmd), -static_cast<int>(cmd));
      break;
    case Motion::SWING:
      chassis.drive_set(static_cast<int>(cmd), 0);
      break;
    case Motion::HEADING: {
      int left  = ez::util::clamp(static_cast<int>(forward_speed + cmd), 127, -127);
      int right = ez::util::clamp(static_cast<int>(forward_speed - cmd), 127, -127);
      chassis.drive_set(left, right);
      break;
    }
  }
}

double PIDAutoTuner::motor_max_temp() {
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

void PIDAutoTuner::wait_for_temp(double max_c) {
  double t = motor_max_temp();
  if (t <= max_c) return;
  while (true) {
    t = motor_max_temp();
    if (t <= max_c) break;
    printf("[tuner] Motor at %.0f C, waiting for <= %.0f C\n", t, max_c);
    pros::delay(5000);
  }
  printf("[tuner] Motors at %.0f C, resuming.\n", t);
}

void PIDAutoTuner::tl_constants(double ku, double pu_ms,
                                double& kp, double& ki, double& kd) {
  // Tyreus-Luyben, rescaled for the discrete loop (EZ sums raw error with no
  // timestep, so ki is multiplied and kd divided by the loop period).
  double pu_s = pu_ms / 1000.0;
  kp = ku / 3.22;
  ki = (kp / (2.2 * pu_s)) * DT_S;
  kd = (kp * (pu_s / 6.3)) / DT_S;
}

PIDAutoTuner::OscData PIDAutoTuner::probe(Motion m, double target, int relay_amp, int forward_speed) {
  const MotionSpec& s = spec(m);
  OscData result;

  // Command amplitude is scaled by battery voltage so measured dynamics do
  // not depend on charge level.
  double bat_mv = pros::battery::get_voltage();
  int adj_amp = std::min(127 - forward_speed, static_cast<int>(relay_amp * (12500.0 / bat_mv)));

  reset_sensors(m);

  std::vector<uint32_t> crossing_times;
  std::vector<double> half_peaks;
  const uint32_t t_start = pros::millis();
  int skipped = 0;
  double current_peak = 0.0;
  double prev_err = target;

  // Relay with hysteresis: the output only flips once the error leaves the
  // band, so sensor noise near zero cannot corrupt the measured period.
  int relay_sign = 1;

  while (pros::millis() - t_start < (uint32_t)s.probe_timeout_ms) {
    if (m == Motion::DRIVE || s.dist_abort) {
      double pos = read_pos();
      double fwd_limit = s.dist_abort ? std::min((double)HEADING_MAX_DIST_IN, runway_in_ - 4.0)
                                      : runway_in_ - 4.0;
      if (pos > fwd_limit || pos < -12.0) {
        printf("[tuner] Runway limit hit at %.1f in, stopping this run.\n", pos);
        break;
      }
    }

    double err = target - read(m);

    if (err > s.relay_hyst) relay_sign = 1;
    else if (err < -s.relay_hyst) relay_sign = -1;
    set_output(m, relay_sign * adj_amp, forward_speed);

    current_peak = std::max(current_peak, std::abs(err));

    bool crossed = (prev_err >= 0.0 && err < 0.0) || (prev_err < 0.0 && err >= 0.0);
    if (crossed) {
      if (skipped < SKIP_CROSSINGS) {
        skipped++;
        current_peak = 0.0;
      } else {
        crossing_times.push_back(pros::millis());
        half_peaks.push_back(current_peak);
        current_peak = 0.0;

        if ((int)crossing_times.size() >= MIN_CROSSINGS) {
          double sum_t = 0.0;
          for (size_t i = 1; i < crossing_times.size(); i++)
            sum_t += (double)(crossing_times[i] - crossing_times[i - 1]);
          double avg_half_ms = sum_t / (double)(crossing_times.size() - 1);

          double sum_a = 0.0;
          for (double a : half_peaks) sum_a += a;
          double A = sum_a / (double)half_peaks.size();

          if (A < 0.01 || A <= s.relay_hyst * 1.5) break;
          result.detected = true;
          // Describing function of a relay with hysteresis: the effective
          // amplitude shrinks to sqrt(A^2 - h^2).
          result.ku = (4.0 * adj_amp) / (M_PI * std::sqrt(A * A - s.relay_hyst * s.relay_hyst));
          result.pu_ms = avg_half_ms * 2.0;
          break;
        }
      }
    }

    prev_err = err;
    pros::delay(DT_MS);
  }

  chassis.drive_set(0, 0);
  return result;
}

void PIDAutoTuner::return_drive(int speed) {
  const uint32_t t_start = pros::millis();

  while (pros::millis() - t_start < 5000) {
    double err = -read_pos();
    if (std::abs(err) < 0.75) break;

    double out = ez::util::clamp(10.0 * err, speed, -speed);
    chassis.drive_set(static_cast<int>(out), static_cast<int>(out));
    pros::delay(DT_MS);
  }

  chassis.drive_set(0, 0);
}

double PIDAutoTuner::trial(Motion m, double target, int speed, double kp, double kd) {
  const MotionSpec& s = spec(m);
  reset_sensors(m);

  double prev_err = target;
  double max_meas = 0.0;
  double meas = 0.0;
  bool in_settle = false;
  uint32_t t_settle = 0;
  const uint32_t t_start = pros::millis();

  while (pros::millis() - t_start < (uint32_t)s.trial_timeout_ms) {
    if (m == Motion::DRIVE || s.dist_abort) {
      double pos = read_pos();
      double fwd_limit = s.dist_abort ? std::min((double)HEADING_MAX_DIST_IN, runway_in_ - 4.0)
                                      : runway_in_ - 4.0;
      if (pos > fwd_limit || pos < -12.0) {
        printf("[tuner] Runway limit hit at %.1f in, stopping this run.\n", pos);
        break;
      }
    }

    meas = read(m);
    double err = target - meas;
    double deriv = err - prev_err;
    double out = ez::util::clamp(kp * err + kd * deriv, speed, -speed);
    set_output(m, out, speed);
    max_meas = std::max(max_meas, meas);

    if (std::abs(err) < s.settle_tol) {
      if (!in_settle) { in_settle = true; t_settle = pros::millis(); }
      if (pros::millis() - t_settle > 300) break;
    } else {
      in_settle = false;
    }

    prev_err = err;
    pros::delay(DT_MS);
  }

  chassis.drive_set(0, 0);
  // Negative return means the target was never reached.
  if (target - meas > s.unreached_tol) return -(target - meas);
  return (max_meas > target) ? max_meas - target : 0.0;
}

double PIDAutoTuner::refine_kp(Motion m, double base_target, int speed,
                               double kp_lo, double kp_hi, double kd) {
  const MotionSpec& s = spec(m);
  double kp = (kp_lo + kp_hi) / 2.0;
  printf("[tuner] Refining %s Kp [%.3f - %.3f] over %d target sizes...\n",
         s.name, kp_lo, kp_hi, s.n_refine);

  for (int i = 0; i < MAX_REFINE_ITER; i++) {
    // A candidate must pass at every size: any unreached target means too
    // low, any overshoot past the limit means too high.
    bool any_unreached = false;
    double worst_os = 0.0;

    for (int ti = 0; ti < s.n_refine; ti++) {
      double target = base_target * s.refine_scales[ti];
      if (m == Motion::DRIVE) {
        double max_t = runway_in_ - 8.0;
        if (target > max_t) {
          if (ti > 0 && base_target * s.refine_scales[ti - 1] >= max_t) continue;
          printf("[tuner]   (%.0f in target clamped to %.0f for the runway)\n", target, max_t);
          target = max_t;
        }
      }
      wait_for_temp(52.0);
      double os = trial(m, target, speed, kp, kd);
      if (s.go_home) return_drive(speed);
      pros::delay(1500);
      printf("[tuner]   iter %d  Kp=%.4f  @ %.1f %s  result=%.3f %s\n",
             i + 1, kp, target, s.unit, os, s.unit);

      if (os < 0.0) { any_unreached = true; break; }
      worst_os = std::max(worst_os, os);
    }

    if (any_unreached)                   kp_lo = kp;
    else if (worst_os > s.overshoot_max) kp_hi = kp;
    else break;

    kp = (kp_lo + kp_hi) / 2.0;
  }

  printf("[tuner]   Final %s Kp = %.4f\n", s.name, kp);
  return kp;
}

void PIDAutoTuner::print_result(const char* label, const TunerResult& r) {
  printf("\n[tuner] === %s ===\n", label);
  if (!r.success) {
    printf("  FAILED: %s\n", r.message.c_str());
    return;
  }
  printf("  Ku = %.4f,  Pu = %.1f ms\n", r.ku, r.pu_ms);
  printf("  kp = %.4f\n  ki = %.6f\n  kd = %.4f\n", r.kp, r.ki, r.kd);
}

TunerResult PIDAutoTuner::tune(Motion m, double target, int relay_amp, int forward_speed) {
  const MotionSpec& s = spec(m);
  TunerResult result;

  char label[32];
  snprintf(label, sizeof(label), "%s PID Results", s.cap_name);

  if (m == Motion::HEADING)
    printf("[tuner] Heading relay test (%d runs), drive speed %d, %.1f deg, relay amp %d\n",
           RELAY_RUNS, forward_speed, target, relay_amp);
  else
    printf("[tuner] %s relay test (%d runs), %.1f %s, relay amp %d\n",
           s.cap_name, RELAY_RUNS, target, s.unit, relay_amp);

  const int run_speed = (m == Motion::HEADING) ? forward_speed : relay_amp;

  double sum_ku = 0.0, sum_pu = 0.0;
  int succeeded = 0;

  for (int run = 1; run <= RELAY_RUNS; run++) {
    wait_for_temp(50.0);
    printf("[tuner]   Run %d/%d...\n", run, RELAY_RUNS);
    OscData osc = probe(m, target, relay_amp, (m == Motion::HEADING) ? forward_speed : 0);
    if (s.go_home) return_drive(run_speed);
    pros::delay(COOL_DOWN_MS);

    if (osc.detected) {
      sum_ku += osc.ku;
      sum_pu += osc.pu_ms;
      succeeded++;
      printf("           Ku=%.4f  Pu=%.1f ms\n", osc.ku, osc.pu_ms);
    } else {
      printf("           no oscillation\n");
    }
  }

  if (succeeded == 0) {
    result.message = "No oscillation in any run; try raising relay_amp.";
    print_result(label, result);
    return result;
  }

  double ku = sum_ku / succeeded;
  double pu = sum_pu / succeeded;
  printf("[tuner]   Average (%d/%d): Ku=%.4f  Pu=%.1f ms\n", succeeded, RELAY_RUNS, ku, pu);

  double kp_tl, ki_tl, kd_tl;
  tl_constants(ku, pu, kp_tl, ki_tl, kd_tl);

  double kp_refined = refine_kp(m, target, run_speed, kp_tl * 0.5, ku * 0.85, kd_tl);

  double ki_final = ki_tl;
  if (m == Motion::HEADING) {
    ki_final = 0.0;
    // The relay test excites a coarse kick, but heading runs continuously
    // against sub-degree drift; cap both gains against the drive-derived
    // estimate so steady-state noise cannot become jitter.
    if (last_drive_.success) {
      double kp_cap = last_drive_.kp * 0.55 * 1.5;
      if (kp_refined > kp_cap) {
        printf("[tuner]   Heading Kp %.4f capped to %.4f\n", kp_refined, kp_cap);
        kp_refined = kp_cap;
      }
      double kd_cap = last_drive_.kd * 0.20 * 1.5;
      if (kd_tl > kd_cap) {
        printf("[tuner]   Heading Kd %.4f capped to %.4f\n", kd_tl, kd_cap);
        kd_tl = kd_cap;
      }
    }
  }

  result.ku = ku;
  result.pu_ms = pu;
  result.kp = kp_refined;
  result.ki = ki_final;
  result.kd = kd_tl;
  result.success = true;
  print_result(label, result);
  printf("    chassis.pid_%s_constants_set(%.4f, %.6f, %.4f);\n",
         s.name, result.kp, result.ki, result.kd);

  switch (m) {
    case Motion::DRIVE:
      chassis.pid_drive_constants_set(result.kp, result.ki, result.kd);
      last_drive_ = result;
      break;
    case Motion::TURN:
      chassis.pid_turn_constants_set(result.kp, result.ki, result.kd);
      last_turn_ = result;
      break;
    case Motion::SWING:
      chassis.pid_swing_constants_set(result.kp, result.ki, result.kd);
      last_swing_ = result;
      break;
    case Motion::HEADING:
      chassis.pid_heading_constants_set(result.kp, result.ki, result.kd);
      last_heading_kp_ = result.kp;
      last_heading_ki_ = result.ki;
      last_heading_kd_ = result.kd;
      break;
  }
  return result;
}

TunerResult PIDAutoTuner::tune_drive(double test_distance_in, int relay_amp) {
  return tune(Motion::DRIVE, test_distance_in, relay_amp);
}

TunerResult PIDAutoTuner::tune_turn(double test_angle_deg, int relay_amp) {
  return tune(Motion::TURN, test_angle_deg, relay_amp);
}

TunerResult PIDAutoTuner::tune_swing(double test_angle_deg, int relay_amp) {
  return tune(Motion::SWING, test_angle_deg, relay_amp);
}

TunerResult PIDAutoTuner::tune_heading(int forward_speed, double target_deg, int relay_amp) {
  return tune(Motion::HEADING, target_deg, relay_amp, forward_speed);
}

void PIDAutoTuner::set_heading_from_drive() {
  if (!last_drive_.success) {
    printf("[tuner] set_heading_from_drive skipped: drive not tuned.\n");
    return;
  }

  last_heading_kp_ = last_drive_.kp * 0.55;
  last_heading_ki_ = 0.0;
  last_heading_kd_ = last_drive_.kd * 0.20;
  chassis.pid_heading_constants_set(last_heading_kp_, last_heading_ki_, last_heading_kd_);
  printf("[tuner] Heading PID derived from drive:\n");
  printf("    chassis.pid_heading_constants_set(%.4f, %.6f, %.4f);\n",
         last_heading_kp_, last_heading_ki_, last_heading_kd_);
}

bool PIDAutoTuner::save_to_sd(const char* path) {
  FILE* f = fopen(path, "w");
  if (!f) {
    printf("[tuner] SD card not available, constants NOT saved.\n");
    return false;
  }

  if (last_drive_.success)
    fprintf(f, "drive %.6f %.8f %.6f\n", last_drive_.kp, last_drive_.ki, last_drive_.kd);
  if (last_heading_kp_ > 0.0)
    fprintf(f, "heading %.6f %.8f %.6f\n", last_heading_kp_, last_heading_ki_, last_heading_kd_);
  if (last_turn_.success)
    fprintf(f, "turn %.6f %.8f %.6f\n", last_turn_.kp, last_turn_.ki, last_turn_.kd);
  if (last_swing_.success)
    fprintf(f, "swing %.6f %.8f %.6f\n", last_swing_.kp, last_swing_.ki, last_swing_.kd);

  fclose(f);
  printf("[tuner] Constants saved to %s; they load on every boot.\n", path);
  return true;
}

// One spin sequence shared by every tracker: each wheel's rolled distance per
// radian turned, averaged over the runs that completed a turn. Returns the
// number of runs used, and fills offsets[] with the averages.
static int spin_offsets(ez::Drive& chassis, ez::tracking_wheel* const* trackers,
                        const char* const* names, double* offsets, int count,
                        int iterations, int turn_speed) {
  double start[TRACKER_SLOTS] = {0.0, 0.0, 0.0, 0.0};
  for (int j = 0; j < count; j++) offsets[j] = 0.0;
  int used = 0;

  for (int i = 0; i < iterations; i++) {
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();

    for (int j = 0; j < count; j++) start[j] = trackers[j]->get();

    // raw, so a full turn is not shortest-pathed into no motion at all.
    chassis.pid_turn_set(360.0, turn_speed, ez::raw);
    chassis.pid_wait();
    pros::delay(300);

    double turned_rad = chassis.drive_imu_get() * M_PI / 180.0;
    if (std::fabs(turned_rad) < 0.5) continue;

    for (int j = 0; j < count; j++) {
      double rolled = trackers[j]->get() - start[j];
      double offset = rolled / turned_rad;
      printf("[tuner] %s run %d: rolled %.2f in over %.1f deg -> %.3f in\n",
             names[j], i + 1, rolled, turned_rad * 180.0 / M_PI, offset);
      offsets[j] += offset;
    }
    used++;
  }

  if (used > 0)
    for (int j = 0; j < count; j++) offsets[j] /= used;
  return used;
}

static const char* slot_name(tracker_slot slot) {
  switch (slot) {
    case tracker_slot::LEFT:  return "left";
    case tracker_slot::RIGHT: return "right";
    case tracker_slot::FRONT: return "front";
    default:                  return "back";
  }
}

static const char* slot_label(tracker_slot slot) {
  switch (slot) {
    case tracker_slot::LEFT:  return "trk-l";
    case tracker_slot::RIGHT: return "trk-r";
    case tracker_slot::FRONT: return "trk-f";
    default:                  return "trk-b";
  }
}

double tracker_offset_measure(ez::Drive& chassis, ez::tracking_wheel& tracker,
                              int iterations, int turn_speed) {
  ez::tracking_wheel* one[1] = {&tracker};
  const char* name[1] = {"offset"};
  double offset[1] = {0.0};

  int used = spin_offsets(chassis, one, name, offset, 1, iterations, turn_speed);
  if (used == 0) {
    printf("[tuner] Tracker offset measurement failed: no full turns completed.\n");
    return NAN;
  }
  printf("[tuner] Tracker offset: %.3f in (from %d runs)\n", offset[0], used);
  return offset[0];
}

int tracker_offsets_measure(ez::Drive& chassis, int iterations, int turn_speed) {
  ez::tracking_wheel* all[TRACKER_SLOTS] = {chassis.odom_tracker_left, chassis.odom_tracker_right,
                                            chassis.odom_tracker_back, chassis.odom_tracker_front};
  const tracker_slot all_slots[TRACKER_SLOTS] = {tracker_slot::LEFT, tracker_slot::RIGHT,
                                                 tracker_slot::BACK, tracker_slot::FRONT};

  ez::tracking_wheel* trackers[TRACKER_SLOTS];
  tracker_slot slots[TRACKER_SLOTS];
  const char* names[TRACKER_SLOTS];
  double offsets[TRACKER_SLOTS];
  int count = 0;

  for (int i = 0; i < TRACKER_SLOTS; i++) {
    if (all[i] == nullptr) continue;
    trackers[count] = all[i];
    slots[count] = all_slots[i];
    names[count] = slot_name(all_slots[i]);
    count++;
  }

  if (count == 0) {
    printf("[tuner] No tracking wheels on the chassis, nothing to measure.\n");
    return 0;
  }

  printf("[tuner] Tracker offsets: %d wheel(s), %d spins.\n", count, iterations);
  int used = spin_offsets(chassis, trackers, names, offsets, count, iterations, turn_speed);
  if (used == 0) {
    printf("[tuner] Tracker offset measurement failed: no full turns completed.\n");
    return 0;
  }

  for (int i = 0; i < count; i++) {
    printf("[tuner] %s tracker offset: %.3f in (from %d runs)\n", names[i], offsets[i], used);
    tracker_offset_apply(*trackers[i], offsets[i], slots[i]);
    tracker_offset_save(slots[i], offsets[i]);
  }
  return count;
}

void PIDAutoTuner::interactive(pros::Controller& controller) {
  const bool has_tracker = chassis.odom_tracker_left != nullptr || chassis.odom_tracker_right != nullptr ||
                           chassis.odom_tracker_back != nullptr || chassis.odom_tracker_front != nullptr;
  const int n = has_tracker ? 6 : 5;
  const char* items[6] = {"Drive", "Turn", "Swing", "Heading", "Everything", "Trk offset"};
  int sel = 0;

  printf("[tuner] Interactive: LEFT/RIGHT pick, A runs, B saves + exits.\n");
  controller.set_text(0, 0, "< " + std::string(items[sel]) + " >      ");

  while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      sel = (sel + 1) % n;
      controller.set_text(0, 0, "< " + std::string(items[sel]) + " >      ");
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      sel = (sel + n - 1) % n;
      controller.set_text(0, 0, "< " + std::string(items[sel]) + " >      ");
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) break;

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      controller.set_text(0, 0, "Running " + std::string(items[sel]) + "  ");
      switch (sel) {
        case 0: tune_drive(); break;
        case 1: tune_turn(); break;
        case 2: tune_swing(); break;
        case 3: tune_heading(); break;
        case 4:
          tune_drive();
          set_heading_from_drive();
          pros::delay(5000);
          tune_turn();
          pros::delay(5000);
          tune_swing();
          break;
        case 5:
          tracker_offsets_measure(chassis);
          break;
      }
      controller.rumble(".");
      controller.set_text(0, 0, "< " + std::string(items[sel]) + " > done ");
    }

    pros::delay(50);
  }

  save_to_sd();
  controller.set_text(0, 0, "Saved            ");
}

void tracker_offset_apply(ez::tracking_wheel& tracker, double measured, tracker_slot slot) {
  // A spin about the tracking center must leave that center still, so the
  // bracket in each solver has to cancel. The angle they are handed is math
  // standard, the negative of the imu (tracking.cpp:169), so the rolled per
  // radian they see is -measured. solve_xy_vert subtracts distance_to_center
  // (tracking.cpp:90) and solve_xy_horiz adds it (tracking.cpp:114), so a
  // parallel wheel wants -measured and a perpendicular wheel wants +measured.
  // Which side of center a wheel is on is already carried by the sign of the
  // measurement: the parallel case reproduces the drive's own known-good track
  // widths, left negative and right positive (tracking.cpp:35-36).
  bool perpendicular = slot == tracker_slot::BACK || slot == tracker_slot::FRONT;
  double effective = perpendicular ? measured : -measured;

  tracker.distance_to_center_set(std::fabs(effective));
  tracker.distance_to_center_flip_set(effective < 0.0);
  printf("[tuner] %s tracker offset set: %.3f in (flip %s)\n",
         slot_name(slot), std::fabs(effective), effective < 0.0 ? "on" : "off");
}

// Read-modify-write of one labeled line so other lines in the file survive.
// Zero padding keeps the loader's label-plus-3-numbers format.
namespace internal {
bool sd_line_save(const char* label, double value, const char* path) {
  std::string kept;
  FILE* f = fopen(path, "r");
  if (f) {
    char line[128];
    size_t n = strlen(label);
    while (fgets(line, sizeof(line), f)) {
      if (strncmp(line, label, n) != 0) kept += line;
    }
    fclose(f);
  }

  f = fopen(path, "w");
  if (!f) {
    printf("[tuner] SD card not available, %s NOT saved.\n", label);
    return false;
  }
  fputs(kept.c_str(), f);
  fprintf(f, "%s %.6f 0 0\n", label, value);
  fclose(f);
  printf("[tuner] %s saved to %s\n", label, path);
  return true;
}
}  // namespace internal

bool tracker_offset_save(tracker_slot slot, double measured, const char* path) {
  return internal::sd_line_save(slot_label(slot), measured, path);
}

double imu_scale_measure(ez::Drive& chassis, pros::Controller& controller,
                         int turns, const char* path) {
  double start = chassis.drive_imu_get();
  double expected = 360.0 * turns;

  printf("[tuner] IMU scale: rotate the robot BY HAND exactly %d full turns\n", turns);
  printf("[tuner] against a straight edge, then press A. B cancels.\n");
  controller.set_text(0, 0, "Spin " + std::to_string(turns) + "x, then A");

  while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) break;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      printf("[tuner] IMU scale cancelled.\n");
      controller.set_text(0, 0, "Cancelled       ");
      return 0.0;
    }
    pros::delay(50);
  }

  double reported = std::fabs(chassis.drive_imu_get() - start);
  if (reported < expected * 0.5) {
    printf("[tuner] IMU scale aborted: reported only %.1f deg of %.0f expected.\n",
           reported, expected);
    controller.set_text(0, 0, "Aborted         ");
    return 0.0;
  }

  double scaler = expected / reported;
  chassis.drive_imu_scaler_set(scaler);
  internal::sd_line_save("imuscale", scaler, path);
  printf("[tuner] IMU reported %.1f deg over %.0f actual; scaler = %.5f (applied).\n",
         reported, expected, scaler);
  printf("[tuner] Re-run the PID auto-tune so the tracker offset uses the new scale.\n");
  controller.set_text(0, 0, "Scaler " + std::to_string(scaler).substr(0, 7));
  return scaler;
}

// Skipped when that slot has no tracker on the chassis.
static bool tracker_line_load(ez::tracking_wheel* tracker, double measured, tracker_slot slot) {
  if (tracker == nullptr) return false;
  tracker_offset_apply(*tracker, measured, slot);
  return true;
}

bool pid_constants_load(ez::Drive& chassis, const char* path) {
  FILE* f = fopen(path, "r");
  if (!f) return false;

  char label[16];
  double kp, ki, kd;
  bool loaded_any = false;

  while (fscanf(f, "%15s %lf %lf %lf", label, &kp, &ki, &kd) == 4) {
    bool loaded = true;
    if (strcmp(label, "drive") == 0) {
      chassis.pid_drive_constants_set(kp, ki, kd);
    } else if (strcmp(label, "heading") == 0) {
      chassis.pid_heading_constants_set(kp, ki, kd);
    } else if (strcmp(label, "turn") == 0) {
      chassis.pid_turn_constants_set(kp, ki, kd);
    } else if (strcmp(label, "swing") == 0) {
      chassis.pid_swing_constants_set(kp, ki, kd);
    } else if (strcmp(label, "trk-l") == 0) {
      loaded = tracker_line_load(chassis.odom_tracker_left, kp, tracker_slot::LEFT);
    } else if (strcmp(label, "trk-r") == 0) {
      loaded = tracker_line_load(chassis.odom_tracker_right, kp, tracker_slot::RIGHT);
    } else if (strcmp(label, "trk-b") == 0) {
      loaded = tracker_line_load(chassis.odom_tracker_back, kp, tracker_slot::BACK);
    } else if (strcmp(label, "trk-f") == 0) {
      loaded = tracker_line_load(chassis.odom_tracker_front, kp, tracker_slot::FRONT);
    } else if (strcmp(label, "htrack") == 0) {
      // Legacy single-horizontal-tracker line.
      if (chassis.odom_tracker_back != nullptr)
        loaded = tracker_line_load(chassis.odom_tracker_back, kp, tracker_slot::BACK);
      else
        loaded = tracker_line_load(chassis.odom_tracker_front, kp, tracker_slot::FRONT);
    } else if (strcmp(label, "imuscale") == 0) {
      chassis.drive_imu_scaler_set(kp);
    } else if (strcmp(label, "dp-ls") == 0) {
      drive_profile().left.ks = kp;
    } else if (strcmp(label, "dp-lv") == 0) {
      drive_profile().left.kv = kp;
    } else if (strcmp(label, "dp-rs") == 0) {
      drive_profile().right.ks = kp;
    } else if (strcmp(label, "dp-rv") == 0) {
      drive_profile().right.kv = kp;
    } else {
      loaded = false;
    }

    if (loaded) {
      loaded_any = true;
      printf("[tuner] Loaded %s: %.4f %.6f %.4f\n", label, kp, ki, kd);
    }
  }

  fclose(f);
  if (loaded_any) printf("[tuner] PID constants loaded from %s\n", path);
  if (drive_profile().valid())
    printf("[tuner] Drive profile loaded; it does nothing until ez::drive_profile_enable(true).\n");
  return loaded_any;
}

}  // namespace ez
