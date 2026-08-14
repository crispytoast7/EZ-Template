#pragma once

#include "EZ-Template/drive/drive.hpp"
#include <cstdio>
#include <string>

namespace ez {

/// Result of one tuned motion. ki and kd are pre-scaled for EZ-Template's
/// discrete 10 ms loop.
struct TunerResult {
  bool success = false;
  double ku = 0;
  double pu_ms = 0;
  double kp = 0;
  double ki = 0;
  double kd = 0;
  std::string message;
};

/// Applies saved PID constants (and the tracking-wheel offset, when
/// `horiz_tracker` is given) from the SD card. Returns false if the file is
/// missing or contains nothing usable. Call once at startup before setting
/// fallback constants.
bool pid_constants_load(ez::Drive& chassis,
                        ez::tracking_wheel* horiz_tracker = nullptr,
                        const char* path = "/usd/pid_constants.txt");

/// Applies a measured horizontal-tracker offset. `measured` is the signed
/// rolled-distance / radians-turned value from a spin test; the sign is
/// translated into EZ-Template's magnitude-plus-flip convention.
void tracker_offset_apply(ez::tracking_wheel& tracker, double measured);

/// Persists a measured tracker offset without touching the PID lines in the
/// same file. Loaded automatically by pid_constants_load.
bool tracker_offset_save(double measured, const char* path = "/usd/pid_constants.txt");

/// Interactive IMU scale calibration. Prompts on the controller: rotate the
/// robot by hand exactly `turns` full rotations against a straight edge, then
/// press A (B cancels). Computes expected/reported, applies
/// drive_imu_scaler_set, and saves the scaler to the SD card, where
/// pid_constants_load restores it on every boot. Returns the scaler, or 0 on
/// cancel/failure.
double imu_scale_measure(ez::Drive& chassis, pros::Controller& controller,
                         int turns = 10,
                         const char* path = "/usd/pid_constants.txt");

/// Relay-feedback (Astrom-Hagglund) auto-tuner for EZ-Template drive PIDs.
///
/// Phase 1 measures the ultimate gain and period with a hysteresis relay,
/// averaged over several runs. Phase 2 converts them with Tyreus-Luyben rules
/// and bisects Kp against multiple move sizes until every size reaches its
/// target without exceeding the overshoot limit. Results are applied to the
/// chassis immediately and can be saved to the SD card.
///
/// Field safety: all translating tests clamp their targets to the configured
/// runway and cut the motors if the robot exceeds it. Motor temperature and
/// battery voltage are monitored throughout.
class PIDAutoTuner {
 public:
  explicit PIDAutoTuner(ez::Drive& chassis);

  TunerResult tune_drive(double test_distance_in = 12.0, int relay_amp = 60);
  TunerResult tune_turn(double test_angle_deg = 90.0, int relay_amp = 60);
  TunerResult tune_swing(double test_angle_deg = 90.0, int relay_amp = 60);

  /// Relay-tests the heading hold while driving straight. The refined Kp/Kd
  /// are capped against the drive-derived estimate to keep steady-state
  /// corrections from oscillating.
  TunerResult tune_heading(int forward_speed = 70, double target_deg = 6.0, int relay_amp = 20);

  /// Derives heading constants from the drive result without moving.
  /// The conservative alternative to tune_heading; call after tune_drive.
  void set_heading_from_drive();

  /// Straight-line clearance ahead of the robot's start position, in inches.
  /// Minimum 24. Default 60, which fits the largest stock refine target.
  void runway_set(double inches);

  bool save_to_sd(const char* path = "/usd/pid_constants.txt");

 private:
  ez::Drive& chassis;

  static constexpr int DT_MS = 10;
  static constexpr double DT_S = DT_MS / 1000.0;
  static constexpr int RELAY_RUNS = 3;
  static constexpr int SKIP_CROSSINGS = 4;
  static constexpr int MIN_CROSSINGS = 10;
  static constexpr int TEST_TIMEOUT_MS = 12000;
  static constexpr int COOL_DOWN_MS = 3000;
  static constexpr int MAX_REFINE_ITER = 5;
  static constexpr double DRIVE_OVERSHOOT_IN = 0.75;
  static constexpr double ANGLE_OVERSHOOT_DEG = 2.0;
  static constexpr int HEADING_TEST_TIMEOUT_MS = 4000;
  static constexpr double HEADING_MAX_DIST_IN = 48.0;
  static constexpr double HEADING_OVERSHOOT_DEG = 1.0;

  double runway_in_ = 60.0;

  TunerResult last_drive_;
  TunerResult last_turn_;
  TunerResult last_swing_;
  double last_heading_kp_ = 0;
  double last_heading_ki_ = 0;
  double last_heading_kd_ = 0;

  struct OscData {
    bool detected = false;
    double ku = 0;
    double pu_ms = 0;
  };

  enum class Motion { DRIVE, TURN, SWING, HEADING };

  struct MotionSpec {
    const char* name;
    const char* cap_name;
    const char* unit;
    bool use_imu;
    bool go_home;
    bool dist_abort;
    int probe_timeout_ms;
    int trial_timeout_ms;
    double settle_tol;
    double unreached_tol;
    double overshoot_max;
    double relay_hyst;
    double refine_scales[3];
    int n_refine;
  };
  static const MotionSpec& spec(Motion m);

  double read_pos();
  double read(Motion m);
  void reset_sensors(Motion m);
  void set_output(Motion m, double cmd, int forward_speed);

  OscData probe(Motion m, double target, int relay_amp, int forward_speed = 0);
  void return_drive(int speed);

  double motor_max_temp();
  void wait_for_temp(double max_c);

  double trial(Motion m, double target, int speed, double kp, double kd);
  double refine_kp(Motion m, double base_target, int speed, double kp_lo, double kp_hi, double kd);
  TunerResult tune(Motion m, double target, int relay_amp, int forward_speed = 0);

  static void tl_constants(double ku, double pu_ms, double& kp, double& ki, double& kd);
  static void print_result(const char* label, const TunerResult& r);
};

}  // namespace ez
