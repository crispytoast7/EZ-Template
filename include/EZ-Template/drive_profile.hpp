#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/misc.hpp"

namespace ez {

/// One side's measured feedforward description.
///
/// `ks` is the command a stopped side needs before its wheels actually turn,
/// in the same -127 to 127 units the drive is commanded in. `kv` is command
/// per rpm of steady-state motor velocity: the side's inverse gain, used only
/// as a ratio against the other side, so a rough value is still useful.
struct DriveProfile {
  double ks = 0.0;
  double kv = 0.0;
  bool valid() const { return kv > 0.0 && ks >= 0.0 && ks < 127.0; }
};

/// Both sides of a characterized drive.
struct DriveProfilePair {
  DriveProfile left;
  DriveProfile right;
  bool valid() const { return left.valid() && right.valid(); }
};

/// Result of fitting one side's command / velocity table.
struct DriveProfileFit {
  double ks = 0.0;
  double kv = 0.0;
  double r2 = 0.0;  ///< fit quality over the moving points, 1.0 is perfect
  int points = 0;   ///< how many samples actually moved
  bool ok = false;
};

/// Mutable access to the loaded profile, the way ez::dsr::tuning() works.
/// pid_constants_load() fills this from the SD card; nothing reads it unless
/// drive_profile_enable(true) has been called.
DriveProfilePair& drive_profile();

/// Turns the output compensation on or off. OFF by default, and a no-op
/// unless drive_profile().valid() — with no profile loaded the drive is
/// commanded exactly as it was before.
///
/// Compensation changes the plant: the same PID output now produces a
/// different amount of wheel motion, so RE-TUNE your PID constants after
/// enabling this (and after re-characterizing).
void drive_profile_enable(bool enable);
bool drive_profile_enabled();

/// Battery normalization, a separate opt-in on top of drive_profile_enable().
/// Scales the output by 12500 / battery mV so a half-charged battery drives
/// like a full one, until the output saturates. OFF by default.
void drive_profile_battery_enable(bool enable);
bool drive_profile_battery_enabled();

/// Interactive, wheels-off-the-ground characterization. Prompts on the
/// controller (A runs, B cancels), then steps each side on its own through
/// ascending command levels, holding each until the velocity settles, and
/// prints the command / rpm / battery table it measured. Velocity comes from
/// the motors themselves (get_actual_velocity averaged over the side's
/// non-PTO motors) because with the wheels in the air no tracking wheel or
/// drive encoder outside the motors turns at all.
///
/// The measured profile is applied to drive_profile() and saved to the SD
/// card, where pid_constants_load() restores it on every boot. Compensation
/// is left in whatever enabled state it was in; enabling it is your call, and
/// it invalidates your PID constants (see drive_profile_enable). Runs for
/// roughly 40 seconds. Returns the measured pair; check valid().
DriveProfilePair drive_characterize(ez::Drive& chassis, pros::Controller& controller,
                                    const char* path = "/usd/pid_constants.txt");

/// Adds "Drive Characterize" to the auton selector so the test can be run
/// like any other routine. Call it in initialize() after your own
/// autons_add() — the entry is appended, nothing already registered is lost.
void drive_characterize_register(ez::Drive& chassis);

/////
//
// The math, kept pure and separate so it can be reasoned about (and tested)
// without a robot attached.
//
/////

/// Least-squares fit of one side's table. `commands` and `rpms` are `count`
/// paired samples; anything slower than `moving_rpm` is treated as stalled
/// and left out of the fit.
///
/// kv is the slope of command against rpm. ks is the larger of the lowest
/// command that actually moved and the fit's extrapolation back to zero rpm,
/// taking the higher of the two so compensation errs toward pushing hard
/// enough to break static friction. With a single moving sample there is no
/// slope to fit, so kv falls back to command / rpm and r2 to 0. With none at
/// all the fit is not ok and both terms stay 0.
DriveProfileFit drive_profile_fit(const double* commands, const double* rpms, int count,
                                  double moving_rpm = 3.0);

/// Maps a command past the side's deadband: 0 stays 0, and everything else is
/// remapped into [ks, 127] so a command of 1 just barely moves and 127 is
/// still 127. Sign preserving.
double drive_profile_deadband_map(double cmd, double ks);

/// How much this side has to be scaled down to match the other. Speed goes as
/// command / kv, so the side with the smaller kv is the faster one and is the
/// only one that gets scaled; the slower side always gets 1.0, which keeps
/// both sides inside 127 and leaves the headroom symmetric.
double drive_profile_balance_scale(double kv_this, double kv_other);

/// 12500 / battery mV, clamped to a sane range so a bad reading cannot
/// multiply the drive by something wild.
double drive_profile_battery_scale(double battery_mv);

/// One side's full transform, rounded to a command at the very end.
int drive_profile_apply(double cmd, const DriveProfile& side, double balance_scale,
                        double battery_scale);

/// Transforms a left/right command pair in place. Returns immediately when
/// compensation is off or no profile is loaded, so the drive's normal path
/// pays for one bool read and nothing else.
void drive_profile_transform(int& left, int& right);

}  // namespace ez
