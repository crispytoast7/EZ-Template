#pragma once

#include "pros/distance.hpp"
#include <vector>

namespace ez {
namespace dsr {

/// Which side of the robot a distance sensor points out of.
/// FRONT = the robot's forward direction, RIGHT = 90 deg clockwise from it, etc.
enum class Side { FRONT, RIGHT, BACK, LEFT };

/// Thresholds for accepting a reading. A rejected reading leaves the pose
/// untouched; every default errs toward keeping the current estimate.
struct Tuning {
  int min_confidence = 45;        ///< sensor confidence floor, 0-63
  int min_mm = 20;                ///< readings closer than this are unreliable
  int max_mm = 1800;              ///< readings farther than this are unreliable
  double max_misalign_deg = 15.0; ///< max angle off square to the wall
  double max_correction_in = 18.0;///< larger implied jumps are rejected
  double field_in = 3600.0 / 25.4;///< interior wall-to-wall span (6 x 600 mm tiles)
};

/// Mutable access to the acceptance thresholds.
Tuning& tuning();

/// Registers a sensor. `offset_in` is the distance from the robot's turning
/// center to the sensor face, measured along the direction the sensor points.
/// Register up to one sensor per side; a later add for the same side replaces
/// the earlier one.
void sensor_add(pros::Distance* device, Side side, double offset_in);

/// Removes all registered sensors.
void sensors_clear();

/// A registered sensor.
struct SensorInfo {
  pros::Distance* device;
  Side side;
  double offset_in;
};

/// Read-only access to the registered sensors (used by ez::health).
const std::vector<SensorInfo>& sensors();

/// Reads every registered sensor and overwrites x and/or y where a wall
/// reading passes all acceptance checks. `field_heading_deg` is the robot's
/// absolute heading (0 = +y, clockwise positive). Returns true if any axis
/// was corrected. Prints the reason whenever a sensor's reading is rejected.
bool correct(double field_heading_deg, double& x, double& y);

/// Heading that points some registered sensor square at the nearest wall with
/// the least rotation from `current_deg`. Returns `current_deg` unchanged if
/// no sensors are registered. The result is continuous with `current_deg`
/// (no wrap jump), suitable for ez::Drive::pid_turn_set.
double align_heading(double x, double y, double current_deg);

}  // namespace dsr
}  // namespace ez
