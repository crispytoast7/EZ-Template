#pragma once

#include "EZ-Template/drive/drive.hpp"
#include <functional>
#include <string>
#include <vector>

namespace ez {

/// One waypoint of a JSON autonomous routine.
///
/// Coordinate system: inches, origin at the field interior's bottom-left
/// corner, +x right, +y away from the driver. Heading 0 deg faces +y,
/// clockwise positive. The field interior spans 141.73 in (6 x 600 mm tiles).
struct JsonPoint {
  double x = 0.0;
  double y = 0.0;
  int speed = 110;
  bool reversed = false;    ///< drive to this point backwards
  bool smooth = false;      ///< pass through without stopping (requires odom)
  double face_deg = 999.0;  ///< arrival heading; 999 = unset. Smooth points
                            ///< boomerang to it; sharp points turn after arrival.
  int wait_ms = 0;
  std::string action;       ///< runs on arrival; "dsr" triggers a wall reset
};

struct JsonPath {
  std::string name;
  double start_heading_deg = 0.0;
  std::vector<JsonPoint> points;  ///< robot starts on points[0]
};

/// Parses a bundle ({"autons":[...]}) or a single-auton file. Returns false
/// and prints the reason if nothing usable was found. Autons with fewer than
/// two points are skipped.
bool json_load(const char* path, std::vector<JsonPath>& out);

/// Loads autons from the SD card and adds one auton-selector entry per auton.
/// Call from initialize() before ez::as::initialize(). Tries `bundle_path`
/// first, then `fallback_path`.
void json_register_selector(ez::Drive& chassis,
                            const char* bundle_path = "/usd/autons.json",
                            const char* fallback_path = "/usd/auton.json");

/// Runs one path. The chassis odom pose and IMU are aligned to the path's
/// field frame at the start; every target is computed from the live pose.
/// Runs of smooth points execute as one pure-pursuit motion; a point with a
/// face_deg ends its run with a boomerang at that heading. The built-in "dsr"
/// action squares the robot to the nearest wall (ez::dsr::align_heading),
/// reads the registered sensors, and corrects the pose.
void json_run(ez::Drive& chassis, const JsonPath& path);

/// Handler for point actions other than "dsr". The default prints a warning
/// for every action name.
void json_action_handler_set(std::function<void(const std::string&)> handler);

}  // namespace ez
