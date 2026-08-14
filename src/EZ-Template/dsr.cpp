#include "EZ-Template/dsr.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace ez {
namespace dsr {

namespace {

std::vector<SensorInfo> g_sensors;
Tuning g_tuning;

const char* side_name(Side s) {
  switch (s) {
    case Side::FRONT: return "front";
    case Side::RIGHT: return "right";
    case Side::BACK:  return "back";
    default:          return "left";
  }
}

/// Bearing of the sensor's pointing direction relative to the robot's front.
double side_bearing(Side s) {
  switch (s) {
    case Side::FRONT: return 0.0;
    case Side::RIGHT: return 90.0;
    case Side::BACK:  return 180.0;
    default:          return 270.0;
  }
}

double wrap_deg(double a) {
  while (a >= 360.0) a -= 360.0;
  while (a < 0.0) a += 360.0;
  return a;
}

double wrap180(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

bool correct_from(const SensorInfo& s, double field_heading_deg, double& x, double& y) {
  int mm = s.device->get();
  int conf = s.device->get_confidence();

  if (mm < g_tuning.min_mm || mm > g_tuning.max_mm) {
    printf("[dsr] %s: skipped, reading %d mm out of trusted range\n", side_name(s.side), mm);
    return false;
  }
  if (conf < g_tuning.min_confidence) {
    printf("[dsr] %s: skipped, confidence %d/63 too low\n", side_name(s.side), conf);
    return false;
  }

  double sensor_heading = wrap_deg(field_heading_deg + side_bearing(s.side));

  // Nearest cardinal decides the wall: 0 = far (+y), 1 = right (+x),
  // 2 = near (-y), 3 = left (-x).
  int cardinal = (int)std::lround(sensor_heading / 90.0) % 4;
  double misalign = sensor_heading - cardinal * 90.0;
  if (misalign > 180.0) misalign -= 360.0;
  if (std::fabs(misalign) > g_tuning.max_misalign_deg) {
    printf("[dsr] %s: skipped, %.1f deg off square\n", side_name(s.side), misalign);
    return false;
  }

  // Perpendicular distance from robot center to the wall. The cosine removes
  // the extra length of the off-square ray.
  double d = (mm / 25.4) * std::cos(misalign * M_PI / 180.0) + s.offset_in;

  double nx = x, ny = y;
  switch (cardinal) {
    case 0: ny = g_tuning.field_in - d; break;
    case 1: nx = g_tuning.field_in - d; break;
    case 2: ny = d; break;
    case 3: nx = d; break;
  }

  double jump = std::fabs(nx - x) + std::fabs(ny - y);
  if (jump > g_tuning.max_correction_in) {
    printf("[dsr] %s: skipped, implied jump %.1f in (obstructed?)\n", side_name(s.side), jump);
    return false;
  }

  printf("[dsr] %s: reset (%.1f, %.1f) -> (%.1f, %.1f)  [%d mm, conf %d, %.1f deg off]\n",
         side_name(s.side), x, y, nx, ny, mm, conf, misalign);
  x = nx;
  y = ny;
  return true;
}

}  // namespace

Tuning& tuning() { return g_tuning; }

void sensor_add(pros::Distance* device, Side side, double offset_in) {
  for (auto& e : g_sensors) {
    if (e.side == side) {
      e = {device, side, offset_in};
      return;
    }
  }
  g_sensors.push_back({device, side, offset_in});
}

void sensors_clear() { g_sensors.clear(); }

const std::vector<SensorInfo>& sensors() { return g_sensors; }

bool correct(double field_heading_deg, double& x, double& y) {
  bool any = false;
  for (const auto& s : g_sensors)
    any = correct_from(s, field_heading_deg, x, y) || any;
  return any;
}

double align_heading(double x, double y, double current_deg) {
  if (g_sensors.empty()) return current_deg;

  struct { double dist, bearing; } walls[4] = {
      {g_tuning.field_in - y, 0.0},
      {g_tuning.field_in - x, 90.0},
      {y, 180.0},
      {x, 270.0}};
  int nearest = 0;
  for (int i = 1; i < 4; i++)
    if (walls[i].dist < walls[nearest].dist) nearest = i;

  // Candidate facings: one per registered sensor, chosen so that sensor
  // points square at the nearest wall. Pick the smallest rotation.
  double best = current_deg;
  double best_turn = 1e9;
  for (const auto& s : g_sensors) {
    double face = walls[nearest].bearing - side_bearing(s.side);
    double turn = std::fabs(wrap180(face - current_deg));
    if (turn < best_turn) {
      best_turn = turn;
      best = current_deg + wrap180(face - current_deg);
    }
  }
  return best;
}

}  // namespace dsr
}  // namespace ez
