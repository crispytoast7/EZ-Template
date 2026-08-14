#include "EZ-Template/json_auton.hpp"

#include "EZ-Template/api.hpp"
#include "EZ-Template/dsr.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ez {

// ---------------------------------------------------------------------------
// Minimal JSON reader for the planner's export format (objects, arrays,
// strings, numbers, booleans). Unknown keys are skipped.
// ---------------------------------------------------------------------------
namespace {

struct Scanner {
  const char* p;

  void skip_ws() {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',' || *p == ':') p++;
  }

  bool expect(char c) {
    skip_ws();
    if (*p != c) return false;
    p++;
    return true;
  }

  bool peek(char c) {
    skip_ws();
    return *p == c;
  }

  bool parse_string(std::string& out) {
    skip_ws();
    if (*p != '"') return false;
    p++;
    out.clear();
    while (*p && *p != '"') {
      if (*p == '\\' && p[1]) p++;
      out += *p++;
    }
    if (*p != '"') return false;
    p++;
    return true;
  }

  bool parse_number(double& out) {
    skip_ws();
    char* end = nullptr;
    out = strtod(p, &end);
    if (end == p) return false;
    p = end;
    return true;
  }

  bool parse_bool(bool& out) {
    skip_ws();
    if (strncmp(p, "true", 4) == 0) { out = true; p += 4; return true; }
    if (strncmp(p, "false", 5) == 0) { out = false; p += 5; return true; }
    return false;
  }

  void skip_value() {
    skip_ws();
    if (*p == '"') { std::string s; parse_string(s); return; }
    if (*p == 't' || *p == 'f') { bool b; parse_bool(b); return; }
    if (*p == '{' || *p == '[') {
      char open = *p, close = (open == '{') ? '}' : ']';
      int depth = 0;
      do {
        if (*p == open) depth++;
        else if (*p == close) depth--;
        else if (*p == '"') { std::string s; parse_string(s); continue; }
        p++;
      } while (*p && depth > 0);
      return;
    }
    double d;
    parse_number(d);
  }
};

bool parse_point(Scanner& sc, JsonPoint& pt) {
  if (!sc.expect('{')) return false;
  while (!sc.peek('}')) {
    std::string key;
    if (!sc.parse_string(key)) return false;

    if      (key == "x")        { if (!sc.parse_number(pt.x)) return false; }
    else if (key == "y")        { if (!sc.parse_number(pt.y)) return false; }
    else if (key == "speed")    { double d; if (!sc.parse_number(d)) return false; pt.speed = (int)d; }
    else if (key == "wait_ms")  { double d; if (!sc.parse_number(d)) return false; pt.wait_ms = (int)d; }
    else if (key == "reversed") { if (!sc.parse_bool(pt.reversed)) return false; }
    else if (key == "smooth")   { if (!sc.parse_bool(pt.smooth)) return false; }
    else if (key == "face")     { if (!sc.parse_number(pt.face_deg)) return false; }
    else if (key == "action")   { if (!sc.parse_string(pt.action)) return false; }
    else sc.skip_value();
  }
  return sc.expect('}');
}

bool parse_auton(Scanner& sc, JsonPath& out) {
  if (!sc.expect('{')) return false;
  while (!sc.peek('}')) {
    std::string key;
    if (!sc.parse_string(key)) return false;

    if (key == "name") {
      if (!sc.parse_string(out.name)) return false;
    } else if (key == "start_heading_deg") {
      if (!sc.parse_number(out.start_heading_deg)) return false;
    } else if (key == "points") {
      if (!sc.expect('[')) return false;
      while (!sc.peek(']')) {
        JsonPoint pt;
        if (!parse_point(sc, pt)) return false;
        out.points.push_back(pt);
      }
      sc.expect(']');
    } else {
      sc.skip_value();
    }
  }
  return sc.expect('}');
}

bool read_file(const char* path, std::string& out) {
  FILE* f = fopen(path, "r");
  if (!f) return false;

  fseek(f, 0, SEEK_END);
  long len = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (len <= 0 || len > 256 * 1024) {
    printf("[auton] %s is empty or too large (%ld bytes)\n", path, len);
    fclose(f);
    return false;
  }

  out.assign(len, '\0');
  fread(&out[0], 1, len, f);
  fclose(f);
  return true;
}

std::function<void(const std::string&)> g_action_handler =
    [](const std::string& name) {
      printf("[auton] Unhandled action \"%s\"\n", name.c_str());
    };

}  // namespace

void json_action_handler_set(std::function<void(const std::string&)> handler) {
  g_action_handler = std::move(handler);
}

bool json_load(const char* path, std::vector<JsonPath>& out) {
  std::string text;
  if (!read_file(path, text)) return false;

  Scanner sc{text.c_str()};
  if (!sc.expect('{')) {
    printf("[auton] %s doesn't start with '{'\n", path);
    return false;
  }

  bool is_bundle = false;
  while (!sc.peek('}')) {
    std::string key;
    if (!sc.parse_string(key)) {
      printf("[auton] Parse error near: %.20s\n", sc.p);
      return false;
    }
    if (key == "autons") {
      is_bundle = true;
      if (!sc.expect('[')) return false;
      while (!sc.peek(']')) {
        JsonPath a;
        if (!parse_auton(sc, a)) {
          printf("[auton] Bad auton #%d in bundle\n", (int)out.size() + 1);
          return false;
        }
        if (a.points.size() >= 2) out.push_back(a);
        else printf("[auton] Skipping \"%s\": needs at least 2 points\n", a.name.c_str());
      }
      sc.expect(']');
    } else {
      sc.skip_value();
    }
  }

  if (!is_bundle) {
    JsonPath a;
    Scanner sc2{text.c_str()};
    if (parse_auton(sc2, a) && a.points.size() >= 2) out.push_back(a);
  }

  if (out.empty()) {
    printf("[auton] No usable autons in %s\n", path);
    return false;
  }
  for (auto& a : out)
    printf("[auton] Loaded \"%s\": %d points, start heading %.0f deg\n",
           a.name.c_str(), (int)a.points.size(), a.start_heading_deg);
  return true;
}

// ---------------------------------------------------------------------------
// Selector registration. Paths live in a static vector so the selector
// lambdas stay valid for the rest of the program.
// ---------------------------------------------------------------------------

static std::vector<JsonPath> g_sd_autons;

void json_register_selector(ez::Drive& chassis, const char* bundle_path, const char* fallback_path) {
  if (!json_load(bundle_path, g_sd_autons))
    json_load(fallback_path, g_sd_autons);

  if (g_sd_autons.empty()) {
    printf("[auton] No SD autons registered.\n");
    return;
  }

  std::vector<ez::Auton> entries;
  for (size_t i = 0; i < g_sd_autons.size(); i++) {
    ez::Drive* c = &chassis;
    entries.push_back({g_sd_autons[i].name,
                       [i, c]() { json_run(*c, g_sd_autons[i]); }});
  }
  ez::as::auton_selector.autons_add(entries);
  printf("[auton] Registered %d SD auton(s) in the selector.\n", (int)g_sd_autons.size());
}

// ---------------------------------------------------------------------------
// Executor. The chassis odom pose is aligned to the path's field frame at the
// start, so every target is computed from the live pose rather than the
// planned one. Runs of smooth points become one pure-pursuit motion.
// ---------------------------------------------------------------------------

namespace {

/// Live odom when available, dead-reckoned otherwise.
struct PoseEst {
  ez::Drive& chassis;
  bool use_odom;
  double ex, ey;
  double x() const { return use_odom ? chassis.odom_x_get() : ex; }
  double y() const { return use_odom ? chassis.odom_y_get() : ey; }
  void arrived_at(const JsonPoint& pt) { ex = pt.x; ey = pt.y; }
  void correct(double nx, double ny) {
    if (use_odom) {
      chassis.odom_x_set(nx);
      chassis.odom_y_set(ny);
    }
    ex = nx;
    ey = ny;
  }
};

void run_point_extras(ez::Drive& chassis, const JsonPoint& pt, PoseEst& pose) {
  if (!pt.action.empty()) {
    if (pt.action == "dsr") {
      double cx = pose.x(), cy = pose.y();
      double align = dsr::align_heading(cx, cy, chassis.drive_imu_get());
      if (std::fabs(align - chassis.drive_imu_get()) > 2.0) {
        printf("[auton] DSR: squaring to the wall at %.1f deg\n", align);
        chassis.pid_turn_set(align, 90);
        chassis.pid_wait();
        pros::delay(150);
      }
      if (dsr::correct(chassis.drive_imu_get(), cx, cy)) pose.correct(cx, cy);
    } else {
      printf("[auton] action \"%s\"\n", pt.action.c_str());
      g_action_handler(pt.action);
    }
  }
  if (pt.wait_ms > 0) {
    printf("[auton] wait %d ms\n", pt.wait_ms);
    pros::delay(pt.wait_ms);
  }
}

/// True when the robot should curve through the point without stopping.
/// An action, wait, or required arrival heading ends the smooth run there.
bool is_pass_through(const JsonPath& path, size_t i) {
  const JsonPoint& pt = path.points[i];
  return pt.smooth && pt.action.empty() && pt.wait_ms == 0 &&
         pt.face_deg >= 900.0 && i + 1 < path.points.size();
}

}  // namespace

void json_run(ez::Drive& chassis, const JsonPath& path) {
  const int TURN_SPEED = 90;

  chassis.odom_xyt_set(path.points[0].x, path.points[0].y, path.start_heading_deg);
  PoseEst pose{chassis, chassis.odom_enabled(), path.points[0].x, path.points[0].y};

  printf("[auton] Running \"%s\" from (%.1f, %.1f), odom %s\n",
         path.name.c_str(), pose.x(), pose.y(),
         pose.use_odom ? "on" : "OFF (dead reckoning)");

  run_point_extras(chassis, path.points[0], pose);

  size_t i = 1;
  while (i < path.points.size()) {
    const JsonPoint& pt = path.points[i];

    if (pt.smooth && pose.use_odom) {
      std::vector<ez::odom> moves;
      size_t j = i;
      while (true) {
        const JsonPoint& p = path.points[j];
        moves.push_back({{p.x, p.y}, p.reversed ? ez::rev : ez::fwd, p.speed});
        if (!is_pass_through(path, j)) break;
        j++;
      }
      // Boomerang: a set arrival heading is blended into the approach.
      if (path.points[j].face_deg < 900.0)
        moves.back().target.theta = path.points[j].face_deg;

      printf("[auton] #%d-#%d: pure pursuit through %d points\n",
             (int)i, (int)j, (int)moves.size());
      chassis.pid_odom_set(moves, true);
      chassis.pid_wait();

      pose.arrived_at(path.points[j]);
      run_point_extras(chassis, path.points[j], pose);
      i = j + 1;
      continue;
    }

    double dx = pt.x - pose.x();
    double dy = pt.y - pose.y();
    double dist = std::hypot(dx, dy);

    if (dist > 0.25) {
      double heading = std::atan2(dx, dy) * 180.0 / M_PI;
      if (pt.reversed) heading += 180.0;

      printf("[auton] #%d: turn to %.1f deg, drive %s%.1f in @ %d\n",
             (int)i, heading, pt.reversed ? "-" : "", dist, pt.speed);

      chassis.pid_turn_set(heading, TURN_SPEED);
      chassis.pid_wait();

      chassis.pid_drive_set(pt.reversed ? -dist : dist, pt.speed, true);
      chassis.pid_wait();

      pose.arrived_at(pt);
    }

    if (pt.face_deg < 900.0 && !pt.smooth) {
      chassis.pid_turn_set(pt.face_deg, TURN_SPEED);
      chassis.pid_wait();
    }

    run_point_extras(chassis, pt, pose);
    i++;
  }

  printf("[auton] Done at (%.1f, %.1f).\n", pose.x(), pose.y());
}

}  // namespace ez
