#include <cassert>
#include <cstdio>
#include <fstream>

static void write_file(const char* path, const char* text) {
  std::ofstream f(path);
  f << text;
}

int main() {
  int passed = 0;

  // planner-style bundle: extra keys (mode, toggles, auto, face) must not break parsing
  write_file("/tmp/autons.json", R"({
    "autons": [
      {
        "name": "left_awp",
        "start_heading_deg": 90,
        "mode": "h2h",
        "toggles": {"top": "y", "bottom": "r", "left": "b", "right": "y"},
        "points": [
          {"x": 24, "y": 12, "speed": 110},
          {"x": 30.5, "y": 40, "speed": 90, "smooth": true, "auto": true},
          {"x": 60, "y": 60, "speed": 127, "smooth": true, "face": 45, "action": "intake_on"},
          {"x": 24, "y": 100, "reversed": true, "wait_ms": 500, "action": "dsr"}
        ]
      },
      {"name": "too_short", "points": [{"x": 1, "y": 1}]},
      {"name": "second", "start_heading_deg": -45,
       "points": [{"x": 10, "y": 10}, {"x": 50, "y": 50, "speed": 80}]}
    ]
  })");

  std::vector<ez::JsonPath> out;
  assert(ez::json_load("/tmp/autons.json", out));
  assert(out.size() == 2);  // too_short skipped
  auto& a = out[0];
  assert(a.name == "left_awp" && a.start_heading_deg == 90);
  assert(a.points.size() == 4);
  assert(a.points[1].smooth && a.points[1].speed == 90);
  assert(a.points[2].face_deg == 45 && a.points[2].action == "intake_on");
  assert(a.points[3].reversed && a.points[3].wait_ms == 500 && a.points[3].action == "dsr");
  assert(out[1].start_heading_deg == -45);
  passed++;

  // single-auton file (old format) still loads
  write_file("/tmp/auton.json", R"({
    "name": "solo", "start_heading_deg": 0,
    "points": [{"x": 0, "y": 0}, {"x": 24, "y": 0, "speed": 100}]
  })");
  out.clear();
  assert(ez::json_load("/tmp/auton.json", out));
  assert(out.size() == 1 && out[0].name == "solo" && out[0].points[1].x == 24);
  passed++;

  // garbage is rejected without crashing
  write_file("/tmp/bad.json", "this is not json at all {{{");
  out.clear();
  assert(!ez::json_load("/tmp/bad.json", out));
  passed++;

  // missing file
  out.clear();
  assert(!ez::json_load("/tmp/nope_does_not_exist.json", out));
  passed++;

  printf("json: %d/4 tests passed\n", passed);
  return 0;
}
