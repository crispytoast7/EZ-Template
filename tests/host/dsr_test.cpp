// host unit tests for ez::dsr, run on a computer with the stub sensor
#include "EZ-Template/dsr.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

using ez::dsr::Side;

static pros::Distance back_d(1), right_d(2), front_d(3);

static void reset() {
  ez::dsr::sensors_clear();
  back_d.mm = 9999; back_d.confidence = 63; back_d.mm_seq.clear();
  right_d.mm = 9999; right_d.confidence = 63; right_d.mm_seq.clear();
  front_d.mm = 9999; front_d.confidence = 63; front_d.mm_seq.clear();
}

static bool close_to(double a, double b, double tol = 0.15) { return std::fabs(a - b) < tol; }

int main() {
  double F = ez::dsr::tuning().field_in;
  int passed = 0;

  // back sensor at the near wall corrects y
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  back_d.mm = (int)((30.0 - 7.0) * 25.4);  // robot really at y=30
  double x = 50, y = 34;                   // odom thinks y=34
  assert(ez::dsr::correct(0.0, x, y));
  assert(close_to(y, 30.0) && close_to(x, 50.0));
  passed++;

  // right sensor facing the +x wall corrects x
  reset();
  ez::dsr::sensor_add(&right_d, Side::RIGHT, 6.0);
  right_d.mm = (int)((40.0 - 6.0) * 25.4);  // robot really at x = F-40
  x = F - 44; y = 60;
  assert(ez::dsr::correct(0.0, x, y));      // heading 0 -> right sensor faces +x
  assert(close_to(x, F - 40.0) && close_to(y, 60.0));
  passed++;

  // cosine correction: 10 deg off square still lands on the true distance
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  double true_perp = 25.0;
  double ray = (true_perp - 7.0) / std::cos(10.0 * M_PI / 180.0);
  back_d.mm = (int)(ray * 25.4);
  x = 50; y = 27;
  assert(ez::dsr::correct(10.0, x, y));
  assert(close_to(y, true_perp, 0.3));
  passed++;

  // too far off square is rejected
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  back_d.mm = 500;
  x = 50; y = 27;
  assert(!ez::dsr::correct(25.0, x, y));
  passed++;

  // low confidence is rejected
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  back_d.mm = 500; back_d.confidence = 10;
  assert(!ez::dsr::correct(0.0, x, y));
  passed++;

  // implied jump too big (something in the way) is rejected
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  back_d.mm = (int)((30.0 - 7.0) * 25.4);
  x = 50; y = 70;  // 40 inch jump
  assert(!ez::dsr::correct(0.0, x, y));
  passed++;

  // two sensors near a corner fix both axes in one call
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  ez::dsr::sensor_add(&right_d, Side::RIGHT, 6.0);
  back_d.mm = (int)((20.0 - 7.0) * 25.4);
  right_d.mm = (int)((18.0 - 6.0) * 25.4);
  x = F - 21; y = 23;
  assert(ez::dsr::correct(0.0, x, y));
  assert(close_to(y, 20.0) && close_to(x, F - 18.0));
  passed++;

  // align_heading picks the cheapest sensor for the nearest wall
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  ez::dsr::sensor_add(&right_d, Side::RIGHT, 6.0);
  // near the left wall (x small): back sensor needs facing 90, right needs 180
  double a = ez::dsr::align_heading(10.0, 70.0, 100.0);
  assert(close_to(a, 90.0, 0.01));
  double b = ez::dsr::align_heading(10.0, 70.0, 170.0);
  assert(close_to(b, 180.0, 0.01));
  passed++;

  // no sensors: heading unchanged
  reset();
  assert(close_to(ez::dsr::align_heading(10, 10, 42.0), 42.0, 0.001));
  passed++;

  // one wild sample in the stream is ignored by the median
  reset();
  ez::dsr::sensor_add(&back_d, Side::BACK, 7.0);
  int good = (int)((30.0 - 7.0) * 25.4);
  back_d.mm_seq = {good, good, 9999, good, good};  // spike in the middle
  x = 50; y = 33;
  assert(ez::dsr::correct(0.0, x, y));
  assert(close_to(y, 30.0));
  passed++;

  printf("dsr: %d/10 tests passed\n", passed);
  return 0;
}
