#pragma once

// host-test stub for pros::Distance
namespace pros {
class Distance {
 public:
  explicit Distance(int port) : port_(port) {}
  int get() const { return mm; }
  int get_confidence() const { return confidence; }
  int get_port() const { return port_; }

  int mm = 9999;
  int confidence = 63;

 private:
  int port_;
};
}  // namespace pros
