#pragma once

// host-test stub for pros::Distance. Set mm/confidence for constant readings,
// or fill mm_seq to return a sequence (cycled).
#include <vector>
namespace pros {
class Distance {
 public:
  explicit Distance(int port) : port_(port) {}
  int get() const {
    if (!mm_seq.empty()) return mm_seq[idx_++ % mm_seq.size()];
    return mm;
  }
  int get_confidence() const { return confidence; }
  int get_port() const { return port_; }

  int mm = 9999;
  int confidence = 63;
  std::vector<int> mm_seq;

 private:
  int port_;
  mutable unsigned idx_ = 0;
};
}  // namespace pros
