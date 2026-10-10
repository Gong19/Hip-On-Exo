#pragma once
#include <algorithm>
#include <cstdint>
// Retain the frequency grid through short wakeup delays; never burst requests.
class MotorSchedule {
 public:
  MotorSchedule(uint64_t start, unsigned hz): next_(start), period_(1000000000ULL/hz) {}
  uint64_t due() const {return last_ ? std::max(next_,last_+750000) : next_;}
  uint64_t sent(uint64_t began,uint64_t completed) {
    uint64_t skipped=began>next_ ? (began-next_)/period_ : 0;
    next_+=(skipped+1)*period_;last_=completed;return skipped;
  }
 private:
  uint64_t next_,period_,last_=0;
};
