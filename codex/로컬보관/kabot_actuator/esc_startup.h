#pragma once
#include <stdint.h>
namespace kabot {
class EscStartup {
 public:
  explicit EscStartup(uint16_t duration) : duration_(duration) {}
  void start(uint32_t now) { since_ = now; pending_ = true; }
  void cancel() { pending_ = false; }
  bool holding(uint32_t now) {
    if (pending_ && uint32_t(now - since_) >= duration_) pending_ = false;
    return pending_;
  }
 private:
  uint16_t duration_;
  uint32_t since_ = 0;
  bool pending_ = false;
};
}
