#pragma once
// 하나의 UART 소유 작업. IA 계약 프레임과 원래 68 B IMU1을 프레임 경계에서만 교대한다.
#include "actuation_bench.hpp"

namespace arena_actuation {
class SensorLinkTx {
 public:
  static constexpr std::size_t kImuBytes = 68, kCapacity = 64;
  bool push(const void* data, std::size_t size) {
    if (size != kImuBytes || data == nullptr || count_ == kCapacity) {
      ++dropped_; return false;
    }
    std::memcpy(queue_[(head_ + count_) % kCapacity].data(), data, size);
    ++count_; return true;
  }
  template<class Sink>
  bool pump(std::int64_t now, ActuationBenchEndpoint& endpoint, Sink&& sink) {
    if (fault_) return false;
    // 이미 시작한 IMU 프레임은 끝까지 전송. IA의 중간에도 IMU 바이트를 끼우지 않는다.
    if (offset_ == 0) {
      endpoint.pump(now, sink);
      if (endpoint.tx().fault() != TxFault::kNone) { fault_ = true; return false; }
      if (endpoint.tx().frame_in_flight()) return true;
    }
    if (count_ == 0) return true;
    const auto remaining = kImuBytes - offset_;
    const auto accepted = sink(queue_[head_].data() + offset_, remaining);
    if (accepted > remaining) { fault_ = true; return false; }
    offset_ += accepted;
    if (offset_ == kImuBytes) {
      head_ = (head_ + 1) % kCapacity; --count_; offset_ = 0; ++completed_;
    }
    return true;
  }
  std::uint64_t dropped() const { return dropped_; }
  std::uint64_t completed() const { return completed_; }
  std::size_t queued() const { return count_; }
 private:
  std::array<std::array<std::uint8_t, kImuBytes>, kCapacity> queue_{};
  std::size_t head_ = 0, count_ = 0, offset_ = 0;
  std::uint64_t dropped_ = 0, completed_ = 0;
  bool fault_ = false;
};
}
