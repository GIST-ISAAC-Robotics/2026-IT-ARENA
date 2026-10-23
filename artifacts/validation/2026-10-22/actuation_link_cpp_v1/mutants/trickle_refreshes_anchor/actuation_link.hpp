#pragma once
// PC 검증용 링크 후보: 부분 프레임 절대 조립 기한 + 연결/수신 오류 정책.
// 바이트 경계만 다룬다. JSON/명령 검증, 구동 허가(CLEAR/ARM), 제어 watchdog, 물리 STOP은 포함하지 않는다.
// 시간값은 운영 기본값이 아니다. 실제 baud·최대 프레임·USB/스케줄 지연 실측 뒤 별도로 정한다.
#include <cstddef>
#include <cstdint>

#include "actuation_wire.hpp"

namespace arena_actuation {

enum class LinkState : std::uint8_t { kDisconnected = 0, kConnected = 1, kFaulted = 2 };
enum class LinkFault : std::uint8_t { kNone = 0, kInvalidConfig = 1, kClockRegressed = 2, kInputBudget = 3 };
enum class LinkResult : std::uint8_t {
  kOk = 0, kNotConnected = 1, kFaultLatched = 2, kFaultRaised = 3, kNotFaulted = 4,
};
// kExpiredDrain: 기한이 지난 부분 버퍼 안에 이미 완성돼 있던 CRC-valid 프레임. 늦게 전달된 것이므로
// 수신 측은 원래 발행/원천 시각으로 만료를 판단해야 하며 도착 시각으로 수명을 늘리면 안 된다.
enum class FrameOrigin : std::uint8_t { kLive = 0, kExpiredDrain = 1 };

// 시각 계약:
// - now는 호출자가 공급하는 단조 증가 부호 없는 64비트 정수 tick이다(단위는 호출자 정의, 예: us).
//   같은 값의 반복은 허용한다. 직전 관측값보다 작으면 kClockRegressed로 고정한다.
//   wrap 또는 32비트 카운터 확장 실패도 역행으로 취급한다(모듈러 산술을 하지 않는다).
// - 부분 버퍼가 비어 있지 않게 된 시각을 anchor로 둔다. 조각을 추가해도 anchor는 늦춰지지 않는다.
//   한 호출 뒤 이전 호출에서 온 바이트가 모두 소비됐을 때만 anchor=now로 갱신한다(남은 바이트가 모두
//   이번 호출에 도착했으므로 정확함). 그 밖의 재동기화에서는 이전 anchor를 유지한다(보수적으로 이른 기한).
// - 만료 조건은 now - anchor >= timeout. now >= anchor가 보장되므로 덧셈 overflow가 없다.
//   즉 anchor + timeout 시각에 도착한 바이트는 이전 부분 프레임에 붙지 않는다.
// - 만료 확인은 feed/poll 호출 때만 일어난다. 실제 폐기 지연은 호출 주기에 의존한다.
class FrameLink {
 public:
  // timeout 0은 구성 오류로 영구 고정한다(clear_fault로도 해제하지 않음).
  explicit FrameLink(std::uint64_t assembly_timeout_ticks) : timeout_(assembly_timeout_ticks) {
    if (timeout_ == 0) {
      state_ = LinkState::kFaulted;
      fault_ = last_fault_ = LinkFault::kInvalidConfig;
    }
  }

  // callback(payload, size, FrameOrigin)은 동기 호출이다. 포인터 보관과 이 객체 재진입을 금지한다.
  // callback 실행 시간은 이 코어가 제한하지 않는다.
  template<class Callback>
  LinkResult feed(std::uint64_t now, const std::uint8_t* data, std::size_t size, Callback&& callback) {
    if (state_ == LinkState::kFaulted) { ++rejected_calls_; return LinkResult::kFaultLatched; }
    if (!observe(now)) return LinkResult::kFaultRaised;
    if (state_ != LinkState::kConnected) { ++rejected_calls_; return LinkResult::kNotConnected; }
    if (size > kMaxInputPerCall || (size != 0 && data == nullptr)) {
      // 상위 수신 작업의 호출량 계약 위반. 같은 호출의 바이트는 하나도 처리하지 않는다.
      latch(LinkFault::kInputBudget);
      return LinkResult::kFaultRaised;
    }
    expire(now, callback);
    const bool accepted = decoder_.feed(data, size, [&](const std::uint8_t* payload, std::size_t n) {
      callback(payload, n, FrameOrigin::kLive);
    });
    if (!accepted) {  // 위 검사로 도달하지 않아야 한다. 도달하면 잘라 받지 않고 고정한다.
      latch(LinkFault::kInputBudget);
      return LinkResult::kFaultRaised;
    }
    const std::size_t remaining = decoder_.buffered_bytes();
    if (remaining == 0) {
      has_anchor_ = false;
    } else {
      anchor_ = now;  // FIFO이므로 남은 바이트가 모두 이번 호출에 도착했다.
      has_anchor_ = true;
    }  // 그 밖에는 이전 호출의 바이트가 남아 있으므로 anchor를 유지한다(버퍼 존재 <=> anchor 존재).
    return LinkResult::kOk;
  }

  // 무수신 상태에서도 독립 작업이 호출해 기한을 확인한다. 구동 watchdog을 대신하지 않는다.
  template<class Callback>
  LinkResult poll(std::uint64_t now, Callback&& callback) {
    if (state_ == LinkState::kFaulted) { ++rejected_calls_; return LinkResult::kFaultLatched; }
    if (!observe(now)) return LinkResult::kFaultRaised;
    if (state_ != LinkState::kConnected) return LinkResult::kNotConnected;
    expire(now, callback);
    return LinkResult::kOk;
  }

  // 연결 중 다시 connect하면 놓친 단절로 보고 세션 경계를 센다. 이전 부분 바이트는 전달하지 않고 폐기한다.
  LinkResult connect(std::uint64_t now) {
    if (state_ == LinkState::kFaulted) { ++rejected_calls_; return LinkResult::kFaultLatched; }
    if (!observe(now)) return LinkResult::kFaultRaised;
    if (state_ == LinkState::kConnected) ++session_breaks_;
    drop_partial();
    state_ = LinkState::kConnected;
    ++sessions_;
    return LinkResult::kOk;
  }

  LinkResult disconnect(std::uint64_t now) {
    if (state_ == LinkState::kFaulted) { ++rejected_calls_; return LinkResult::kFaultLatched; }
    if (!observe(now)) return LinkResult::kFaultRaised;
    if (state_ != LinkState::kConnected) return LinkResult::kNotConnected;
    ++session_breaks_;
    drop_partial();
    state_ = LinkState::kDisconnected;
    return LinkResult::kOk;
  }

  // 명시적 복구. 연결/구동 허가를 만들지 않으며 kDisconnected로만 돌아간다. 새 시계 기준을 now로 둔다.
  LinkResult clear_fault(std::uint64_t now) {
    if (state_ != LinkState::kFaulted) return LinkResult::kNotFaulted;
    if (fault_ == LinkFault::kInvalidConfig) { ++rejected_calls_; return LinkResult::kFaultLatched; }
    fault_ = LinkFault::kNone;
    state_ = LinkState::kDisconnected;
    last_now_ = now;
    has_time_ = true;
    return LinkResult::kOk;
  }

  LinkState state() const { return state_; }
  LinkFault fault() const { return fault_; }
  LinkFault last_fault() const { return last_fault_; }
  bool has_pending() const { return decoder_.buffered_bytes() != 0; }
  bool has_anchor() const { return has_anchor_; }
  std::uint64_t pending_since() const { return anchor_; }
  std::uint64_t assembly_timeout() const { return timeout_; }
  std::size_t buffered_bytes() const { return decoder_.buffered_bytes(); }
  const std::uint8_t* buffered_data() const { return decoder_.buffered_data(); }
  std::uint64_t framing_errors() const { return decoder_.errors(); }
  std::uint64_t assembly_timeouts() const { return timeouts_; }
  std::uint64_t rejected_calls() const { return rejected_calls_; }
  std::uint64_t dropped_bytes() const { return dropped_bytes_; }
  // 연결된 세션의 종료(단절·놓친 단절)와 모든 고장 고정을 센다. 구동 정책은 증가를 세션 경계로 처리한다.
  std::uint64_t session_breaks() const { return session_breaks_; }
  std::uint64_t sessions() const { return sessions_; }

 private:
  bool observe(std::uint64_t now) {
    if (has_time_ && now < last_now_) {
      latch(LinkFault::kClockRegressed);
      return false;
    }
    last_now_ = now;
    has_time_ = true;
    return true;
  }

  template<class Callback>
  void expire(std::uint64_t now, Callback& callback) {
    if (!has_anchor_ || now - anchor_ < timeout_) return;
    ++timeouts_;
    // 기한이 지난 머리를 손상으로 보고 남은 바이트를 CRC 실패와 같은 규칙으로 재검사한다.
    // 이미 완성돼 있던 프레임만 전달되고, 새 바이트와 결합할 부분 바이트는 남기지 않는다.
    while (decoder_.buffered_bytes() != 0) {
      decoder_.reject_head([&](const std::uint8_t* payload, std::size_t n) {
        callback(payload, n, FrameOrigin::kExpiredDrain);
      });
    }
    has_anchor_ = false;
  }

  void drop_partial() {
    dropped_bytes_ += decoder_.buffered_bytes();
    decoder_.reset();
    has_anchor_ = false;
  }

  void latch(LinkFault fault) {
    drop_partial();
    ++session_breaks_;
    state_ = LinkState::kFaulted;
    fault_ = fault;
    last_fault_ = fault;
  }

  FrameDecoder decoder_;
  std::uint64_t timeout_;
  std::uint64_t anchor_ = 0;
  std::uint64_t last_now_ = 0;
  std::uint64_t timeouts_ = 0;
  std::uint64_t rejected_calls_ = 0;
  std::uint64_t dropped_bytes_ = 0;
  std::uint64_t session_breaks_ = 0;
  std::uint64_t sessions_ = 0;
  LinkState state_ = LinkState::kDisconnected;
  LinkFault fault_ = LinkFault::kNone;
  LinkFault last_fault_ = LinkFault::kNone;
  bool has_anchor_ = false;
  bool has_time_ = false;
};

}  // namespace arena_actuation
