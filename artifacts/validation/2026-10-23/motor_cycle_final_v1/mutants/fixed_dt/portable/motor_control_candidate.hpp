#pragma once
// 출력 없는 제어 주기 결합 후보. GPIO/PWM/DIR/서보·독립 watchdog/HAL이 아니다.
// receiver/ingress/TX와 같은 소유 작업에서 호출하며 ISR/다른 task 재진입을 금지한다.
#include "actuation_receiver.hpp"
#include "speed_pi.hpp"

namespace arena_actuation {

struct MotorCyclePreview {
  MotorIntent motor{};
  double steering_rad = 0.0;
  std::int64_t now_us = 0;
  std::int64_t generation = 0;
  ReceiverState state = ReceiverState::kDisarmed;
  const char* reason = "not_started";
};

class MotorControlCandidate {
 public:
  // receiver가 더 오래 살아야 한다. 이 객체는 모터축 모드만 지원한다.
  explicit MotorControlCandidate(ActuationReceiverCore& receiver, double kp = 0.6,
                                 double ki = 1.2, double max_dt_s = 0.05)
      : receiver_(receiver), pi_(kp, ki, max_dt_s), max_dt_s_(max_dt_s) {}
  MotorControlCandidate(const MotorControlCandidate&) = delete;
  MotorControlCandidate& operator=(const MotorControlCandidate&) = delete;

  // 준비값은 이번 주기의 외부 증거다. 생략하면 false. true도 실물 허가/감시 검증은 아니다.
  // 반환값은 이 시각의 계산 의도이며, 저장해 두었다 나중에 적용하는 출력 패킷이 아니다.
  MotorCyclePreview tick(std::int64_t now, bool output_ready = false) {
    receiver_.tick(now);  // 명령·피드백·제어 공백 검사부터. 통신 callback에서 이 함수를 호출하지 않는다.
    if (!pi_.valid() || !receiver_.motor_mode())
      return fail(ControlFault::kInvalidConfig, now);
    if (now < 0 || now > kMaxNowUs || (primed_ && now < last_cycle_)) {
      reset();
      return preview(now, "invalid_clock");
    }
    const bool first = !primed_;
    const double dt = first ? 0.0 : 0.005;
    last_cycle_ = now;
    primed_ = true;
    if (generation_ != receiver_.generation()) {
      reset();
      generation_ = receiver_.generation();
    }
    if (!receiver_.armed()) {
      reset();
      return preview(now, receiver_.reason());
    }
    if (!output_ready) return fail(ControlFault::kOutputUnavailable, now);
    if (first || !(dt > 0.0 && dt <= max_dt_s_)) return fail(ControlFault::kInvalidPeriod, now);
    // 신뢰된 같은 receiver 객체가 ARM/DRIVE를 검증한다. 주기 사이 두 명령이 연속 도착해도
    // 세대 변경에서 적분을 버린다. 비신뢰 상태 사본을 읽는 AppliedGuard와 달리 중간 ARMED
    // 관측을 강제하지 않는다. 이미 ACTIVE인 상태에서 새 객체의 첫 tick은 위에서 거절한다.
    if (receiver_.state() == ReceiverState::kArmed) {
      pi_.reset();
      auto out = preview(now, "zero_target");
      out.motor.status = PiStatus::kZeroTarget;
      return out;
    }
    const auto intent = pi_.update_mean(receiver_.target_speed(), receiver_.mean_mps(), dt, true);
    if (intent.status != PiStatus::kNormal) {
      auto out = fail(ControlFault::kPiArithmetic, now);
      out.motor.status = intent.status;
      return out;
    }
    auto out = preview(now, "calculated_not_applied");
    out.motor = intent;
    return out;
  }

  double integral() const { return pi_.integral(); }

 private:
  void reset() { pi_.reset(); }
  MotorCyclePreview preview(std::int64_t now, const char* reason) const {
    return MotorCyclePreview{MotorIntent{}, receiver_.steering(), now, receiver_.generation(),
                             receiver_.state(), reason};
  }
  MotorCyclePreview fail(ControlFault reason, std::int64_t now) {
    receiver_.report_control_fault(reason, now);
    reset();
    return preview(now, receiver_.reason());
  }
  ActuationReceiverCore& receiver_;
  SpeedPI pi_;
  const double max_dt_s_;
  bool primed_ = false;
  std::int64_t last_cycle_ = 0;
  std::int64_t generation_ = -1;
};

}  // namespace arena_actuation
