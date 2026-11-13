#pragma once
// 전진 전용 MD20A 명령 계산. GPIO/PWM, 수신기 상태 전이, HAL 허가를 구현하지 않는다.
#include "motor_control_candidate.hpp"

namespace arena_actuation {

// 장착 교정 프로필에서 명시적으로 전달한다. 시험 duty를 운영 기본값으로 채택하지 않는다.
struct Md20aForwardProfile {
  double maximum_duty;
  bool forward_dir_high;
  bool valid() const noexcept {
    return std::isfinite(maximum_duty) && maximum_duty > 0.0 && maximum_duty <= 1.0;
  }
};

enum class Md20aMapStatus { kActive, kInhibited, kInvalidProfile, kInvalidIntent };

struct Md20aForwardCommand {
  double duty_ratio;
  bool dir_high;
  Md20aMapStatus status;
};

// 같은 소유 작업의 control_tick 직후 호출한다. preview 저장/재사용을 허용하는 API가 아니다.
// output_ready 하나로 허가하지 않는다. 기존 receiver가 검증한 상태와 같은 주기 결과가 필요하다.
// kActive + duty 0은 정상 overspeed 감속이며 STOP/FAULT 또는 역토크 명령이 아니다.
inline Md20aForwardCommand md20a_forward_command(
    const MotorCyclePreview& cycle, const ActuationReceiverCore& receiver,
    const Md20aForwardProfile& profile, bool output_ready = false) noexcept {
  Md20aForwardCommand out{0.0, profile.forward_dir_high, Md20aMapStatus::kInhibited};
  if (!profile.valid()) { out.status = Md20aMapStatus::kInvalidProfile; return out; }
  if (!output_ready || receiver.state() != ReceiverState::kActive || !receiver.armed() ||
      !receiver.has_owner() || !(receiver.target_speed() > 0.0) ||
      cycle.state != receiver.state() || cycle.generation != receiver.generation() ||
      !receiver.has_now() || cycle.now_us != receiver.now()) return out;
  if (cycle.motor.status != PiStatus::kNormal || !std::isfinite(cycle.motor.effort) ||
      cycle.motor.effort < 0.0 || cycle.motor.effort > profile.maximum_duty ||
      cycle.motor.brake_requested) {
    out.status = Md20aMapStatus::kInvalidIntent;
    return out;
  }
  out.duty_ratio = cycle.motor.effort;
  out.status = Md20aMapStatus::kActive;
  return out;
}

}  // namespace arena_actuation
