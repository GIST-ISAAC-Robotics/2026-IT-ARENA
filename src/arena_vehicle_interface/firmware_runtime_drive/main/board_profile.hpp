#pragma once
// 실장 확인 뒤 작성할 빌드 구성. 기본값은 송신/구동 허가가 없는 미교정 상태다.
#include "cad_steering_table.hpp"
#include "runtime_servo.hpp"
#include "md20a_forward_output.hpp"

struct RuntimeBoardProfile {
  // 부호/게인/duty는 실장 확인값이어야 한다. 과거 무부하5% 시험을 자동 채택하지 않는다.
  int encoder_direction=0;
  bool motor_direction_confirmed=false, forward_dir_high=false;
  double maximum_duty=0.0, kp=0.0, ki=0.0, maximum_speed_mps=0.0;
  // 선택형 encoder 타당성 경계. 생략 시 기존5m/s, 물리 과도 속도 보증이 아니다.
  double max_wheel_mps=5.0;
  std::int64_t feedback_period_us=20000;
  arena_servo::RuntimeServoConfig servo{};

  arena_actuation::ReceiverConfig receiver_config() const {
    arena_actuation::ReceiverConfig c;
    c.feedback_mode=arena_actuation::FeedbackMode::kMotorShaft;
    c.ticks_per_rev=28;c.gear_ratio=2.5*81.0/17.0;c.wheel_radius_m=.0325;
    c.max_speed_mps=maximum_speed_mps;c.max_steering_rad=servo.command_limit;
    c.max_wheel_mps=max_wheel_mps;
    return c;
  }
  arena_actuation::Md20aForwardProfile motor_profile() const {
    return {maximum_duty,forward_dir_high};
  }
  bool valid(const arena_steering::CadMapping& map) const {
    const auto left=map.central(servo.command_limit,servo.wheelbase,servo.track,servo.wheel_limit);
    const auto right=map.central(-servo.command_limit,servo.wheelbase,servo.track,servo.wheel_limit);
    return motor_direction_confirmed && (encoder_direction==1 || encoder_direction==-1) &&
      motor_profile().valid() && std::isfinite(kp) && kp>0 && std::isfinite(ki) && ki>0 &&
      feedback_period_us>=5000 && feedback_period_us<receiver_config().feedback_timeout_us &&
      receiver_config().valid() && servo.position_period_us>0 &&
      servo.observation_max_age_us>servo.position_period_us &&
      servo.id<=253 && servo.goal_speed>0 && servo.goal_speed<=400 &&
      servo.goal_acceleration>=5 && servo.goal_acceleration<=20 &&
      left.valid && !left.clamped && right.valid && !right.clamped &&
      arena_steering::ResolveCadServoGoal(map,servo.zero,0,servo.command_limit,
        servo.wheelbase,servo.track,servo.wheel_limit).valid;
  }
  arena_actuation::ReceiverConfig endpoint_receiver_config(const arena_steering::CadMapping& map) const {
    if(valid(map))return receiver_config();
    // 미교정 상태도 boot/진단을 송신할 수 있는 wire 구성이다. 실제 운용값의 대체/승인이 아니다.
    // 이 경로는 ready=false, 실제 속도 미주입이며 ARM을 허가하지 않는다.
    arena_actuation::ReceiverConfig c;
    c.feedback_mode=arena_actuation::FeedbackMode::kMotorShaft;
    c.ticks_per_rev=28;c.gear_ratio=2.5*81.0/17.0;c.wheel_radius_m=.0325;
    return c;
  }
};

#if defined(ARENA_RUNTIME_INSTALLED_PROFILE) && ARENA_RUNTIME_INSTALLED_PROFILE
#include "runtime_installed_profile.hpp"
#endif

inline RuntimeBoardProfile installed_board_profile() {
#if defined(ARENA_RUNTIME_INSTALLED_PROFILE) && ARENA_RUNTIME_INSTALLED_PROFILE
  return arena_runtime_mount::Profile();
#else
  RuntimeBoardProfile p;
  // r376 export-cpp의 실장 ZeroProfile과 실제 확인한 방향/상한/PI를 이 함수에 연결한다.
  // 유효 프로필을 넣으면 ARM과 실제 DRIVE에 따라 출력할 수 있으므로 무인 변경하지 않는다.
  return p;
#endif
}
