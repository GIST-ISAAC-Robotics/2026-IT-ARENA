#pragma once
#include "mounted_steering_zero.hpp"
// Explicit installed values; build only, no automatic ARM.
namespace arena_runtime_mount {
inline RuntimeBoardProfile Profile() {
  RuntimeBoardProfile p;
  p.encoder_direction=-1;
  p.motor_direction_confirmed=true;
  p.forward_dir_high=true;
  p.maximum_duty=0.4;
  p.kp=0.6;
  p.ki=1.2;
  p.maximum_speed_mps=2.0;
  p.feedback_period_us=20000LL;
  p.servo.id=1u;
  p.servo.goal_speed=120u;
  p.servo.goal_acceleration=10u;
  p.servo.position_period_us=100000LL;
  p.servo.observation_max_age_us=200000LL;
  p.servo.command_limit=0.4;
  p.servo.wheelbase=0.128;
  p.servo.track=0.1239;
  p.servo.wheel_limit=0.5235987755982988;
  p.servo.zero=arena_steering_mount::Profile();
  return p;
}
}  // namespace arena_runtime_mount
 