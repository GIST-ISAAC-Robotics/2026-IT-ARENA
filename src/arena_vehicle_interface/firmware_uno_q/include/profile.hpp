#pragma once
#include "actuation_receiver.hpp"
namespace unoq {
enum class Mode { Unconfigured, Bench, Installed };
struct Profile {
  Mode mode=Mode::Unconfigured;
  int encoder_direction=0;
  bool motor_direction_confirmed=false, forward_dir_high=false;
  double maximum_duty=0, kp=0, ki=0, maximum_speed_mps=0;
  double maximum_steering_rad=0.28475436568260193;
  std::int64_t feedback_period_us=20000;
  bool installed() const {
    return mode==Mode::Installed && motor_direction_confirmed &&
      (encoder_direction==1 || encoder_direction==-1) &&
      std::isfinite(maximum_duty) && maximum_duty>0 && maximum_duty<=1 &&
      std::isfinite(kp) && kp>0 && std::isfinite(ki) && ki>0 &&
      std::isfinite(maximum_speed_mps) && maximum_speed_mps>0 && maximum_speed_mps<=13./3.6 &&
      maximum_steering_rad>0 && maximum_steering_rad<=0.28475436568260193 &&
      feedback_period_us>=5000 && feedback_period_us<50000;
  }
  arena_actuation::ReceiverConfig config() const {
    arena_actuation::ReceiverConfig c;
    c.feedback_mode=arena_actuation::FeedbackMode::kMotorShaft;
    c.ticks_per_rev=28; c.gear_ratio=2.5*81./17.; c.wheel_radius_m=.0325;
    c.max_steering_rad=maximum_steering_rad;
    c.max_speed_mps=installed()?maximum_speed_mps:13./3.6;
    c.max_wheel_mps=13./3.6;
    return c;
  }
};
}
