#pragma once
// 공통 steering_rad → CAD 상대각 → 실장 offset. RAM 송신/ARM은 호출자 소유.
#include "cad_steering.hpp"
#include "steering_zero.hpp"

namespace arena_steering {
struct CadServoGoal {
  bool valid=false;
  CadGoal cad;
  ZeroGoal servo;
  const char* reason="calibration_pending";
};
inline CadServoGoal ResolveCadServoGoal(const CadMapping& map,const ZeroProfile& zero,
    double steering,double command_limit,double wheelbase,double track,double wheel_limit) {
  if(!std::isfinite(command_limit)||command_limit<=0 || !std::isfinite(steering) ||
     std::abs(steering)>command_limit)return {false,{}, {},"outside_command_contract"};
  if(!zero.valid()||!map.valid())return {};
  const auto a=map.central(-command_limit,wheelbase,track,wheel_limit);
  const auto b=map.central(command_limit,wheelbase,track,wheel_limit);
  if(!a.valid||!b.valid)return {false,{}, {},"invalid_geometry"};
  if(!zero.covers(std::min(a.servo_rad,b.servo_rad),std::max(a.servo_rad,b.servo_rad)))
    return {false,{}, {},"required_range_crosses_mounted_or_count_limit"};
  const auto cad=map.central(steering,wheelbase,track,wheel_limit);
  auto goal=zero.goal(cad.servo_rad);
  if(!goal.valid)return {false,cad,goal,"invalid_servo_goal"};
  // count 양자화까지 적용한 실제 목표가 표/내륜 상한을 넘으면 거부한다.
  auto quantized=map.relative(goal.relative_rad);
  if(quantized.clamped || std::max(std::abs(quantized.left_rad),std::abs(quantized.right_rad))>wheel_limit){
    goal.position += goal.position>static_cast<unsigned>(zero.zero) ? -1:1;
    goal.relative_rad=(static_cast<double>(goal.position)-zero.zero)*6.2831853071795864769/
      (zero.direction*static_cast<double>(zero.counts_per_turn));
    quantized=map.relative(goal.relative_rad);
  }
  if(goal.clamped || !quantized.valid || quantized.clamped ||
     std::max(std::abs(quantized.left_rad),std::abs(quantized.right_rad))>wheel_limit)
    return {false,cad,goal,"quantized_target_outside_geometry"};
  return {true,cad,goal,"cad_provisional_with_mounted_zero"};
}
}  // namespace arena_steering
