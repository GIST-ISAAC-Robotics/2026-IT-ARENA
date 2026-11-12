#include "steering_mapping.hpp"
#include "st3215_transaction.hpp"
// RAM 프레임 준비만. 출력 허가/실제 UART 송신을 포함하지 않는 대상 object 검사.
extern "C" bool prepare_measured_servo_goal(arena_servo::Transaction& wire,
    const arena_steering::Sample* samples,std::size_t size,std::int64_t now,
    double angle,unsigned speed,unsigned acceleration){
  const arena_steering::Mapping map(samples,size);
  const auto goal=map.goal(angle);
  return goal.valid && wire.goal(now,1,goal.position,speed,acceleration);
}
