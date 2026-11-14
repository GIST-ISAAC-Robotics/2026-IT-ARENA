// 기존 portable MCU 계산만 검사한다. HAL/board/포트/펌웨어 실행 없음.
#include "cad_steering_table.hpp"
#include "cad_servo_goal.hpp"
#include "../selected_v2/mounted_steering_zero.hpp"
#include <cassert>
#include <iostream>

int main() {
  arena_steering::CadMapping map(arena_steering::kCadSamples,arena_steering::kCadSampleCount);
  const auto zero=arena_steering_mount::Profile();
  constexpr double limit=0.28475436568260193;
  assert(zero.valid() && zero.zero==2811 && zero.minimum==2600 && zero.maximum==2980 && zero.direction==-1);
  unsigned previous=0;
  for(int i=0;i<=400;++i){
    const double command=limit*(i-200)/200.;
    auto result=arena_steering::ResolveCadServoGoal(map,zero,command,limit,.128,.1239,3.14159265358979323846/6);
    assert(result.valid && !result.servo.clamped);
    assert(result.servo.position>=2600 && result.servo.position<=2980);
    assert(result.servo.position>=previous);previous=result.servo.position;
    if(i==0)assert(result.servo.position==2643);
    if(i==200)assert(result.servo.position==2811);
    if(i==400)assert(result.servo.position==2980);
  }
  assert(zero.goal(-10).clamped && zero.goal(-10).position==2980);
  assert(zero.goal(10).clamped && zero.goal(10).position==2600);
  assert(!arena_steering::ResolveCadServoGoal(map,zero,limit+.001,limit,.128,.1239,.5235987755982988).valid);
  std::cout << "portable MCU 401 goals, direction, zero, saturation and contract rejection passed\n";
}
