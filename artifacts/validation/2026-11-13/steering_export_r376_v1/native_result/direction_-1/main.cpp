
#include <iostream>
#include <iomanip>
#include <cstring>
#include "mounted.hpp"
#include "cad_servo_goal.hpp"
#include "cad_steering_table.hpp"
int main() {
  using namespace arena_steering;
  if(std::strcmp(kCadSha256,arena_steering_mount::kCadTableSha256))return 2;
  const auto zero=arena_steering_mount::Profile();
  const CadMapping map(kCadSamples,kCadSampleCount);
  const double pi=std::acos(-1.0);
  const double limit=std::min(std::abs(map.central(1,.128,.1239,pi/6).equivalent_rad),
                            std::abs(map.central(-1,.128,.1239,pi/6).equivalent_rad));
  if(!zero.valid() || !map.valid())return 3;
  std::cout<<std::setprecision(17);
  for(int i=-100;i<=100;++i) {
    const double angle=limit*i/100;
    const auto goal=ResolveCadServoGoal(map,zero,angle,limit,.128,.1239,pi/6);
    if(!goal.valid)return 4;
    const auto actual=map.relative(goal.servo.relative_rad);
    std::cout<<i<<' '<<goal.servo.position<<' '<<actual.left_rad<<' '
             <<actual.right_rad<<' '<<actual.equivalent_rad<<'\n';
  }
  if(ResolveCadServoGoal(map,zero,limit+.001,limit,.128,.1239,pi/6).valid)return 5;
  return 0;
}
