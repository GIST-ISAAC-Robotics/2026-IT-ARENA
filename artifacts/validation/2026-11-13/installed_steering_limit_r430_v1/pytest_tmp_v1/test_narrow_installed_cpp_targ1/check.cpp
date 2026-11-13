#include "board_profile.hpp"
#include "actuation_motor_endpoint.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
int main(){
const arena_steering::CadMapping map(arena_steering::kCadSamples,sizeof(arena_steering::kCadSamples)/sizeof(arena_steering::kCadSamples[0]));
auto p=installed_board_profile();
assert(p.valid(map));

const std::uint32_t boot[]={1}; arena_actuation::TxConfig tc;
arena_actuation::ActuationMotorEndpoint endpoint(p.endpoint_receiver_config(map),boot,1,30000,
  arena_actuation::PayloadLayout::kEnvelope,tc,p.kp,p.ki,.05,0.,p.maximum_duty);
endpoint.connect(0); assert(!endpoint.receiver().armed());
arena_servo::RuntimeServo servo(map,p.servo); assert(!servo.engaged());

std::cout << std::setprecision(17);
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,-0.15,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,-0.075,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,0.0,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,0.075,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,0.15,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,-0.01760284015233917,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
{ const auto g=arena_steering::ResolveCadServoGoal(map,p.servo.zero,-0.017606193640169045,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit); assert(g.valid); const auto actual=map.relative(g.servo.relative_rad); std::cout << g.servo.position << " " << g.servo.relative_rad << " " << actual.left_rad << " " << actual.right_rad << "\n"; }
assert(!arena_steering::ResolveCadServoGoal(map,p.servo.zero,.15001,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit).valid);
p.servo.zero.minimum=1999; p.servo.zero.maximum=2001; assert(!p.valid(map));
assert(!arena_steering::ResolveCadServoGoal(map,p.servo.zero,0.,p.servo.command_limit,p.servo.wheelbase,p.servo.track,p.servo.wheel_limit).valid);
}
