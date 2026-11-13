#include "board_profile.hpp"
#include "actuation_motor_endpoint.hpp"
#include <cassert>
int main(){
const arena_steering::CadMapping map(arena_steering::kCadSamples,sizeof(arena_steering::kCadSamples)/sizeof(arena_steering::kCadSamples[0]));
auto p=installed_board_profile();
assert(p.encoder_direction==-1);
assert(p.motor_direction_confirmed==true);
assert(p.forward_dir_high==true);
assert(p.maximum_duty==0.4);
assert(p.kp==0.6);
assert(p.ki==1.2);
assert(p.maximum_speed_mps==2.0);
assert(p.feedback_period_us==20000LL);
assert(p.servo.id==1u);
assert(p.servo.goal_speed==120u);
assert(p.servo.goal_acceleration==10u);
assert(p.servo.position_period_us==100000LL);
assert(p.servo.observation_max_age_us==200000LL);
assert(p.servo.command_limit==0.4);
assert(p.servo.wheelbase==0.128);
assert(p.servo.track==0.1239);
assert(p.servo.wheel_limit==0.5235987755982988);
assert(p.servo.zero.zero==2000 && p.servo.zero.direction==-1);
assert(p.servo.zero.minimum==1000 && p.servo.zero.maximum==3000);
assert(p.valid(map));

const std::uint32_t boot[]={1}; arena_actuation::TxConfig tc;
arena_actuation::ActuationMotorEndpoint endpoint(p.endpoint_receiver_config(map),boot,1,30000,
  arena_actuation::PayloadLayout::kEnvelope,tc,p.kp,p.ki,.05,0.,p.maximum_duty);
endpoint.connect(0); assert(!endpoint.receiver().armed());
arena_servo::RuntimeServo servo(map,p.servo); assert(!servo.engaged());

p.kp=0; assert(!p.valid(map)); p.kp=.6;
p.motor_direction_confirmed=false; assert(!p.valid(map));
}
