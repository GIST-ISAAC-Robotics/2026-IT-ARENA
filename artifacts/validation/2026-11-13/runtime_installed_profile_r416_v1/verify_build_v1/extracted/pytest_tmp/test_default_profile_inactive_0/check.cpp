#include "board_profile.hpp"
#include "actuation_motor_endpoint.hpp"
#include <cassert>
int main(){
const arena_steering::CadMapping map(arena_steering::kCadSamples,sizeof(arena_steering::kCadSamples)/sizeof(arena_steering::kCadSamples[0]));
auto p=installed_board_profile();
assert(!p.valid(map)); assert(p.encoder_direction==0);

const std::uint32_t boot[]={1}; arena_actuation::TxConfig tc;
arena_actuation::ActuationMotorEndpoint endpoint(p.endpoint_receiver_config(map),boot,1,30000,
  arena_actuation::PayloadLayout::kEnvelope,tc,p.kp,p.ki,.05,0.,p.maximum_duty);
endpoint.connect(0); assert(!endpoint.receiver().armed());
arena_servo::RuntimeServo servo(map,p.servo); assert(!servo.engaged());

}
