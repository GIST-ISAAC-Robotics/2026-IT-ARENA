// ESP32-S3 도구체인의 컴파일/링크 크기 확인용. app_main/입출력/flash 없음.
#include "actuation_motor_endpoint.hpp"
static constexpr std::uint32_t boot[]={'c','o','m','p','i','l','e'};
static arena_actuation::ReceiverConfig config(){
  arena_actuation::ReceiverConfig c;c.feedback_mode=arena_actuation::FeedbackMode::kMotorShaft;
  c.ticks_per_rev=28;c.gear_ratio=2.5*81./17.;c.wheel_radius_m=.0325;return c;
}
static arena_actuation::TxConfig txconfig(){
  arena_actuation::TxConfig c;c.offer_period_us=20000;c.max_consecutive_events=2;return c;
}
arena_actuation::ActuationMotorEndpoint endpoint{config(),boot,7,30000,arena_actuation::PayloadLayout::kEnvelope,txconfig(),.6,1.2,.05};
extern "C" void compile_endpoint_only(){
  endpoint.connect(0);endpoint.control_tick(0);endpoint.disconnect(1);
}
