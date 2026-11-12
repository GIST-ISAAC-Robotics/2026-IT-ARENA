// ESP32-S3 target compile only. No app_main/GPIO/UART/flash.
#include "actuation_motor_endpoint.hpp"
static constexpr std::uint32_t boot[]={'r','2','8','5'};
static arena_actuation::ReceiverConfig config(){
  arena_actuation::ReceiverConfig c;c.feedback_mode=arena_actuation::FeedbackMode::kMotorShaft;
  c.ticks_per_rev=28;c.gear_ratio=2.5*81./17.;c.wheel_radius_m=.0325;return c;
}
static arena_actuation::TxConfig txconfig(){
  arena_actuation::TxConfig c;c.offer_period_us=20000;c.max_consecutive_events=2;
  c.offer_status_v1=true;c.offer_motor_feedback_v1=true;return c;
}
static arena_actuation::ActuationMotorEndpoint endpoint{config(),boot,4,30000,arena_actuation::PayloadLayout::kEnvelope,txconfig(),.6,1.2,.05};
extern "C" std::size_t compile_feedback_only(std::int64_t now,std::uint8_t* out,std::size_t capacity){
  endpoint.connect(now);endpoint.control_tick(now);
  const auto result=endpoint.pump(now,[&](const std::uint8_t* p,std::size_t size){
    const auto n=std::min(capacity,size);std::memcpy(out,p,n);return n;
  });
  return result.bytes_written;
}
