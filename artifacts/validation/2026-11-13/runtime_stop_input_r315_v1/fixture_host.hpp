// 합성 count/clock/ready만 쓰는 호스트 결합 검사. 장치 파일을 열지 않는다.
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>
#include <type_traits>
#include "actuation_motor_endpoint.hpp"
namespace aa=arena_actuation;
static constexpr std::uint32_t boot[]={'s','m','o','k','e','-','o','n','l','y'};
static aa::ReceiverConfig config(){
  aa::ReceiverConfig c;c.feedback_mode=aa::FeedbackMode::kMotorShaft;c.ticks_per_rev=28;
  c.gear_ratio=2.5*81./17.;c.wheel_radius_m=.0325;return c;
}
static aa::TxConfig txconfig(){aa::TxConfig t;t.offer_period_us=20000;t.max_consecutive_events=2;return t;}
struct Fixture {
  aa::ActuationMotorEndpoint endpoint{config(),boot,10,30000,aa::PayloadLayout::kEnvelope,txconfig(),.6,1.2,.05};
  std::uint64_t seq=0;std::vector<std::string> received;aa::FrameDecoder decoder;
  void feedback(std::int64_t now,std::int64_t count=0){
    if(!endpoint.motor_feedback(now,aa::IntInput::of(now/5000),aa::IntInput::of(now),aa::IntInput::of(count)))
      throw "synthetic feedback rejected";
  }
  void pump(std::int64_t now,bool ready=true){
    std::vector<std::uint8_t> bytes;
    const auto sink=[&](const std::uint8_t* p,std::size_t n){bytes.insert(bytes.end(),p,p+n);return n;};
    endpoint.pump(now,sink,ready);endpoint.pump(now,sink,ready);
    for(std::size_t i=0;i<bytes.size();i+=aa::kMaxInputPerCall){
      const auto n=std::min(aa::kMaxInputPerCall,bytes.size()-i);
      if(!decoder.feed(bytes.data()+i,n,[&](const std::uint8_t* p,std::size_t s){received.emplace_back(reinterpret_cast<const char*>(p),s);}))throw "decoder failed";
    }
  }
  void send(std::int64_t now,const char* kind,double speed=0,bool ready=true){
    pump(now,ready);
    const auto& r=endpoint.receiver();if(!r.lease_count())throw "no transmitted lease";
    const auto lease=r.lease_at(r.lease_count()-1);char text[768],drive[200]{};
    if(std::strcmp(kind,"DRIVE")==0)std::snprintf(drive,sizeof(drive),
      ",\"speed_mps\":%.17g,\"steering_rad\":0.2,\"source_age_us\":0,\"ttl_us\":100000",speed);
    const int n=std::strcmp(kind,"STOP")==0 ? std::snprintf(text,sizeof(text),
      "{\"type\":\"command\",\"request\":\"test\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}") : std::snprintf(text,sizeof(text),
      "{\"type\":\"command\",\"request\":\"test\",\"command\":{\"v\":1,\"kind\":\"%s\",\"boot\":\"smoke-only\",\"generation\":%lld,\"lease\":%lld,\"client\":\"smoke-only\",\"seq\":%llu%s}}",
      kind,static_cast<long long>(r.generation()),static_cast<long long>(lease.id),static_cast<unsigned long long>(++seq),drive);
    if(n<=0 || n>=static_cast<int>(sizeof(text)))throw "format";
    std::uint8_t wire[aa::kMaxFrame];const auto size=aa::encode_frame(reinterpret_cast<const std::uint8_t*>(text),n,wire,sizeof(wire));
    if(!size)throw "frame";
    for(std::size_t i=0;i<size;i+=aa::kMaxInputPerCall)
      if(endpoint.feed(now,wire+i,std::min(aa::kMaxInputPerCall,size-i),ready).link!=aa::LinkResult::kOk)throw "feed";
    pump(now,ready);
  }
  void prepare(){
    endpoint.connect(0);
    for(std::int64_t t=0;t<=215000;t+=5000){feedback(t);endpoint.control_tick(t);}
  }
  aa::MotorCyclePreview start(){
    prepare();send(220000,"ARM");
    if(endpoint.receiver().state()!=aa::ReceiverState::kArmed){
      for(const auto& s:received)std::fprintf(stderr,"%s\n",s.c_str());
      throw "arm failed";
    }
    send(220001,"DRIVE",.5);feedback(225000);return endpoint.control_tick(225000,true);
  }
};
static_assert(std::is_same<decltype(std::declval<Fixture&>().endpoint.receiver()),const aa::ActuationReceiverCore&>::value);
static bool halted(const aa::MotorCyclePreview& r){return r.motor.effort==0 && r.motor.brake_requested;}
static bool active(const aa::MotorCyclePreview& r){return r.state==aa::ReceiverState::kActive && r.motor.effort>0;}
