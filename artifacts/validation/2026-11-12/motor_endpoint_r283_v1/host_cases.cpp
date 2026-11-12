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
static bool run(const std::string& name){
  Fixture f;
  if(name=="default_not_ready"){
    f.prepare();f.send(220000,"ARM",0,false);
    return f.endpoint.receiver().state()==aa::ReceiverState::kDisarmed && halted(f.endpoint.control_tick(225000));
  }
  auto first=f.start();if(!active(first))return false;
  if(name=="arm_drive_wire"){
    bool ack=false;for(auto& s:f.received)if(s.find("\"accepted\":true")!=std::string::npos)ack=true;
    return ack && f.endpoint.receiver().motor_mode() && first.steering_rad==.2;
  }
  if(name=="negative_motor_motion"){
    f.feedback(230000,-1);auto next=f.endpoint.control_tick(230000,true);
    return active(next) && f.endpoint.receiver().mean_mps()<0 && next.motor.effort>first.motor.effort;
  }
  if(name=="zero_resume"){
    f.send(230000,"DRIVE",0);f.feedback(230000);
    if(!halted(f.endpoint.control_tick(230000,true)))return false;
    f.send(235000,"DRIVE",.5);f.feedback(235000);return active(f.endpoint.control_tick(235000,true));
  }
  if(name=="stop_clear_arm"){
    f.send(230000,"STOP");f.feedback(230000);auto stop=f.endpoint.control_tick(230000,true);
    if(stop.state!=aa::ReceiverState::kEstop || !halted(stop)){std::fprintf(stderr,"stop state:%d reason:%s\n",int(stop.state),stop.reason);return false;}
    f.send(235000,"CLEAR");f.feedback(235000);f.endpoint.control_tick(235000,true);
    f.send(240000,"ARM");f.feedback(240000);f.endpoint.control_tick(240000,true);
    f.send(245000,"DRIVE",.5);f.feedback(245000);auto out=f.endpoint.control_tick(245000,true);
    if(!active(out)){for(auto& s:f.received)std::fprintf(stderr,"%s\n",s.c_str());std::fprintf(stderr,"resume state:%d reason:%s\n",int(out.state),out.reason);}
    return active(out);
  }
  if(name=="output_unavailable"){
    f.feedback(230000);auto out=f.endpoint.control_tick(230000);return halted(out) && out.state==aa::ReceiverState::kFault;
  }
  if(name=="feedback_expired"){
    for(std::int64_t t=230000;t<=270000;t+=5000)f.endpoint.control_tick(t,true);
    auto out=f.endpoint.control_tick(275001,true);return halted(out) && out.state==aa::ReceiverState::kFault;
  }
  if(name=="command_expired"){
    aa::MotorCyclePreview out;
    for(std::int64_t t=230000;t<=325000;t+=5000){f.feedback(t);out=f.endpoint.control_tick(t,true);}
    return halted(out) && out.state==aa::ReceiverState::kFault;
  }
  if(name=="disconnect"){
    f.endpoint.disconnect(230000);f.feedback(230000);auto out=f.endpoint.control_tick(230000,true);
    return halted(out) && out.state==aa::ReceiverState::kEstop;
  }
  if(name=="tx_contract_fault"){
    f.endpoint.pump(245000,[](const std::uint8_t*,std::size_t size){return size+1;},true);
    f.feedback(245000);auto out=f.endpoint.control_tick(245000,true);
    return f.endpoint.tx().fault()==aa::TxFault::kSinkContract && halted(out) && out.state==aa::ReceiverState::kFault;
  }
  return false;
}
int main(){
  const char* cases[]={"default_not_ready","arm_drive_wire","negative_motor_motion","zero_resume","stop_clear_arm","output_unavailable","feedback_expired","command_expired","disconnect","tx_contract_fault"};
  bool all=true;
  for(auto name:cases){bool ok=false;try{ok=run(name);}catch(const char* why){std::fprintf(stderr,"%s: %s\n",name,why);}
    std::printf("%s:%s\n",name,ok?"PASS":"FAIL");all=all&&ok;}
  return all?0:1;
}
