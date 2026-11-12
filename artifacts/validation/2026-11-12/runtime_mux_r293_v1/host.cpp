// PC 프레임 결합/대역 예산 검사. 장치/핀 접근이나 실시간 보증이 아니다.
#include "runtime_tx.hpp"
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include <stdexcept>
namespace aa=arena_actuation;
using Bytes=std::vector<std::uint8_t>;
static void require(bool condition,const char* what){if(!condition)throw std::runtime_error(what);}
static void put(Bytes& out,std::size_t at,std::uint64_t v,int count){for(int i=0;i<count;++i)out[at+i]=static_cast<std::uint8_t>(v>>(i*8));}
static Bytes imu(std::uint32_t seq,std::int64_t now){
  Bytes b(68);put(b,0,0x31554d49,4);put(b,4,68,2);b[6]=1;b[7]=1;
  put(b,8,1234,4);put(b,12,seq,4);put(b,16,now,8);put(b,24,now+100,8);
  b[39]=static_cast<std::uint8_t>((seq%2+1)<<3);
  // 본문 안 다른 magic도 프레임 중간으로 끼어들어서는 안 된다.
  b[40]='I';b[41]='A';b[42]='I';b[43]='M';b[44]='U';b[45]='1';
  put(b,64,aa::crc32(b.data(),64),4);return b;
}
static Bytes observation(unsigned seq,std::int64_t now){
  char text[850];int size=std::snprintf(text,sizeof(text),
    "{\"type\":\"runtime_observation_v1\",\"boot\":1234,\"seq\":%u,\"mcu_us\":%lld,"
    "\"output_ready\":false,\"encoder_motion_verified\":false,\"pcnt_count\":0,"
    "\"pcnt_begin_us\":%lld,\"pcnt_end_us\":%lld,\"servo_mode\":-1,\"servo_position\":-1,"
    "\"servo_reply_us\":0,\"servo_failed\":true,\"imu_dropped\":0,\"observation_dropped\":0}",
    seq,static_cast<long long>(now),static_cast<long long>(now),static_cast<long long>(now+100));
  require(size>0 && size<850,"observation format");Bytes b(size+8);
  require(aa::encode_frame(reinterpret_cast<std::uint8_t*>(text),size,b.data(),b.size())==b.size(),"observation encode");return b;
}
struct Stub {
  Bytes pending;std::size_t offset=0,calls=0;
  aa::TxFault fault()const{return aa::TxFault::kNone;}
  bool frame_in_flight()const{return offset>0 && offset<pending.size();}
  Stub& tx(){return *this;}
  template<class Sink>void pump(std::int64_t,Sink& sink,bool ready){
    require(!ready,"preview readiness");++calls;
    if(offset<pending.size())offset+=sink(pending.data()+offset,pending.size()-offset);
  }
};
static void boundaries(){
  RuntimeTx mux;Stub endpoint;Bytes received;std::size_t budget=7;
  auto sink=[&](const std::uint8_t* p,std::size_t n){auto take=std::min(budget,n);budget-=take;received.insert(received.end(),p,p+take);return take;};
  auto first=imu(0,0),second=imu(1,0),obs=observation(0,0),control=observation(99,0);
  require(mux.imu(first.data(),first.size()),"imu enqueue");require(mux.pump(0,endpoint,sink),"first partial");
  require(received.size()==7,"first partial amount");endpoint.pending=control;auto calls=endpoint.calls;
  budget=10000;require(mux.pump(1,endpoint,sink),"imu tail");require(endpoint.calls==calls && received==first,"endpoint interleaved IMU");
  require(mux.observation(obs.data(),obs.size()),"observation enqueue");
  require(mux.imu(second.data(),second.size()),"second IMU enqueue");
  budget=control.size()+3;require(mux.pump(2,endpoint,sink),"control plus partial observation");
  require(!mux.observation(control.data(),control.size()) && mux.observation_dropped()==1,"overwrite inflight observation");
  calls=endpoint.calls;budget=10000;require(mux.pump(3,endpoint,sink),"observation tail");
  require(endpoint.calls==calls,"endpoint interleaved observation");
  require(mux.pump(4,endpoint,sink),"second IMU send");
  Bytes expected=first;expected.insert(expected.end(),control.begin(),control.end());expected.insert(expected.end(),obs.begin(),obs.end());expected.insert(expected.end(),second.begin(),second.end());
  require(received==expected,"mixed frame bytes changed");
  RuntimeTx overflow;Stub quiet;Bytes all;
  for(unsigned i=0;i<64;++i){auto b=imu(i,0);require(overflow.imu(b.data(),b.size()),"queue capacity");all.insert(all.end(),b.begin(),b.end());}
  require(!overflow.imu(first.data(),first.size()) && overflow.imu_dropped()==1,"queue overflow accounting");
  Bytes actual;auto full=[&](const std::uint8_t* p,std::size_t n){actual.insert(actual.end(),p,p+n);return n;};
  for(int i=0;i<70;++i)require(overflow.pump(i,quiet,full),"queue drain");
  require(actual==all,"queue order");
  for(bool obs_fault:{false,true}){
    RuntimeTx fault;Stub quiet2;
    require(obs_fault?fault.observation(obs.data(),obs.size()):fault.imu(first.data(),first.size()),"fault setup");
    auto bad=[](const std::uint8_t*,std::size_t n){return n+1;};
    require(!fault.pump(0,quiet2,bad),"sink overcount accepted");
    require(!fault.pump(1,quiet2,full),"sticky fault lost");
  }
}
static void integration(const std::string& folder,bool burst){
  aa::ReceiverConfig cfg;cfg.feedback_mode=aa::FeedbackMode::kMotorShaft;cfg.ticks_per_rev=28;cfg.gear_ratio=2.5*81./17.;cfg.wheel_radius_m=.0325;
  aa::TxConfig tc;tc.offer_status_v1=true;tc.offer_motor_feedback_v1=true;tc.offer_period_us=20000;
  tc.diag_period_us=1000000;tc.max_consecutive_events=2;tc.line_ns_per_byte=10851;
  constexpr std::uint32_t boot[]={'1','2','3','4'};
  aa::ActuationMotorEndpoint endpoint(cfg,boot,4,30000,aa::PayloadLayout::kEnvelope,tc,.6,1.2,.05);
  endpoint.connect(0);RuntimeTx mux;Bytes wire;unsigned seq=0,observations=0;std::size_t control_ticks=0;
  std::uint64_t fractional=0;std::size_t budget=0;
  for(std::int64_t t=0;t<5500000;t+=1000){
    if(t%5000==0){auto intent=endpoint.control_tick(t,false);++control_ticks;require(intent.motor.effort==0 && !endpoint.receiver().armed(),"unexpected drive permission");}
    if(t<5000000 && t%5000==0)for(int i=0;i<2;++i){auto b=imu(seq++,t);mux.imu(b.data(),b.size());}
    if(t<5000000 && t%20000==0){auto b=observation(observations++,t);mux.observation(b.data(),b.size());}
    endpoint.poll(t,false);fractional+=92160;budget=fractional/1000;fractional%=1000;
    if(burst && t>=1000000 && t<1025000)budget=0;
    auto sink=[&](const std::uint8_t* p,std::size_t n){auto take=std::min(budget,n);budget-=take;wire.insert(wire.end(),p,p+take);return take;};
    require(mux.pump(t,endpoint,sink),"integration TX fault");
  }
  // 현재 시작된 프레임만 마저 전송. 새 프레임이 계속 생기는 idle probe는 하지 않는다.
  for(int i=0;i<200;++i){auto sink=[&](const std::uint8_t* p,std::size_t n){wire.insert(wire.end(),p,p+n);return n;};require(mux.pump(5500000,endpoint,sink),"final drain");}
  auto name=folder+(burst?"/burst":"/steady");auto file=std::fopen((name+".bin").c_str(),"wb");require(file,"open raw");require(std::fwrite(wire.data(),1,wire.size(),file)==wire.size(),"write raw");std::fclose(file);
  file=std::fopen((name+".json").c_str(),"w");require(file,"open summary");
  std::fprintf(file,"{\"imu_generated\":%u,\"observation_generated\":%u,\"imu_dropped\":%llu,\"observation_dropped\":%llu,\"bytes\":%zu,\"control_ticks\":%zu,\"output_ready\":false}\n",seq,observations,static_cast<unsigned long long>(mux.imu_dropped()),static_cast<unsigned long long>(mux.observation_dropped()),wire.size(),control_ticks);std::fclose(file);
}
int main(int argc,char** argv){try{require(argc==2,"output folder required");boundaries();integration(argv[1],false);integration(argv[1],true);std::puts("frame boundaries, queue overflow, immutable partial frames, sticky faults, steady/burst synthetic budgets passed");return 0;}catch(const std::exception& e){std::fprintf(stderr,"FAIL: %s\n",e.what());return 1;}}
