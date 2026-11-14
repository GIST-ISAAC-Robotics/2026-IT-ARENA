#pragma once
#include "external_servo.hpp"
namespace unoq {
class Bench {
 public:
  // BNV1 <4s8sQQQBBHI. 1 ARM, 2 PULSE, 3 STOP, 4 KEEPALIVE.
  bool accept(const std::uint8_t* p,std::size_t n,std::int64_t now,
              const arena_actuation::ActuationReceiverCore& r,bool enabled,bool peripherals_ready) {
    if(n!=44 || std::memcmp(p,"BNV1",4)!=0){stop("malformed");return false;}
    const auto seq=read_u64(p+28);const auto action=p[36];
    if(action==3){stop("explicit_stop");return true;}
    const auto* l=lease(r,read_u64(p+20));
    if(!enabled || !peripherals_ready || !identity(p,r) || !l || now<l->issued_us || now>=l->expires_us ||
       r.state()!=arena_actuation::ReceiverState::kDisarmed ||
       (seen_ && seq<=sequence_) || seq>std::uint64_t(INT64_MAX))return false;
    const unsigned duty=unsigned(p[38])|(unsigned(p[39])<<8);const auto duration=read_u32(p+40);
    if(p[37]>1 || action<1 || action>4 || duty>150 || duration>5000000) {stop("invalid_request");return false;}
    if(action==1){
      if(armed_ || duty!=0 || duration!=0)return false;
      armed_=true;running_=false;reason_="explicit_arm";generation_=r.generation();
    }else if(action==2){
      if(!armed_ || running_ || now>=watchdog_ || duty==0 || duration==0)return false;
      running_=true;stop_at_=now+duration;duty_=duty;dir_=p[37]!=0;reason_="pulse";
    }else if(action==4){
      if(!armed_ || now>=watchdog_ || duty!=0 || duration!=0)return false;
    }
    seen_=true;sequence_=seq;watchdog_=l->expires_us;return true;
  }
  void tick(std::int64_t now,const arena_actuation::ActuationReceiverCore& r,bool ready) {
    if(!armed_)return;
    if(!ready || r.state()!=arena_actuation::ReceiverState::kDisarmed || generation_!=r.generation())stop("not_ready");
    else if(now>=watchdog_)stop("host_expired");
    else if(running_ && now>=stop_at_)stop("duration_complete");
  }
  void stop(const char* reason){armed_=running_=false;duty_=0;reason_=reason;}
  // The 1 ms output guard can beat the 5 ms main tick at a normal pulse end.
  // A control-gap guard before the bench's own deadline is still a fault.
  bool handle_output_deadline(std::int64_t now,const arena_actuation::ActuationReceiverCore& r,bool ready){
    if(!running_ || now<deadline())return false;
    tick(now,r,ready);return true;
  }
  bool armed()const{return armed_;} bool running()const{return running_;}
  double duty()const{return running_?duty_/1000.0:0.;} bool dir()const{return dir_;}
  const char* reason()const{return reason_;}
  std::int64_t sequence()const{return seen_?static_cast<std::int64_t>(sequence_):-1;}
  std::int64_t stop_at()const{return stop_at_;}
  std::int64_t deadline()const{return running_ && stop_at_<watchdog_?stop_at_:watchdog_;}
 private:
  bool seen_=false,armed_=false,running_=false,dir_=false;
  std::uint64_t sequence_=0;unsigned duty_=0;
  std::int64_t watchdog_=0,stop_at_=0,generation_=-1;
  const char* reason_="disarmed";
};
}
