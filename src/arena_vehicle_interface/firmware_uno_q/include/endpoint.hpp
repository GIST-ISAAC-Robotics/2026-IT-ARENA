#pragma once
#include "actuation_tx.hpp"
#include "motor_control_candidate.hpp"
namespace unoq {
// 기존 IA receiver/ingress/TX/PI를 그대로 소유. 미교정/bench에서는 PI를 호출하지 않는다.
class Endpoint {
 public:
  Endpoint(const arena_actuation::ReceiverConfig& cfg,const std::uint32_t* boot,
           const arena_actuation::TxConfig& tx,double kp,double ki,double duty,bool installed)
    : r_(cfg,boot,8),ingress_(r_,30000,arena_actuation::PayloadLayout::kEnvelope),
      tx_(tx,r_),cycle_(r_,kp,ki,.05,0.,duty),installed_(installed){}
  void connect(std::int64_t now){ingress_.connect(now);sync(now,false);}
  void feed(std::int64_t now,const std::uint8_t* p,std::size_t n,bool ready){
    ingress_.feed(now,p,n,[&](const arena_actuation::IngressDelivery& d,const arena_actuation::Command& c){tx_.on_delivery(d,c,now);},ready&&installed_);
    sync(now,ready);
  }
  void poll(std::int64_t now,bool ready){
    ingress_.poll(now,[&](const arena_actuation::IngressDelivery& d,const arena_actuation::Command& c){tx_.on_delivery(d,c,now);},ready&&installed_);sync(now,ready);
  }
  void check_deadlines(std::int64_t now,bool ready){
    r_.check_deadlines(now);if(r_.armed()&&!ready)r_.report_control_fault(arena_actuation::ControlFault::kOutputUnavailable,now);sync(now,ready);
  }
  void fault(std::int64_t now){r_.report_control_fault(arena_actuation::ControlFault::kOutputUnavailable,now);sync(now,false);}
  bool feedback(std::int64_t now,std::int64_t seq,std::int64_t count){
    using I=arena_actuation::IntInput;return r_.update_motor_encoder(now,I::of(seq),I::of(now),I::of(count));
  }
  arena_actuation::MotorCyclePreview tick(std::int64_t now,bool ready,bool inhibit){
    if(installed_)return cycle_.tick(now,ready,inhibit);
    r_.tick(now);return {};
  }
  template<class Sink> auto pump(std::int64_t now,Sink&& sink,bool ready){
    sync(now,ready);auto result=tx_.pump(now,sink);
    if(tx_.fault()!=arena_actuation::TxFault::kNone)fault(now);
    return result;
  }
  const arena_actuation::ActuationReceiverCore& receiver()const{return r_;}
  const arena_actuation::ActuationTxScheduler& tx()const{return tx_;}
 private:
  void sync(std::int64_t now,bool ready){
    if(ingress_.link().session_breaks()!=breaks_){breaks_=ingress_.link().session_breaks();tx_.on_session_break(now);}
    tx_.set_link_up(ingress_.link().state()==arena_actuation::LinkState::kConnected);
    tx_.set_output_ready(ready&&installed_&&ingress_.fault()==arena_actuation::IngressFault::kNone&&tx_.fault()==arena_actuation::TxFault::kNone);tx_.observe(now);
  }
  arena_actuation::ActuationReceiverCore r_;
  arena_actuation::ActuationIngress ingress_;
  arena_actuation::ActuationTxScheduler tx_;
  arena_actuation::MotorControlCandidate cycle_;
  bool installed_;std::uint64_t breaks_=0;
};
}
