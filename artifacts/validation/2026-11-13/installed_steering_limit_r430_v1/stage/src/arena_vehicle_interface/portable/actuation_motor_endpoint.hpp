#pragma once
// 한 소유 작업의 UART 계약 + 실제 HAL 피드백 연결점 + PI 계산.
// 이 파일은 GPIO/PWM/서보 출력이나 독립 watchdog을 구현하지 않는다.
#include "actuation_tx.hpp"
#include "motor_control_candidate.hpp"

namespace arena_actuation {
class ActuationMotorEndpoint {
 public:
  ActuationMotorEndpoint(const ReceiverConfig& cfg, const std::uint32_t* boot,
                        std::size_t length, std::uint64_t assembly_timeout_us,
                        PayloadLayout layout, const TxConfig& tx,
                        double kp, double ki, double max_dt_s,
                        double output_min = -1.0, double output_max = 1.0)
      : receiver_(cfg,boot,length), ingress_(receiver_,assembly_timeout_us,layout),
        tx_(tx,receiver_), cycle_(receiver_,kp,ki,max_dt_s,output_min,output_max) {}
  ActuationMotorEndpoint(const ActuationMotorEndpoint&)=delete;
  ActuationMotorEndpoint& operator=(const ActuationMotorEndpoint&)=delete;

  IngressResult connect(std::int64_t now) {
    auto r=ingress_.connect(now);sync(now,false);return r;
  }
  IngressResult disconnect(std::int64_t now) {
    auto r=ingress_.disconnect(now);sync(now,false);return r;
  }
  IngressResult clear_link_fault(std::int64_t now) {
    auto r=ingress_.clear_link_fault(now);sync(now,false);return r;
  }
  // 준비 여부는 호출 당시 HAL 관측이다. 이전 true를 저장해 다음 호출에 쓰지 않는다.
  IngressResult feed(std::int64_t now,const std::uint8_t* bytes,std::size_t size,bool ready=false) {
    auto r=ingress_.feed(now,bytes,size,[&](const IngressDelivery& d,const Command& c){
      tx_.on_delivery(d,c,now);
    },usable_ready(ready));
    sync(now,ready);return r;
  }
  IngressResult poll(std::int64_t now,bool ready=false) {
    auto r=ingress_.poll(now,[&](const IngressDelivery& d,const Command& c){
      tx_.on_delivery(d,c,now);
    },usable_ready(ready));
    sync(now,ready);return r;
  }
  bool motor_feedback(std::int64_t now,IntInput sequence,IntInput capture,IntInput count) {
    return receiver_.update_motor_encoder(now,sequence,capture,count);
  }
  void physical_stop(bool pressed,std::int64_t now) { receiver_.set_estop(pressed,now); }
  void check_deadlines(std::int64_t now,bool ready=false) {
    receiver_.check_deadlines(now);
    if(receiver_.armed() && !usable_ready(ready))
      receiver_.report_control_fault(ControlFault::kOutputUnavailable,now);
    sync(now,ready);
  }
  MotorCyclePreview control_tick(std::int64_t now,bool ready=false,bool inhibit_motor=false) {
    // 유효 시각·명령·피드백/주기부터 검사한다. 반환 의도는 이 호출에만 적용한다.
    auto result=cycle_.tick(now,usable_ready(ready),inhibit_motor);sync(now,ready);return result;
  }
  template<class Sink>
  TxPumpResult pump(std::int64_t now,Sink&& sink,bool ready=false) {
    sync(now,ready);
    auto result=tx_.pump(now,sink);
    if(tx_.fault()!=TxFault::kNone)receiver_.report_control_fault(ControlFault::kOutputUnavailable,now);
    return result;
  }
  const ActuationReceiverCore& receiver() const { return receiver_; }
  const ActuationIngress& ingress() const { return ingress_; }
  const ActuationTxScheduler& tx() const { return tx_; }
 private:
  bool usable_ready(bool ready) const {
    return ready && ingress_.link().state()==LinkState::kConnected &&
      ingress_.fault()==IngressFault::kNone && tx_.fault()==TxFault::kNone;
  }
  void sync(std::int64_t now,bool ready) {
    if(ingress_.link().session_breaks()!=breaks_seen_) {
      breaks_seen_=ingress_.link().session_breaks();tx_.on_session_break(now);
    }
    const bool connected=ingress_.link().state()==LinkState::kConnected;
    tx_.set_link_up(connected);
    tx_.set_output_ready(usable_ready(ready));
    tx_.observe(now);
  }
  ActuationReceiverCore receiver_;
  ActuationIngress ingress_;
  ActuationTxScheduler tx_;
  MotorControlCandidate cycle_;
  std::uint64_t breaks_seen_=0;
};
} // namespace arena_actuation
