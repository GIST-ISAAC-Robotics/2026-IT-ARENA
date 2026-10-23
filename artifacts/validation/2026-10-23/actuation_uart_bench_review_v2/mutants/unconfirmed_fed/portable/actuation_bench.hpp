#pragma once
// 출력 비활성 통신 bring-up 연결부. GPIO/UART/타이머 driver 또는 구동 펌웨어가 아니다.
// 생성 뒤 단일 작업에서만 호출한다. 통신 ISR/callback에서 객체에 직접 접근하지 않는다.
#include "actuation_tx.hpp"

namespace arena_actuation {

class ActuationBenchEndpoint {
 public:
  ActuationBenchEndpoint(const ReceiverConfig& receiver_config, const std::uint32_t* boot,
                         std::size_t boot_length, std::uint64_t assembly_timeout_us,
                         PayloadLayout layout, const TxConfig& tx_config)
      : receiver_(receiver_config, boot, boot_length),
        ingress_(receiver_, assembly_timeout_us, layout), tx_(tx_config, receiver_) {
    tx_.set_output_ready(false);
    tx_.set_link_up(false);
  }
  ActuationBenchEndpoint(const ActuationBenchEndpoint&) = delete;
  ActuationBenchEndpoint& operator=(const ActuationBenchEndpoint&) = delete;

  IngressResult connect(std::int64_t now) {
    auto result = ingress_.connect(now);
    sync(now);
    return result;
  }
  IngressResult disconnect(std::int64_t now) {
    auto result = ingress_.disconnect(now);
    sync(now);
    return result;
  }
  IngressResult clear_link_fault(std::int64_t now) {
    auto result = ingress_.clear_link_fault(now);
    sync(now);
    return result;
  }
  IngressResult feed(std::int64_t now, const std::uint8_t* bytes, std::size_t size) {
    auto result = ingress_.feed(now, bytes, size,
        [&](const IngressDelivery& delivery, const Command& command) {
          tx_.on_delivery(delivery, command, now);
        }, false);
    sync(now);
    return result;
  }
  IngressResult poll(std::int64_t now) {
    auto result = ingress_.poll(now,
        [&](const IngressDelivery& delivery, const Command& command) {
          tx_.on_delivery(delivery, command, now);
        }, false);
    sync(now);
    return result;
  }
  void control_tick(std::int64_t now) {
    receiver_.tick(now);
    sync(now);
  }
  template<class Sink>
  TxPumpResult pump(std::int64_t now, Sink&& sink) {
    // feed/poll/pump로 control_tick을 대신하지 않는다. sink는 비차단이어야 한다.
    sync(now);
    return tx_.pump(now, sink);
  }

  const ActuationReceiverCore& receiver() const { return receiver_; }
  const ActuationIngress& ingress() const { return ingress_; }
  const ActuationTxScheduler& tx() const { return tx_; }
  static constexpr bool output_ready() { return false; }

 private:
  void sync(std::int64_t now) {
    if (ingress_.link().session_breaks() != breaks_seen_) {
      breaks_seen_ = ingress_.link().session_breaks();
      tx_.on_session_break(now);
    }
    tx_.set_link_up(ingress_.link().state() == LinkState::kConnected);
    tx_.observe(now);
  }
  // const 조회만 제공한다. 가짜 피드백/준비값으로 ARM할 수 있는 API가 없다.
  ActuationReceiverCore receiver_;
  ActuationIngress ingress_;
  ActuationTxScheduler tx_;
  std::uint64_t breaks_seen_ = 0;
};

}  // namespace arena_actuation
