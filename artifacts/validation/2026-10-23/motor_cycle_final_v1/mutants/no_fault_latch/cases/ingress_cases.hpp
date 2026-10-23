#pragma once
#include "smoke_cases.hpp"
#include "actuation_ingress.hpp"

namespace arena_smoke {
inline constexpr std::uint32_t kSmokeBoot[] = {'s','m','o','k','e','-','o','n','l','y'};
// 크기가 큰 연결부를 task stack에 두지 않는다. 호스트/대상 main이 정적 객체로 소유한다.
struct IngressScratch {
  aa::ActuationReceiverCore receiver{motor_config(), kSmokeBoot, 10};
  aa::ActuationIngress ingress{receiver, 30000, aa::PayloadLayout::kBareCommand};
};

inline Report run_ingress(Scratch& scratch, IngressScratch& fixture) {
  Report report;
  auto& receiver = fixture.receiver;
  auto& ingress = fixture.ingress;
  unsigned delivered = 0;
  aa::ReceiveResult received{false, aa::ReceiveReason::kInvalidMessage};
  auto observer = [&](const aa::IngressDelivery& d, const aa::Command&) {
    if (d.receiver_called || d.output_gate_rejected) { ++delivered; received = d.result; }
  };
  auto send = [&](bool output_ready = false) {
    delivered = 0;
    const auto n = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(scratch.text), std::strlen(scratch.text),
                                    scratch.frame.data(), scratch.frame.size());
    if (n == 0) return false;
    for (std::size_t start = 0; start < n; start += aa::kMaxInputPerCall) {
      const auto count = n - start < aa::kMaxInputPerCall ? n - start : aa::kMaxInputPerCall;
      if (ingress.feed(220000, scratch.frame.data() + start, count, observer, output_ready).link != aa::LinkResult::kOk)
        return false;
    }
    return delivered == 1;
  };
  report.record(ingress.connect(0).link == aa::LinkResult::kOk && receiver.state() == aa::ReceiverState::kDisarmed);
  stationary_samples(receiver, 0, 220000);
  auto offer = receiver.offer(220000);
  report.record(offer.ok && command(scratch, "ARM", receiver, offer.lease, 1) && send() &&
                !received.accepted && received.reason == aa::ReceiveReason::kOutputNotReady &&
                receiver.state() == aa::ReceiverState::kDisarmed && receiver.generation() == 0);
  report.record(send(true) && received.accepted);
  offer = receiver.offer(220000);
  report.record(offer.ok && command(scratch, "DRIVE", receiver, offer.lease, 2, true) && send() &&
                received.accepted && receiver.target_speed() == .5);
  const auto disconnected = ingress.disconnect(220000);
  report.record(disconnected.stop_injected && disconnected.stop_result.accepted &&
                receiver.state() == aa::ReceiverState::kEstop && receiver.target_speed() == 0. && receiver.steering() == .2);
  report.record(ingress.connect(220000).link == aa::LinkResult::kOk && receiver.state() == aa::ReceiverState::kEstop);
  // text에는 이전 세션 DRIVE가 그대로 남아 있다. 재연결 뒤 재사용하면 세션 불일치다.
  report.record(send() && !received.accepted && received.reason == aa::ReceiveReason::kSession);
  offer = receiver.offer(220000);
  report.record(offer.ok && command(scratch, "CLEAR", receiver, offer.lease, 1) && send() &&
                received.accepted && receiver.state() == aa::ReceiverState::kDisarmed);
  offer = receiver.offer(220000);
  // 앞 호출의 true를 연결부가 저장하지 않는다. 새 ARM은 다시 명시한 준비값이 필요하다.
  report.record(offer.ok && command(scratch, "ARM", receiver, offer.lease, 1) && send() &&
                !received.accepted && received.reason == aa::ReceiveReason::kOutputNotReady &&
                receiver.state() == aa::ReceiverState::kDisarmed);
  report.record(send(true) &&
                received.accepted && receiver.state() == aa::ReceiverState::kArmed);
  const auto oversize = ingress.feed(220000, nullptr, aa::kMaxInputPerCall + 1, observer);
  report.record(oversize.link == aa::LinkResult::kFaultRaised && oversize.stop_injected &&
                receiver.state() == aa::ReceiverState::kEstop && receiver.target_speed() == 0.);
  report.record(ingress.clear_link_fault(220000).link == aa::LinkResult::kOk &&
                ingress.link().state() == aa::LinkState::kDisconnected && receiver.state() == aa::ReceiverState::kEstop);
  return report;
}
}  // namespace arena_smoke
