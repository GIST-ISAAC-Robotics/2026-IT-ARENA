#pragma once
// 송신 정책 후보의 공유 합성 사례. 실제 UART에 쓰지 않고 고정 배열 sink에만 쓴다.
#include "ingress_cases.hpp"
#include "actuation_tx.hpp"

namespace arena_smoke {

// 쓰기 결과를 고정 배열에 모은다. limit는 호출당 수락 바이트(0이면 backpressure).
struct ArraySink {
  std::array<std::uint8_t, 4096> data{};
  std::size_t used = 0;
  std::size_t limit = 4096;
  std::size_t operator()(const std::uint8_t* bytes, std::size_t size) {
    std::size_t n = size < limit ? size : limit;
    if (n > data.size() - used) n = data.size() - used;
    std::memcpy(data.data() + used, bytes, n);
    used += n;
    return n;
  }
};
struct ViolatingSink {
  std::size_t operator()(const std::uint8_t*, std::size_t size) { return size + 1; }
};

inline constexpr std::uint32_t kQuoteBoot[] = {'q', '"', 'x'};
inline aa::TxConfig tx_config() {
  aa::TxConfig c;
  c.offer_period_us = 20000;   // 합성 시험값. 실물 운영값 아님
  c.diag_period_us = 0;
  c.max_consecutive_events = 2;
  return c;
}

// 큰 객체는 정적 저장 공간에 둔다(호스트/대상 main이 소유).
struct TxScratch {
  aa::ActuationReceiverCore receiver{motor_config(), kSmokeBoot, 10};
  aa::ActuationTxScheduler tx{tx_config(), receiver};
  aa::ActuationReceiverCore clock_receiver{motor_config(), kSmokeBoot, 10};
  aa::ActuationTxScheduler clock_tx{tx_config(), clock_receiver};
  aa::ActuationTxScheduler contract_tx{tx_config(), clock_receiver};
  aa::ActuationReceiverCore quote_receiver{motor_config(), kQuoteBoot, 3};
  aa::ActuationTxScheduler quote_tx{tx_config(), quote_receiver};
  ArraySink sink;
  aa::FrameDecoder decoder;
};

inline bool starts_with(const std::uint8_t* payload, std::size_t size, const char* prefix) {
  const std::size_t n = std::strlen(prefix);
  return size >= n && std::memcmp(payload, prefix, n) == 0;
}

inline Report run_tx(Scratch& scratch, TxScratch& f) {
  Report report;
  auto& receiver = f.receiver;
  auto& tx = f.tx;
  report.record(tx.fault() == aa::TxFault::kNone && tx.worst_payload(aa::TxKind::kAck) <= aa::kMaxPayload &&
                tx.worst_payload(aa::TxKind::kOffer) <= aa::kMaxPayload &&
                tx.worst_payload(aa::TxKind::kState) <= aa::kMaxPayload &&
                tx.worst_payload(aa::TxKind::kDiagnostic) <= aa::kMaxPayload);
  // 부팅 상태 알림과 첫 offer. offer 임대는 materialize 시각에 발행된다.
  stationary_samples(receiver, 0, 220000);
  f.sink.used = 0;
  const auto first = tx.pump(220000, f.sink);
  unsigned kinds = 0;
  f.decoder.feed(f.sink.data.data(), f.sink.used < 256 ? f.sink.used : 256, [&](const std::uint8_t* p, std::size_t n) {
    if (kinds == 0 && starts_with(p, n, "{\"type\":\"state\"")) kinds |= 1;
    else if (kinds == 1 && starts_with(p, n, "{\"type\":\"offer\"")) kinds |= 2;
  });
  f.decoder.feed(f.sink.data.data() + 256, f.sink.used > 256 ? f.sink.used - 256 : 0,
                 [&](const std::uint8_t* p, std::size_t n) {
    if (kinds == 1 && starts_with(p, n, "{\"type\":\"offer\"")) kinds |= 2;
  });
  report.record(first.materialized == 2 && first.completed == 2 && kinds == 3 && receiver.lease_count() == 1 &&
                receiver.lease_at(0).issued_us == 220000);

  // 요청 ID의 escape/astral 문자가 Python ensure_ascii와 같은 바이트로 ack에 실린다.
  const int size = std::snprintf(scratch.text, sizeof(scratch.text),
      "{\"type\":\"command\",\"request\":\"q\\\"\\\\\\n\xF0\x9F\x98\x80\",\"command\":{\"v\":1,\"kind\":\"ARM\","
      "\"boot\":\"smoke-only\",\"generation\":0,\"lease\":%lld,\"client\":\"smoke-only\",\"seq\":1}}",
      static_cast<long long>(receiver.lease_at(0).id));
  const bool parsed = size > 0 && scratch.parser.parse(reinterpret_cast<const std::uint8_t*>(scratch.text),
      static_cast<std::size_t>(size), aa::PayloadLayout::kEnvelope, scratch.command) == aa::ParseStatus::kObject;
  const auto armed = receiver.receive(scratch.command, 220000);
  tx.on_delivery(aa::IngressDelivery{aa::FrameOrigin::kLive, aa::ParseStatus::kObject, aa::classify(scratch.command),
                                     true, armed}, scratch.command, 220000);
  f.sink.used = 0;
  const auto after_arm = tx.pump(220000, f.sink);
  constexpr char expected_request[] = {'"', 'q', '\\', '"', '\\', '\\', '\\', 'n', '\\', 'u', 'd', '8', '3', 'd',
                                       '\\', 'u', 'd', 'e', '0', '0', '"', ','};
  constexpr char ack_head[] = "{\"type\":\"ack\",\"request\":";
  const std::size_t head = sizeof(ack_head) - 1;
  report.record(parsed && armed.accepted && after_arm.materialized == 2 &&
                after_arm.frames[0].kind == aa::TxKind::kAck && after_arm.frames[1].kind == aa::TxKind::kOffer &&
                after_arm.frames[1].urgent && starts_with(f.sink.data.data() + 4, f.sink.used, ack_head) &&
                std::memcmp(f.sink.data.data() + 4 + head, expected_request, sizeof(expected_request)) == 0);

  // 부분 쓰기 중 STOP: 시작한 프레임은 끝까지 그대로, 그 뒤 새 세대 offer.
  f.sink.used = 0;
  f.sink.limit = 3;
  const auto partial = tx.pump(240000, f.sink);   // state 알림(ARM)을 3 B만 보낸다
  const auto remaining = tx.in_flight_remaining();
  const char stop_text[] = "{\"v\":1,\"kind\":\"STOP\"}";
  const bool stop_ok = parse(scratch, stop_text) && receiver.receive(scratch.command, 240000).accepted;
  tx.observe(240000);
  const auto in_flight_kind = tx.current_frame().kind;
  report.record(partial.backpressure && partial.bytes_written == 3 && remaining > 0 && stop_ok &&
                in_flight_kind == aa::TxKind::kState && tx.frame_in_flight() && tx.in_flight_remaining() == remaining);
  f.sink.limit = 4096;
  const auto rest = tx.pump(240000, f.sink);
  report.record(rest.completed >= 1 && rest.materialized >= 1 && rest.frames[0].kind == aa::TxKind::kOffer &&
                rest.frames[0].urgent && rest.frames[0].generation == receiver.generation());

  // ack 큐 한도: 비STOP 8개 뒤 STOP 결과는 가장 오래된 비STOP을 축출, 그 외에는 명시 폐기.
  f.sink.limit = 0;
  tx.pump(250000, f.sink);
  const char disarm_text[] = "{\"type\":\"command\",\"request\":\"d\",\"command\":{\"v\":1,\"kind\":\"DISARM\","
      "\"boot\":\"smoke-only\",\"generation\":0,\"lease\":1,\"client\":\"smoke-only\",\"seq\":1}}";
  for (int i = 0; i < 9; ++i) {
    scratch.parser.parse(reinterpret_cast<const std::uint8_t*>(disarm_text), sizeof(disarm_text) - 1,
                         aa::PayloadLayout::kEnvelope, scratch.command);
    const auto r = receiver.receive(scratch.command, 250000);
    tx.on_delivery(aa::IngressDelivery{aa::FrameOrigin::kLive, aa::ParseStatus::kObject,
                                       aa::classify(scratch.command), true, r}, scratch.command, 250000);
  }
  const char stop_env[] = "{\"type\":\"command\",\"request\":\"s\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}";
  scratch.parser.parse(reinterpret_cast<const std::uint8_t*>(stop_env), sizeof(stop_env) - 1,
                       aa::PayloadLayout::kEnvelope, scratch.command);
  const auto stopped = receiver.receive(scratch.command, 250000);
  tx.on_delivery(aa::IngressDelivery{aa::FrameOrigin::kLive, aa::ParseStatus::kObject, aa::classify(scratch.command),
                                     true, stopped}, scratch.command, 250000);
  report.record(tx.ack_count() == aa::kTxAckCapacity && tx.stats().ack_dropped_overflow == 1 &&
                tx.stats().ack_evicted_for_stop == 1);

  // 세션 경계: 전송 중 꼬리 폐기·ack 비움. 이 객체는 CLEAR/ARM을 만들지 않는다.
  f.sink.limit = 2;
  tx.pump(250000, f.sink);
  const bool had_tail = tx.frame_in_flight();
  tx.on_session_break(250000);
  report.record(had_tail && !tx.frame_in_flight() && tx.ack_count() == 0 && tx.stats().aborted_frames == 1 &&
                receiver.state() == aa::ReceiverState::kEstop);

  // 막힌 sink에서는 임대가 추가 발행되지 않는다(전송 중 프레임 하나만 고정).
  f.sink.limit = 0;
  tx.pump(260000, f.sink);
  const auto leases = receiver.lease_count();
  for (std::int64_t now = 280000; now <= 400000; now += 20000) tx.pump(now, f.sink);
  report.record(receiver.lease_count() <= leases && tx.frame_in_flight());

  // 시계 역행·sink 계약 위반·escape가 필요한 boot는 영구 고장으로 침묵한다.
  f.sink.limit = 4096;
  f.sink.used = 0;
  f.clock_tx.pump(1000, f.sink);
  const auto used = f.sink.used;
  f.clock_tx.pump(999, f.sink);
  f.clock_tx.pump(50000, f.sink);
  ViolatingSink violating;
  f.contract_tx.pump(2000, violating);
  f.sink.used = 0;
  f.quote_tx.pump(0, f.sink);
  report.record(used > 0 && f.clock_tx.fault() == aa::TxFault::kInvalidClock &&
                f.contract_tx.fault() == aa::TxFault::kSinkContract && !f.contract_tx.frame_in_flight() &&
                f.quote_tx.fault() == aa::TxFault::kInvalidConfig && f.sink.used == 0);
  return report;
}

}  // namespace arena_smoke
