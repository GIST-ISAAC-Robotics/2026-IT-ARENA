#pragma once
// 출력 비활성 bring-up 연결부의 공유 검사. UART·GPIO를 열지 않는다.
#include "tx_cases.hpp"
#include "actuation_bench.hpp"

namespace arena_smoke {
static_assert(std::is_same<decltype(std::declval<aa::ActuationBenchEndpoint&>().receiver()),
                           const aa::ActuationReceiverCore&>::value, "receiver must remain read-only");
static_assert(std::is_same<decltype(std::declval<aa::ActuationBenchEndpoint&>().tx()),
                           const aa::ActuationTxScheduler&>::value, "TX must remain read-only");
static_assert(!aa::ActuationBenchEndpoint::output_ready(), "bench cannot enable output");

struct BenchScratch {
  aa::ActuationBenchEndpoint primary{motor_config(), kSmokeBoot, 10, 30000,
                                     aa::PayloadLayout::kEnvelope, tx_config()};
  aa::ActuationBenchEndpoint partial{motor_config(), kSmokeBoot, 10, 30000,
                                     aa::PayloadLayout::kEnvelope, tx_config()};
  aa::ActuationBenchEndpoint sink_fault{motor_config(), kSmokeBoot, 10, 30000,
                                        aa::PayloadLayout::kEnvelope, tx_config()};
  ArraySink sink;
  aa::FrameDecoder decoder;
};

inline bool contains(const std::uint8_t* payload, std::size_t size, const char* text) {
  const auto length = std::strlen(text);
  for (std::size_t i = 0; i + length <= size; ++i)
    if (std::memcmp(payload + i, text, length) == 0) return true;
  return false;
}

inline Report run_bench(Scratch& scratch, BenchScratch& f) {
  Report report;
  auto& endpoint = f.primary;
  const auto& receiver = endpoint.receiver();
  auto send = [&](std::int64_t now, const char* payload) {
    const auto size = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(payload), std::strlen(payload),
                                        scratch.frame.data(), scratch.frame.size());
    if (size == 0) return false;
    for (std::size_t i = 0; i < size; i += aa::kMaxInputPerCall) {
      const auto n = size-i < aa::kMaxInputPerCall ? size-i : aa::kMaxInputPerCall;
      if (endpoint.feed(now, scratch.frame.data()+i, n).link != aa::LinkResult::kOk) return false;
    }
    return true;
  };
  auto pump_contains = [&](std::int64_t now, const char* token) {
    f.sink.used = 0;
    f.sink.limit = f.sink.data.size();
    endpoint.pump(now, f.sink);
    endpoint.pump(now, f.sink);
    bool seen = false;
    for (std::size_t i = 0; i < f.sink.used; i += aa::kMaxInputPerCall) {
      const auto n = f.sink.used-i < aa::kMaxInputPerCall ? f.sink.used-i : aa::kMaxInputPerCall;
      f.decoder.feed(f.sink.data.data()+i, n, [&](const std::uint8_t* p, std::size_t size) {
        seen = contains(p, size, token) || seen;
      });
    }
    return seen;
  };
  report.record(endpoint.connect(0).link == aa::LinkResult::kOk && !endpoint.output_ready() &&
                receiver.state() == aa::ReceiverState::kDisarmed && receiver.target_speed() == 0.);
  endpoint.control_tick(220000);
  report.record(pump_contains(220000, "\"output_ready\":false") && receiver.lease_count() > 0 &&
                !receiver.fresh_feedback(220000));
  const auto lease = receiver.lease_at(0).id;
  const auto control = [&](const char* kind, std::int64_t generation, std::int64_t id) {
    return std::snprintf(scratch.text, sizeof(scratch.text),
        "{\"type\":\"command\",\"request\":\"req\",\"command\":{\"v\":1,\"kind\":\"%s\","
        "\"boot\":\"smoke-only\",\"generation\":%lld,\"lease\":%lld,\"client\":\"smoke-only\",\"seq\":1}}",
        kind, static_cast<long long>(generation), static_cast<long long>(id));
  };
  control("ARM", 0, lease);
  report.record(send(220000, scratch.text) && pump_contains(220000, "\"reason\":\"output_not_ready\"") &&
                receiver.state() == aa::ReceiverState::kDisarmed && receiver.generation() == 0);
  control("DISARM", 0, lease);
  report.record(send(220000, scratch.text) && pump_contains(220000, "\"accepted\":false") && receiver.target_speed() == 0.);
  report.record(send(220000, "{\"type\":\"command\",\"request\":\"drive\",\"command\":{\"v\":1,\"kind\":\"DRIVE\","
                             "\"boot\":\"smoke-only\",\"generation\":0,\"lease\":1,\"client\":\"smoke-only\",\"seq\":2,"
                             "\"speed_mps\":0.5,\"steering_rad\":0.2,\"source_age_us\":0,\"ttl_us\":100000}}") &&
                pump_contains(220000, "\"accepted\":false") && receiver.target_speed() == 0. &&
                receiver.state() == aa::ReceiverState::kDisarmed);
  report.record(send(220000, "{\"type\":\"command\",\"request\":\"stop\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}") &&
                pump_contains(220000, "\"accepted\":true") && receiver.state() == aa::ReceiverState::kEstop);
  const auto fresh = receiver.lease_at(receiver.lease_count()-1);
  control("CLEAR", receiver.generation(), fresh.id);
  report.record(send(220000, scratch.text) && pump_contains(220000, "\"accepted\":false") &&
                receiver.state() == aa::ReceiverState::kEstop && !receiver.fresh_feedback(220000));
  f.sink.used = 0;
  f.sink.limit = 1;
  endpoint.pump(240000, f.sink);
  report.record(endpoint.tx().in_flight_remaining() > 0);
  endpoint.disconnect(240000);
  const auto discarded = endpoint.tx().stats().aborted_frames;
  f.sink.used = 0;
  endpoint.pump(240001, f.sink);
  report.record(discarded == 1 && endpoint.tx().in_flight_remaining() == 0 && f.sink.used == 0);
  report.record(endpoint.connect(240001).link == aa::LinkResult::kOk && receiver.state() == aa::ReceiverState::kEstop);
  report.record(endpoint.feed(240001, nullptr, aa::kMaxInputPerCall+1).link == aa::LinkResult::kFaultRaised &&
                receiver.target_speed() == 0. && endpoint.ingress().link().state() == aa::LinkState::kFaulted);
  report.record(endpoint.clear_link_fault(240002).link == aa::LinkResult::kOk &&
                endpoint.ingress().link().state() == aa::LinkState::kDisconnected && receiver.state() == aa::ReceiverState::kEstop);
  endpoint.connect(240003);
  endpoint.poll(240002);
  f.sink.used = 0;
  endpoint.pump(240004, f.sink);
  report.record(endpoint.ingress().link().state() == aa::LinkState::kFaulted &&
                endpoint.tx().fault() == aa::TxFault::kInvalidClock && f.sink.used == 0 && receiver.target_speed() == 0.);
  f.partial.connect(0);
  constexpr std::uint8_t partial_bytes[] = {'I', 'A', 100, 0, '{'};
  f.partial.feed(0, partial_bytes, sizeof(partial_bytes));
  f.partial.poll(30000);
  report.record(!f.partial.ingress().link().has_pending() && f.partial.receiver().state() == aa::ReceiverState::kDisarmed);
  f.sink_fault.connect(0);
  ViolatingSink bad_sink;
  f.sink_fault.pump(0, bad_sink);
  report.record(f.sink_fault.tx().fault() == aa::TxFault::kSinkContract &&
                f.sink_fault.receiver().target_speed() == 0. && !f.sink_fault.output_ready());
  return report;
}
}  // namespace arena_smoke
