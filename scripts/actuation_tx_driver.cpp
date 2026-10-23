// PC 전용 lock-step 어댑터: 입력 한 줄마다 JSON 한 줄을 출력하고 flush한다.
// 동적 메모리·iostream은 MCU 송신 정책의 일부가 아니다. 선로 시간 모델은 Python 쪽이 소유한다.
#include "actuation_tx.hpp"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace aa = arena_actuation;
namespace {

const char* kind_name(aa::TxKind k) {
  constexpr const char* names[] = {"none", "ack", "state", "offer", "diag"};
  return names[static_cast<int>(k)];
}

void put_hex(std::ostream& out, const std::uint8_t* data, std::size_t n) {
  constexpr char digits[] = "0123456789abcdef";
  for (std::size_t i = 0; i < n; ++i) out << digits[data[i] >> 4] << digits[data[i] & 0xF];
}

void put_stored(std::ostream& out, const aa::StoredString& s) {
  out << '[';
  for (std::size_t i = 0; i < s.length && s.complete; ++i) out << (i ? "," : "") << s.code_points[i];
  out << ']';
}

void put_stats(std::ostream& out, const aa::TxStats& s) {
  const auto arr = [&](const char* key, const auto& values) {
    out << ",\"" << key << "\":[";
    for (std::size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << values[i];
    out << ']';
  };
  out << "{\"ack_enqueued\":" << s.ack_enqueued << ",\"ack_dropped_overflow\":" << s.ack_dropped_overflow
      << ",\"ack_evicted_for_stop\":" << s.ack_evicted_for_stop << ",\"ack_purged_session\":" << s.ack_purged_session
      << ",\"ack_queue_max\":" << s.ack_queue_max << ",\"state_changes\":" << s.state_changes
      << ",\"state_merged\":" << s.state_merged << ",\"offers_urgent\":" << s.offers_urgent
      << ",\"offer_unavailable\":" << s.offer_unavailable << ",\"diag_skipped_periods\":" << s.diag_skipped_periods
      << ",\"diag_deferred_for_offer\":" << s.diag_deferred_for_offer
      << ",\"aborted_frames\":" << s.aborted_frames << ",\"aborted_bytes\":" << s.aborted_bytes
      << ",\"encode_failures\":" << s.encode_failures << ",\"sink_zero_writes\":" << s.sink_zero_writes
      << ",\"sink_partial_writes\":" << s.sink_partial_writes << ",\"starvation_guard_used\":" << s.starvation_guard_used;
  arr("frames", s.frames);
  arr("completed", s.completed);
  arr("bytes", s.bytes);
  arr("max_dwell_us", s.max_dwell_us);
  arr("max_inflight_us", s.max_inflight_us);
  out << '}';
}

bool parse_hex(const std::string& hex, std::vector<std::uint8_t>& bytes) {
  if (hex.size() % 2 != 0) return false;
  for (std::size_t i = 0; i < hex.size(); i += 2) {
    const int hi = aa::detail::hex_value(static_cast<std::uint8_t>(hex[i]));
    const int lo = aa::detail::hex_value(static_cast<std::uint8_t>(hex[i+1]));
    if (hi < 0 || lo < 0) return false;
    bytes.push_back(static_cast<std::uint8_t>(16 * hi + lo));
  }
  return true;
}

}  // namespace

int main() {
  std::unique_ptr<aa::ActuationReceiverCore> receiver;
  std::unique_ptr<aa::ActuationIngress> ingress;
  std::unique_ptr<aa::ActuationTxScheduler> tx;
  std::uint64_t breaks_seen = 0;
  bool output_ready = false;  // 시험 입력. 매 수신 호출에 명시적으로 전달한다.
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    char op;
    if (!(input >> op)) return 2;
    std::ostringstream out;
    out << std::setprecision(17);
    if (op == 'I') {
      // I offer_period diag_period max_consecutive line_ns_per_byte link_timeout layout cp,cp,...
      std::int64_t offer_period, diag_period, ns_per_byte;
      int max_consecutive, layout;
      std::uint64_t timeout;
      std::string boot_text;
      if (!(input >> offer_period >> diag_period >> max_consecutive >> ns_per_byte >> timeout >> layout >> boot_text))
        return 2;
      std::vector<std::uint32_t> boot;
      std::istringstream cps(boot_text);
      std::string item;
      while (std::getline(cps, item, ',')) boot.push_back(static_cast<std::uint32_t>(std::stoul(item)));
      aa::ReceiverConfig cfg;
      cfg.feedback_mode = aa::FeedbackMode::kMotorShaft;
      cfg.ticks_per_rev = 28; cfg.gear_ratio = 15.; cfg.wheel_radius_m = .0325;
      tx.reset();
      ingress.reset();
      receiver = std::make_unique<aa::ActuationReceiverCore>(cfg, boot.data(), boot.size());
      ingress = std::make_unique<aa::ActuationIngress>(*receiver, timeout, static_cast<aa::PayloadLayout>(layout));
      aa::TxConfig tx_cfg;
      tx_cfg.offer_period_us = offer_period;
      tx_cfg.diag_period_us = diag_period;
      tx_cfg.max_consecutive_events = static_cast<std::uint8_t>(max_consecutive);
      tx_cfg.line_ns_per_byte = ns_per_byte;
      tx = std::make_unique<aa::ActuationTxScheduler>(tx_cfg, *receiver);
      tx->set_link_up(false);
      breaks_seen = 0;
      output_ready = false;
      out << "{\"init\":true,\"tx_fault\":" << static_cast<int>(tx->fault()) << ",\"worst_payload\":{";
      const aa::TxKind kinds[] = {aa::TxKind::kAck, aa::TxKind::kState, aa::TxKind::kOffer, aa::TxKind::kDiagnostic};
      for (std::size_t i = 0; i < 4; ++i)
        out << (i ? "," : "") << '"' << kind_name(kinds[i]) << "\":" << tx->worst_payload(kinds[i]);
      out << "},\"sizeof_tx\":" << sizeof(aa::ActuationTxScheduler) << "}\n";
      std::cout << out.str() << std::flush;
      continue;
    }
    if (!tx) return 2;
    std::int64_t now;
    if (!(input >> now)) return 2;
    std::ostringstream deliveries;
    bool first = true;
    auto observe = [&](const aa::IngressDelivery& d, const aa::Command& c) {
      tx->on_delivery(d, c, now);
      deliveries << (first ? "" : ",") << "{\"origin\":" << static_cast<int>(d.origin)
                 << ",\"called\":" << (d.receiver_called ? "true" : "false")
                 << ",\"output_gate_rejected\":" << (d.output_gate_rejected ? "true" : "false")
                 << ",\"class\":" << static_cast<int>(d.classification)
                 << ",\"result\":[" << (d.result.accepted ? "true" : "false") << ",\""
                 << aa::receive_reason_name(d.result.reason) << "\"],\"request\":";
      put_stored(deliveries, c.request);
      deliveries << '}';
      first = false;
    };
    aa::TxPumpResult pump;
    bool pumped = false;
    std::vector<std::uint8_t> written;
    switch (op) {
      case 'C': ingress->connect(now); break;
      case 'D': ingress->disconnect(now); break;
      case 'X': ingress->clear_link_fault(now); break;
      case 'P': ingress->poll(now, observe, output_ready); break;
      case 'F': {
        std::string hex;
        std::vector<std::uint8_t> bytes;
        input >> hex;
        if (!parse_hex(hex, bytes)) return 2;
        ingress->feed(now, bytes.data(), bytes.size(), observe, output_ready);
        break;
      }
      case 'T': receiver->tick(now); break;
      case 'S': { int pressed; if (!(input >> pressed)) return 2; receiver->set_estop(pressed != 0, now); break; }
      case 'R': { int ready; if (!(input >> ready)) return 2; output_ready = ready != 0;
                  tx->set_output_ready(output_ready); break; }
      case 'E': {
        std::int64_t seq, capture, count;
        if (!(input >> seq >> capture >> count)) return 2;
        receiver->update_motor_encoder(now, aa::IntInput::of(seq), aa::IntInput::of(capture), aa::IntInput::of(count));
        break;
      }
      case 'W': case 'Z': {
        // W: sink가 n바이트까지 수락. Z: 계약 위반 sink(요청보다 1 큰 값을 반환).
        std::size_t capacity;
        if (!(input >> capacity)) return 2;
        const bool violate = op == 'Z';
        pump = tx->pump(now, [&](const std::uint8_t* data, std::size_t size) -> std::size_t {
          if (violate) return size + 1;
          const std::size_t n = size < capacity ? size : capacity;
          written.insert(written.end(), data, data + n);
          capacity -= n;
          return n;
        });
        pumped = true;
        break;
      }
      default: return 2;
    }
    // 링크 세션 경계는 ingress가 이미 로컬 STOP을 넣었다. 송신 쪽은 꼬리 폐기·ack 비움만 한다.
    if (ingress->link().session_breaks() != breaks_seen) {
      breaks_seen = ingress->link().session_breaks();
      tx->on_session_break(now);
    }
    tx->set_link_up(ingress->link().state() == aa::LinkState::kConnected);
    tx->observe(now);

    const auto& r = *receiver;
    out << "{\"now\":" << now << ",\"state\":\"" << aa::tx_detail::state_name(r.state()) << "\",\"reason\":\""
        << r.reason() << "\",\"generation\":" << r.generation() << ",\"accepted_seq\":" << r.accepted_seq()
        << ",\"target_speed_mps\":" << r.target_speed() << ",\"steering_rad\":" << r.steering()
        << ",\"mean_wheel_mps\":" << r.mean_mps()
        << ",\"deadline_us\":" << (r.has_deadline() ? std::to_string(r.deadline()) : "null")
        << ",\"estop_held\":" << (r.estop_held() ? "true" : "false")
        << ",\"owner\":";
    if (r.has_owner()) put_stored(out, r.owner()); else out << "null";
    out << ",\"leases\":[";
    for (std::size_t i = 0; i < r.lease_count(); ++i) {
      const auto& l = r.lease_at(i);
      out << (i ? "," : "") << '[' << l.id << ',' << l.issued_us << ',' << l.expires_us << ']';
    }
    out << "],\"feedback_seq\":" << (r.has_feedback() ? std::to_string(r.feedback_seq()) : "null")
        << ",\"feedback_capture_us\":" << (r.has_feedback() ? std::to_string(r.feedback_capture()) : "null")
        << ",\"motor_counts\":" << (r.has_feedback() ? std::to_string(r.feedback_count(0)) : "null")
        << ",\"deadline_miss_count\":" << r.deadline_miss_count()
        << ",\"link_state\":" << static_cast<int>(ingress->link().state())
        << ",\"breaks\":" << ingress->link().session_breaks()
        << ",\"tx_fault\":" << static_cast<int>(tx->fault()) << ",\"ack_count\":" << tx->ack_count()
        << ",\"state_pending\":" << (tx->state_pending() ? "true" : "false")
        << ",\"urgent_offer\":" << (tx->urgent_offer_pending() ? "true" : "false")
        << ",\"in_flight_remaining\":" << tx->in_flight_remaining() << ",\"stats\":";
    put_stats(out, tx->stats());
    out << ",\"deliveries\":[" << deliveries.str() << ']';
    if (pumped) {
      out << ",\"written\":\"";
      put_hex(out, written.data(), written.size());
      out << "\",\"completed\":" << pump.completed << ",\"backpressure\":" << (pump.backpressure ? "true" : "false")
          << ",\"frames\":[";
      for (std::size_t i = 0; i < pump.materialized; ++i) {
        const auto& f = pump.frames[i];
        out << (i ? "," : "") << "{\"kind\":\"" << kind_name(f.kind) << "\",\"id\":" << f.frame_id
            << ",\"size\":" << f.size << ",\"trigger_us\":" << f.trigger_us << ",\"materialize_us\":" << f.materialize_us
            << ",\"generation\":" << f.generation << ",\"lease\":" << f.lease
            << ",\"urgent\":" << (f.urgent ? "true" : "false") << ",\"stop_ack\":" << (f.stop_ack ? "true" : "false") << '}';
      }
      out << ']';
    }
    out << "}\n";
    std::cout << out.str() << std::flush;
  }
  return 0;
}
