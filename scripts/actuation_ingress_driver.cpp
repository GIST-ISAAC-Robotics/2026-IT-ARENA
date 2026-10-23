// PC 전용 입력/출력 어댑터. 동적 메모리·iostream은 MCU 연결부의 일부가 아니다.
#include "actuation_ingress.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace aa = arena_actuation;
int main() {
  std::unique_ptr<aa::ActuationReceiverCore> receiver;
  std::unique_ptr<aa::ActuationIngress> ingress;
  bool output_ready = false;  // 시험 입력. 실제 출력 감시가 아니다.
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    char op;
    if (!(input >> op)) return 2;
    if (op == 'I') {
      std::uint64_t timeout;
      int layout;
      if (!(input >> timeout >> layout)) return 2;
      aa::ReceiverConfig cfg;
      cfg.feedback_mode = aa::FeedbackMode::kMotorShaft;
      cfg.ticks_per_rev = 28; cfg.gear_ratio = 15.; cfg.wheel_radius_m = .0325;
      constexpr char name[] = "conformance-mcu-boot-a";
      std::uint32_t boot[sizeof(name) - 1];
      for (std::size_t i = 0; i < sizeof(name) - 1; ++i) boot[i] = name[i];
      ingress.reset();
      receiver = std::make_unique<aa::ActuationReceiverCore>(cfg, boot, sizeof(name) - 1);
      ingress = std::make_unique<aa::ActuationIngress>(*receiver, timeout, static_cast<aa::PayloadLayout>(layout));
      output_ready = false;
      continue;
    }
    if (!ingress || !receiver) return 2;
    std::int64_t now;
    if (!(input >> now)) return 2;
    std::ostringstream deliveries;
    bool first = true;
    auto observe = [&](const aa::IngressDelivery& d, const aa::Command& c) {
      if (!first) deliveries << ',';
      first = false;
      deliveries << "{\"origin\":" << static_cast<int>(d.origin)
        << ",\"parse\":" << static_cast<int>(d.parse_status)
        << ",\"called\":" << (d.receiver_called ? "true" : "false")
        << ",\"output_gate_rejected\":" << (d.output_gate_rejected ? "true" : "false")
        << ",\"result\":[" << (d.result.accepted ? "true" : "false") << ",\""
        << aa::receive_reason_name(d.result.reason) << "\"],\"request\":[";
      for (std::size_t i = 0; i < c.request.length && c.request.complete; ++i) {
        if (i) deliveries << ',';
        deliveries << c.request.code_points[i];
      }
      deliveries << "]}";
    };
    aa::IngressResult result{aa::LinkResult::kOk, ingress->fault()};
    bool link_op = true;
    switch (op) {
      case 'C': result = ingress->connect(now); break;
      case 'D': result = ingress->disconnect(now); break;
      case 'X': result = ingress->clear_link_fault(now); break;
      case 'P': result = ingress->poll(now, observe, output_ready); break;
      case 'R': { int ready; if (!(input >> ready)) return 2; output_ready = ready != 0; link_op = false; break; }
      case 'Z': {
        std::size_t n;
        if (!(input >> n)) return 2;
        result = ingress->feed(now, nullptr, n, observe, output_ready); break;
      }
      case 'F': {
        std::string hex;
        input >> hex;
        if (hex.size() % 2 != 0) return 2;
        std::vector<std::uint8_t> bytes;
        for (std::size_t i = 0; i < hex.size(); i += 2) {
          const int hi = aa::detail::hex_value(hex[i]), lo = aa::detail::hex_value(hex[i+1]);
          if (hi < 0 || lo < 0) return 2;
          bytes.push_back(static_cast<std::uint8_t>(16 * hi + lo));
        }
        result = ingress->feed(now, bytes.data(), bytes.size(), observe, output_ready); break;
      }
      case 'T': receiver->tick(now); link_op = false; break;
      case 'O': receiver->offer(now); link_op = false; break;
      case 'S': {
        int pressed;
        if (!(input >> pressed)) return 2;
        receiver->set_estop(pressed != 0, now); link_op = false; break;
      }
      case 'E': {
        std::int64_t seq, capture, count;
        if (!(input >> seq >> capture >> count)) return 2;
        receiver->update_motor_encoder(now, aa::IntInput::of(seq), aa::IntInput::of(capture), aa::IntInput::of(count));
        link_op = false; break;
      }
      default: return 2;
    }
    constexpr const char* states[] = {"DISARMED", "ARMED", "ACTIVE", "ESTOP_LATCHED", "FAULT_LATCHED"};
    const auto& r = *receiver;
    std::cout << std::setprecision(17) << "{\"state\":\"" << states[static_cast<int>(r.state())]
      << "\",\"reason\":\"" << r.reason() << "\",\"generation\":" << r.generation()
      << ",\"accepted_seq\":" << r.accepted_seq() << ",\"target_speed_mps\":" << r.target_speed()
      << ",\"steering_rad\":" << r.steering() << ",\"deadline_us\":"
      << (r.has_deadline() ? std::to_string(r.deadline()) : "null")
      << ",\"estop_held\":" << (r.estop_held() ? "true" : "false")
      << ",\"link_result\":" << (link_op ? static_cast<int>(result.link) : -1)
      << ",\"ingress_fault\":" << static_cast<int>(ingress->fault())
      << ",\"link_state\":" << static_cast<int>(ingress->link().state())
      << ",\"link_fault\":" << static_cast<int>(ingress->link().fault())
      << ",\"breaks\":" << ingress->link().session_breaks()
      << ",\"buffered\":" << ingress->link().buffered_bytes()
      << ",\"stop\":";
    if (result.stop_injected)
      std::cout << '[' << (result.stop_result.accepted ? "true" : "false") << ",\""
                << aa::receive_reason_name(result.stop_result.reason) << "\"]";
    else std::cout << "null";
    std::cout << ",\"deliveries\":[" << deliveries.str() << "]}\n";
  }
}
