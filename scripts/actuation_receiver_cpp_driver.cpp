// PC 대조 전용 CLI. string/vector/표준 I/O·JSON 출력은 MCU 코어의 일부가 아니다.
#define ARENA_ACTUATION_TESTING 1
#include "actuation_receiver.hpp"
#include "actuation_link.hpp"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using Bytes = std::vector<std::uint8_t>;
namespace aa = arena_actuation;

int nibble(char c) {
  if (c >= '0' && c <= '9') return c-'0';
  if (c >= 'a' && c <= 'f') return c-'a'+10;
  return -1;
}
bool unhex(const std::string& text, Bytes& out) {
  if (text.size()%2) return false;
  for (std::size_t i=0; i<text.size(); i+=2) {
    const int h=nibble(text[i]), l=nibble(text[i+1]);
    if (h<0 || l<0) return false;
    out.push_back(static_cast<std::uint8_t>(16*h+l));
  }
  return true;
}
bool i64(const std::string& t, std::int64_t& v) {
  if (t.empty() || t.size() > 20) return false;
  std::size_t i = t[0]=='-' ? 1 : 0;
  if (i == t.size()) return false;
  std::uint64_t m = 0;
  for (; i<t.size(); ++i) {
    if (t[i]<'0' || t[i]>'9') return false;
    const std::uint64_t d = static_cast<std::uint64_t>(t[i]-'0');
    if (m > (UINT64_MAX-d)/10) return false;
    m = m*10+d;
  }
  if (t[0]=='-') { if (m > static_cast<std::uint64_t>(INT64_MAX)+1) return false;
                   v = m == static_cast<std::uint64_t>(INT64_MAX)+1 ? INT64_MIN : -static_cast<std::int64_t>(m); }
  else { if (m > static_cast<std::uint64_t>(INT64_MAX)) return false; v = static_cast<std::int64_t>(m); }
  return true;
}
aa::IntInput arg(const std::string& t) {  // "X" = Python integer() 판정 실패 입력
  std::int64_t v = 0;
  return t != "X" && i64(t, v) ? aa::IntInput::of(v) : aa::IntInput{};
}
double from_bits(const std::string& t) {
  std::uint64_t raw = 0;
  for (char c : t) raw = raw*16 + static_cast<std::uint64_t>(nibble(c));
  double v; std::memcpy(&v, &raw, 8); return v;
}
std::string num(double v) {
  char buf[40];
  std::snprintf(buf, sizeof buf, "%.17g", v);
  std::string s(buf);
  if (s.find_first_of(".eni") == std::string::npos) s += ".0";
  return s;
}
std::string jstr(const aa::StoredString& s) {
  std::ostringstream o;
  o << '"';
  char buf[16];
  for (std::size_t i=0; i<s.length; ++i) {
    std::uint32_t cp = s.code_points[i];
    auto unit = [&](std::uint32_t u) { std::snprintf(buf, sizeof buf, "\\u%04x", u); o << buf; };
    if (cp == '"' || cp == '\\') o << '\\' << static_cast<char>(cp);
    else if (cp >= 0x20 && cp < 0x7f) o << static_cast<char>(cp);
    else if (cp < 0x10000) unit(cp);
    else { cp -= 0x10000; unit(0xD800 + (cp >> 10)); unit(0xDC00 + (cp & 0x3FF)); }
  }
  o << '"';
  return o.str();
}
const char* state_name(aa::ReceiverState s) {
  constexpr const char* n[] = {"DISARMED", "ARMED", "ACTIVE", "ESTOP_LATCHED", "FAULT_LATCHED"};
  return n[static_cast<int>(s)];
}
std::string status(const aa::ActuationReceiverCore& r) {
  std::ostringstream o;
  const bool motor = r.motor_mode();
  auto opt = [&](bool has, std::int64_t v) { return has ? std::to_string(v) : std::string("null"); };
  o << "{\"v\":1,\"boot\":" << jstr(r.boot()) << ",\"generation\":" << r.generation()
    << ",\"state\":\"" << state_name(r.state()) << "\",\"reason\":\"" << r.reason() << "\",\"owner\":"
    << (r.has_owner() ? jstr(r.owner()) : "null") << ",\"accepted_seq\":" << r.accepted_seq()
    << ",\"target_speed_mps\":" << num(r.target_speed()) << ",\"steering_rad\":" << num(r.steering())
    << ",\"left_wheel_mps\":" << (motor ? "null" : num(r.left_mps()))
    << ",\"right_wheel_mps\":" << (motor ? "null" : num(r.right_mps()))
    << ",\"mean_wheel_mps\":" << num(r.mean_mps())
    << ",\"feedback_mode\":\"" << (motor ? "drive_motor_shaft" : "rear_wheel_pair_legacy")
    << "\",\"stationary_scope\":\"" << (motor ? "mean_wheel_only_opposite_rotation_unobservable" : "both_rear_wheels_low_speed")
    << "\",\"feedback_valid\":" << (r.fresh_feedback(r.has_now() ? r.now() : 0) ? "true" : "false")
    << ",\"feedback_seq\":" << opt(r.has_feedback(), r.feedback_seq())
    << ",\"left_ticks\":" << opt(r.has_feedback() && !motor, r.feedback_count(0))
    << ",\"right_ticks\":" << opt(r.has_feedback() && !motor, r.feedback_count(1))
    << ",\"motor_counts\":" << opt(r.has_feedback() && motor, r.feedback_count(0))
    << ",\"feedback_capture_us\":" << opt(r.has_feedback(), r.feedback_capture())
    << ",\"deadline_us\":" << opt(r.has_deadline(), r.deadline())
    << ",\"estop_held\":" << (r.estop_held() ? "true" : "false") << '}';
  return o.str();
}
std::string result_pair(const aa::ReceiveResult& x) {
  return std::string("[") + (x.accepted ? "true" : "false") + ",\"" + aa::receive_reason_name(x.reason) + "\"]";
}

int main() {
  std::unique_ptr<aa::ActuationReceiverCore> rx, saved;
  auto parser = std::make_unique<aa::CommandParser>();
  aa::Command command;
  std::unique_ptr<aa::FrameLink> link;
  std::uint64_t breaks_seen = 0;
  std::string text;
  const aa::Command stop_command = [] {
    aa::Command c;
    const char stop[] = "{\"v\":1,\"kind\":\"STOP\"}";
    aa::CommandParser p;
    p.parse(reinterpret_cast<const std::uint8_t*>(stop), sizeof stop - 1, aa::PayloadLayout::kBareCommand, c);
    return c;
  }();
  while (std::getline(std::cin, text)) {
    std::istringstream in(text);
    std::vector<std::string> f;
    for (std::string w; in >> w;) f.push_back(w);
    if (f.empty()) return 2;
    std::string result = "null";
    std::int64_t now = 0;
    if (f[0] != "G" && f[0] != "V" && f[0] != "U" && f[0] != "Q" && (f.size() < 2 || !i64(f[1], now))) return 2;
    if (f[0] == "G") {
      if (f.size() != 16) return 2;
      aa::ReceiverConfig c;
      c.feedback_mode = f[1] == "1" ? aa::FeedbackMode::kLegacyRearPair : aa::FeedbackMode::kMotorShaft;
      if (!i64(f[2], c.lease_us) || !i64(f[3], c.feedback_timeout_us) || !i64(f[4], c.settle_us) ||
          !i64(f[5], c.max_control_gap_us) || !i64(f[6], c.ticks_per_rev) || !i64(f[7], c.motor_no_progress_us)) return 2;
      c.wheel_radius_m = from_bits(f[8]); c.max_speed_mps = from_bits(f[9]); c.max_steering_rad = from_bits(f[10]);
      c.stopped_wheel_mps = from_bits(f[11]); c.max_wheel_mps = from_bits(f[12]); c.gear_ratio = from_bits(f[13]);
      c.motor_progress_min_command_mps = from_bits(f[14]);
      if (!c.valid()) return 3;
      Bytes boot;
      if (!unhex(f[15], boot)) return 2;
      std::vector<std::uint32_t> cps(boot.begin(), boot.end());  // ASCII boot id
      rx = std::make_unique<aa::ActuationReceiverCore>(c, cps.data(), cps.size());
      link = std::make_unique<aa::FrameLink>(UINT64_MAX);
      if (link->connect(0) != aa::LinkResult::kOk) return 2;
      breaks_seen = link->session_breaks();
    } else if (!rx) {
      return 2;
    } else if (f[0] == "V") {  // PC 시험 전용: 상태 저장(같은 상태에서 여러 명령 비교)
      saved = std::make_unique<aa::ActuationReceiverCore>(*rx);
    } else if (f[0] == "U") {
      if (!saved) return 2;
      rx = std::make_unique<aa::ActuationReceiverCore>(*saved);
    } else if (f[0] == "Q" && f.size() == 2) {  // 링크 정책 사례: 새 링크(미연결), 시험 timeout
      std::int64_t timeout = 0;
      if (!i64(f[1], timeout)) return 2;
      link = std::make_unique<aa::FrameLink>(static_cast<std::uint64_t>(timeout));
      breaks_seen = 0;
    } else if (f[0] == "T") {
      rx->tick(now);
      result = "\"STATUS\"";
    } else if (f[0] == "E" && f.size() == 5) {
      result = rx->update_motor_encoder(now, arg(f[2]), arg(f[3]), arg(f[4])) ? "true" : "false";
    } else if (f[0] == "Y" && f.size() == 6) {
      result = rx->update_legacy_encoder(now, arg(f[2]), arg(f[3]), arg(f[4]), arg(f[5])) ? "true" : "false";
    } else if (f[0] == "S" && f.size() == 3) {
      rx->set_estop(f[2] == "1", now);
    } else if (f[0] == "O") {
      const auto offer = rx->offer(now);
      result = offer.ok ? "{\"lease\":" + std::to_string(offer.lease.id) + ",\"lease_issued_us\":" +
                          std::to_string(offer.lease.issued_us) + ",\"lease_expires_us\":" +
                          std::to_string(offer.lease.expires_us) + "}" : "\"OFFER_ERROR\"";
    } else if (f[0] == "Z" && f.size() == 4) {  // PC 경계 시험 전용 카운터 주입
      std::int64_t g = 0, l = 0;
      if (!i64(f[2], g) || !i64(f[3], l)) return 2;
      rx->test_set_counters(g, l);
    } else if (f[0] == "M" && f.size() >= 3) {
      // 완성 payload 하나. Python Decoder처럼 JSON 객체가 아니면 receive를 호출하지 않는다.
      Bytes data;
      if (!unhex(f.size() > 3 ? f[3] : "", data)) return 2;
      const auto layout = f[2] == "1" ? aa::PayloadLayout::kEnvelope : aa::PayloadLayout::kBareCommand;
      parser->parse(data.empty() ? nullptr : data.data(), data.size(), layout, command);
      const auto cls = aa::classify(command);
      if (command.status != aa::ParseStatus::kObject) result = "\"NOT_DELIVERED\"";
      else if (cls == aa::CommandClass::kEnvelopeIgnored) result = "\"ENVELOPE_IGNORED\"";
      else result = result_pair(rx->receive(command, now));
    } else if (f[0] == "K" || f[0] == "F" || f[0] == "N" || f[0] == "D" || f[0] == "P" || f[0] == "R" || f[0] == "X" ||
               f[0] == "W") {
      // FrameLink → CommandParser → 수신기. callback 안에서는 표현만 복사하고 receive는 같은 작업에서 순서대로 호출한다.
      std::vector<aa::Command> queued;
      auto collect = [&](const std::uint8_t* p, std::size_t n, aa::FrameOrigin) {
        aa::Command c;
        parser->parse(p, n, aa::PayloadLayout::kBareCommand, c);
        queued.push_back(c);
      };
      const auto t = static_cast<std::uint64_t>(now);
      if (f[0] == "F") {  // 한 번의 feed 호출(호출량 예산 위반도 그대로 전달)
        Bytes data;
        if (!unhex(f.size() > 2 ? f[2] : "", data)) return 2;
        link->feed(t, data.empty() ? nullptr : data.data(), data.size(), collect);
      } else if (f[0] == "K") {  // 전송 사례: 운영 Decoder처럼 큰 입력을 256 B 호출로 나눈다
        Bytes data;
        if (!unhex(f.size() > 2 ? f[2] : "", data)) return 2;
        for (std::size_t s = 0; s < data.size(); s += aa::kMaxInputPerCall)
          link->feed(t, data.data()+s, std::min(aa::kMaxInputPerCall, data.size()-s), collect);
        if (data.empty()) link->feed(t, nullptr, 0, collect);
      } else if (f[0] == "N") link->connect(t);
      else if (f[0] == "D") link->disconnect(t);
      else if (f[0] == "P") link->poll(t, collect);
      else if (f[0] == "X") link->clear_fault(t);
      else if (f[0] == "W") {
        std::int64_t n = 0;
        if (f.size() != 3 || !i64(f[2], n) || n < 0) return 2;
        link->feed(t, nullptr, static_cast<std::size_t>(n), collect);
      } else { link->connect(t); breaks_seen = link->session_breaks(); }  // parser reset만: 정책 비적용
      std::ostringstream o;
      o << '[';
      bool first = true;
      for (const auto& c : queued) {
        if (c.status != aa::ParseStatus::kObject) continue;  // Python Decoder는 비JSON 프레임을 전달하지 않는다.
        if (!first) o << ',';
        first = false;
        o << '[' << result_pair(rx->receive(c, now)) << ',' << status(*rx) << ']';
      }
      o << ']';
      // 세션 경계(단절·놓친 단절·링크 고장)는 참조 정책과 같이 로컬 STOP으로 고정한다(사유는 software_stop 유지).
      std::string injected = "null";
      if (link->session_breaks() > breaks_seen) {
        breaks_seen = link->session_breaks();
        if (rx->state() != aa::ReceiverState::kEstop && rx->state() != aa::ReceiverState::kFault)
          injected = result_pair(rx->receive(stop_command, now));
      }
      result = "{\"deliveries\":" + o.str() + ",\"injected_stop\":" + injected + ",\"buffered\":" +
               std::to_string(link->buffered_bytes()) + "}";
    } else {
      return 2;
    }
    std::cout << "{\"r\":" << result << ",\"s\":" << status(*rx) << "}\n";
  }
  return 0;
}
