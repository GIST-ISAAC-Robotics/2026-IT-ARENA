// PC 대조 전용 CLI. 이 파일의 string/vector/표준 I/O는 MCU 코어의 일부가 아니다.
#include "actuation_command.hpp"
#include "actuation_link.hpp"
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
bool unhex(const std::string& text, Bytes& output) {
  if (text.size()%2) return false;
  for (std::size_t i=0; i<text.size(); i+=2) {
    const int high=nibble(text[i]), low=nibble(text[i+1]);
    if (high<0 || low<0) return false;
    output.push_back(static_cast<std::uint8_t>(16*high+low));
  }
  return true;
}
bool parse_u64(const std::string& text, std::uint64_t& value, int base=10) {
  if (text.empty() || text.size() > 20) return false;
  value = 0;
  for (char c : text) {
    const int digit = base==16 ? nibble(c) : (c>='0' && c<='9' ? c-'0' : -1);
    if (digit < 0) return false;
    if (value > (UINT64_MAX - static_cast<std::uint64_t>(digit)) / static_cast<std::uint64_t>(base)) return false;
    value = value*static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(digit);
  }
  return true;
}
std::string bits(double value) {
  std::uint64_t raw;
  std::memcpy(&raw, &value, sizeof raw);
  std::ostringstream out;
  out.width(16); out.fill('0');
  out << std::hex << raw;
  return out.str();
}
std::string value_text(const aa::JsonValue& v) {
  switch (v.type) {
    case aa::ValueType::kAbsent: return "A";
    case aa::ValueType::kNull: return "N";
    case aa::ValueType::kBool: return v.boolean ? "T" : "F";
    case aa::ValueType::kInt: return "I" + std::to_string(v.integer);
    case aa::ValueType::kBigInt: return v.negative ? "B-" : "B+";
    case aa::ValueType::kFloat: return "D" + bits(v.real);
    case aa::ValueType::kString: return "S" + std::to_string(v.length);
    case aa::ValueType::kArray: return "R";
    case aa::ValueType::kObject: return "O";
  }
  return "?";
}
std::string stored_text(const aa::StoredString& s) {
  if (!s.is_string) return "-";
  std::ostringstream out;
  out << s.length << ':';
  if (!s.complete) { out << '*'; return out.str(); }
  out << std::hex;
  for (std::size_t i=0; i<s.length; ++i) { out.width(6); out.fill('0'); out << s.code_points[i]; }
  return out.str();
}
std::string line(const aa::Command& c, const aa::DriveLimits& limits) {
  std::ostringstream out;
  out << "P;" << int(c.status);
  if (c.status != aa::ParseStatus::kObject) return out.str();
  out << ';' << int(c.layout) << ';' << int(c.envelope_type_command) << ';' << int(c.envelope_command_object)
      << ';' << stored_text(c.request) << ';' << c.present << ';' << int(c.unknown_key) << ';' << int(c.kind);
  for (const auto& field : c.fields) out << ';' << value_text(field);
  const bool drive = aa::drive_values_ok(c, limits);
  out << ';' << stored_text(c.boot) << ';' << stored_text(c.client) << ';' << int(aa::classify(c))
      << ';' << int(aa::session_shape_ok(c)) << ';' << int(drive) << ';';
  if (drive) out << 'D' << bits(aa::numeric_value(c.field(aa::CommandField::kSpeed))) << ';'
                 << 'D' << bits(aa::numeric_value(c.field(aa::CommandField::kSteering)));
  else out << "-;-";
  return out.str();
}

int main() {
  static_assert(sizeof(aa::CommandParser) <= 8192, "unexpected host parser size");
  static_assert(sizeof(aa::Command) <= 2048, "unexpected host command size");
  aa::DriveLimits limits{2.5, .37, 100000};
  auto parser = std::make_unique<aa::CommandParser>();
  aa::Command command;  // 모든 입력에서 같은 객체를 재사용해 이전 결과 누출을 검사한다.
  auto link = std::make_unique<aa::FrameLink>(UINT64_MAX);
  if (link->connect(0) != aa::LinkResult::kOk) return 3;
  // 경계 API: null/0/초과 크기는 내용과 무관하게 kInvalidSize이며 이전 결과를 지운다.
  const std::uint8_t probe[] = {'{', '}'};
  if (parser->parse(probe, 2, aa::PayloadLayout::kBareCommand, command) != aa::ParseStatus::kObject ||
      parser->parse(nullptr, 2, aa::PayloadLayout::kBareCommand, command) != aa::ParseStatus::kInvalidSize ||
      parser->parse(probe, 0, aa::PayloadLayout::kBareCommand, command) != aa::ParseStatus::kInvalidSize ||
      parser->parse(probe, aa::kMaxPayload + 1, aa::PayloadLayout::kBareCommand, command) != aa::ParseStatus::kInvalidSize ||
      command.present != 0)
    return 3;
  std::string text;
  while (std::getline(std::cin, text)) {
    std::istringstream fields(text);
    std::string op, a, b, c;
    fields >> op;
    if (op == "L") {
      std::uint64_t speed=0, steer=0, lease=0;
      if (!(fields >> a >> b >> c) || !parse_u64(a, speed, 16) || !parse_u64(b, steer, 16) || !parse_u64(c, lease)) return 2;
      std::memcpy(&limits.max_speed_mps, &speed, 8);
      std::memcpy(&limits.max_steering_rad, &steer, 8);
      limits.lease_us = static_cast<std::int64_t>(lease);
      std::cout << "L;" << int(aa::limits_supported(limits)) << '\n';
    } else if (op == "P" || op == "W") {
      if (!(fields >> a)) return 2;
      fields >> b;  // 빈 payload 허용
      if (a != "0" && a != "1") return 2;
      const auto layout = a == "1" ? aa::PayloadLayout::kEnvelope : aa::PayloadLayout::kBareCommand;
      Bytes data;
      if (!unhex(b, data)) return 2;
      if (op == "P") {
        parser->parse(data.empty() ? nullptr : data.data(), data.size(), layout, command);
        std::fill(data.begin(), data.end(), 0xA5);  // 결과가 입력 버퍼를 참조하지 않음을 확인
        std::cout << line(command, limits) << '\n';
      } else {
        // FrameLink 콜백 안에서만 payload를 읽고 고정 Command로 복사한다.
        std::vector<std::string> lines;
        for (std::size_t start=0; start<data.size(); start+=aa::kMaxInputPerCall) {
          const std::size_t size = std::min(aa::kMaxInputPerCall, data.size()-start);
          if (link->feed(0, data.data()+start, size, [&](const std::uint8_t* payload, std::size_t n, aa::FrameOrigin) {
                parser->parse(payload, n, layout, command);
                lines.push_back(line(command, limits));
              }) != aa::LinkResult::kOk) return 2;
        }
        for (const auto& item : lines) std::cout << item << '\n';
        std::cout << "W;" << lines.size() << '\n';
      }
    } else if (op == "S") {
      // 호스트 빌드의 고정 객체 크기(MCU 크기는 툴체인/정렬에 따라 다르다).
      std::cout << "S;" << sizeof(aa::CommandParser) << ';' << sizeof(aa::Command) << ';'
                << sizeof(aa::FrameLink) << '\n';
    } else if (op == "R") {
      if (link->connect(0) != aa::LinkResult::kOk) return 2;
      std::cout << "R\n";
    } else {
      return 2;
    }
  }
  return 0;
}
