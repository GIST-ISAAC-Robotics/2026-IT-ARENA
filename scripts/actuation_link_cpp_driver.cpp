// PC 대조 전용 CLI. 이 파일의 string/vector/memory/I/O는 MCU 코어의 일부가 아니다.
#include "actuation_link.hpp"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using Bytes = std::vector<std::uint8_t>;
using arena_actuation::FrameLink;
using arena_actuation::FrameOrigin;

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
// 10진 uint64만 허용한다. 부호/공백/초과는 입력 파일 오류다.
bool parse_u64(const std::string& text, std::uint64_t& value) {
  if (text.empty() || text.size() > 20) return false;
  value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') return false;
    const std::uint64_t digit = static_cast<std::uint64_t>(c-'0');
    if (value > (UINT64_MAX - digit) / 10) return false;
    value = value*10 + digit;
  }
  return true;
}
void hex(std::ostream& out, const std::uint8_t* data, std::size_t size) {
  constexpr char digits[]="0123456789abcdef";
  for (std::size_t i=0; i<size; ++i) out << digits[data[i]>>4] << digits[data[i]&15];
}
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> parts;
  std::size_t start=0;
  while (true) {
    const auto end=line.find(' ', start);
    parts.push_back(line.substr(start, end==std::string::npos ? std::string::npos : end-start));
    if (end==std::string::npos) return parts;
    start=end+1;
  }
}

int main() {
  static_assert(sizeof(FrameLink) <= 2048, "unexpected host link size");
  std::unique_ptr<FrameLink> link;
  std::string line;
  while (std::getline(std::cin, line)) {
    const auto parts=split(line);
    if (parts.empty() || parts[0].size()!=1) return 2;
    const char op=parts[0][0];
    std::uint64_t value=0;
    if (parts.size()<2 || !parse_u64(parts[1], value)) return 2;
    if (op=='N') {
      if (parts.size()!=2) return 2;
      link.reset(new FrameLink(value));
      std::cout << "N;" << int(link->state()) << ';' << int(link->fault()) << '\n';
      continue;
    }
    if (!link) return 2;
    std::vector<std::pair<FrameOrigin, Bytes>> frames;
    const auto collect=[&](const std::uint8_t* payload, std::size_t size, FrameOrigin origin) {
      frames.emplace_back(origin, Bytes(payload, payload+size));
    };
    arena_actuation::LinkResult result;
    if (op=='F') {
      if (parts.size()>3) return 2;
      Bytes data;
      if (!unhex(parts.size()==3 ? parts[2] : "", data)) return 2;
      result=link->feed(value, data.empty() ? nullptr : data.data(), data.size(), collect);
    } else if (op=='Z') {
      std::uint64_t size=0;
      if (parts.size()!=3 || !parse_u64(parts[2], size)) return 2;
      result=link->feed(value, nullptr, static_cast<std::size_t>(size), collect);
    } else if (parts.size()!=2) {
      return 2;
    } else if (op=='P') {
      result=link->poll(value, collect);
    } else if (op=='C') {
      result=link->connect(value);
    } else if (op=='D') {
      result=link->disconnect(value);
    } else if (op=='X') {
      result=link->clear_fault(value);
    } else {
      return 2;
    }
    std::ostringstream out;
    out << "L;" << int(result) << ';' << int(link->state()) << ';' << int(link->fault()) << ';'
        << int(link->last_fault()) << ';' << link->framing_errors() << ';' << link->assembly_timeouts()
        << ';' << link->rejected_calls() << ';' << link->dropped_bytes() << ';' << link->session_breaks()
        << ';' << link->sessions() << ';';
    if (link->has_anchor()) out << link->pending_since(); else out << '-';
    out << ';';
    hex(out, link->buffered_data(), link->buffered_bytes());
    out << ';';
    for (std::size_t i=0; i<frames.size(); ++i) {
      if (i) out << ',';
      out << (frames[i].first==FrameOrigin::kLive ? 'L' : 'X') << ':';
      hex(out, frames[i].second.data(), frames[i].second.size());
    }
    std::cout << out.str() << '\n';
  }
  return 0;
}
