// PC 대조 전용 CLI. 이 파일의 string/vector/I/O는 MCU 코어의 일부가 아니다.
#include "actuation_wire.hpp"
#include <iostream>
#include <string>
#include <vector>

using Bytes = std::vector<std::uint8_t>;
int nibble(char c) {
  if (c >= '0' && c <= '9') return c-'0';
  if (c >= 'a' && c <= 'f') return c-'a'+10;
  if (c >= 'A' && c <= 'F') return c-'A'+10;
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
void hex(const std::uint8_t* data, std::size_t size) {
  constexpr char digits[]="0123456789abcdef";
  for (std::size_t i=0; i<size; ++i)
    std::cout << digits[data[i]>>4] << digits[data[i]&15];
}
int main() {
  static_assert(sizeof(arena_actuation::FrameDecoder) <= 2048, "unexpected host decoder size");
  // 포인터/출력 공간 경계는 입력 파일과 별도로 확인한다.
  std::array<std::uint8_t, 9> tiny{};
  const std::uint8_t byte='x';
  if (arena_actuation::encode_frame(&byte,1,tiny.data(),8)!=0 ||
      arena_actuation::encode_frame(nullptr,1,tiny.data(),9)!=0 ||
      arena_actuation::encode_frame(&byte,1,nullptr,9)!=0) return 3;
  arena_actuation::FrameDecoder guard;
  const auto ignore=[](const std::uint8_t*,std::size_t) {};
  if (guard.feed(nullptr,1,ignore) || !guard.feed(nullptr,0,ignore) || guard.errors()!=1) return 3;
  arena_actuation::FrameDecoder decoder;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) return 2;
    Bytes data;
    if (!unhex(line.size()>1 ? line.substr(2) : "", data)) return 2;
    if (line[0]=='E') {
      std::array<std::uint8_t, arena_actuation::kMaxFrame> result{};
      const auto size=arena_actuation::encode_frame(data.data(),data.size(),result.data(),result.size());
      std::cout << "E;";
      if (size==0) std::cout << "REJECT"; else hex(result.data(),size);
      std::cout << '\n';
    } else if (line[0]=='C') {
      std::cout << "C;" << arena_actuation::crc32(data.data(),data.size()) << '\n';
    } else if (line[0]=='F' || line[0]=='R') {
      std::vector<Bytes> payloads;
      bool accepted=true;
      if (line[0]=='R') decoder.reset();
      else accepted=decoder.feed(data.data(),data.size(),[&](const std::uint8_t* payload,std::size_t size) {
        payloads.emplace_back(payload,payload+size);
      });
      std::cout << "D;" << decoder.errors() << ';' << (accepted ? 1 : 0) << ';';
      hex(decoder.buffered_data(),decoder.buffered_bytes());
      std::cout << ';';
      for (std::size_t i=0; i<payloads.size(); ++i) {
        if (i) std::cout << ',';
        hex(payloads[i].data(),payloads[i].size());
      }
      std::cout << '\n';
    } else return 2;
  }
  return 0;
}
