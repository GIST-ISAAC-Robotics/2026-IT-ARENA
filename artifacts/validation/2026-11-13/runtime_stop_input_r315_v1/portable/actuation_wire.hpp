#pragma once
// 보드 독립 wire 프레이밍 코어. JSON 검증/수신 상태기계/타이머/전기 출력은 포함하지 않는다.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace arena_actuation {
constexpr std::size_t kMaxPayload = 1536;
constexpr std::size_t kFrameOverhead = 8;
constexpr std::size_t kMaxFrame = kMaxPayload + kFrameOverhead;
constexpr std::size_t kMaxInputPerCall = 256;

inline std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

// payload는 호출자가 검증할 JSON 원문이다. 출력 공간은 별도 버퍼여야 한다.
inline std::size_t encode_frame(const std::uint8_t* payload, std::size_t size,
                                std::uint8_t* output, std::size_t capacity) {
  if (payload == nullptr || output == nullptr || size == 0 || size > kMaxPayload ||
      capacity < size + kFrameOverhead) return 0;
  output[0] = 'I'; output[1] = 'A';
  output[2] = static_cast<std::uint8_t>(size & 0xFFu);
  output[3] = static_cast<std::uint8_t>((size >> 8) & 0xFFu);
  std::memcpy(output + 4, payload, size);
  const auto crc = crc32(output + 2, size + 2);
  for (unsigned i = 0; i < 4; ++i)
    output[4 + size + i] = static_cast<std::uint8_t>((crc >> (8*i)) & 0xFFu);
  return size + kFrameOverhead;
}

class FrameDecoder {
 public:
  // callback은 동기 호출이며 payload 포인터를 보관하거나 이 decoder에 재진입하면 안 된다.
  // CRC 성공은 JSON/명령 성공이 아니다. callback 쪽에 별도 엄격한 검증이 필요하다.
  template<class Callback>
  bool feed(const std::uint8_t* data, std::size_t size, Callback&& callback) {
    if (size > kMaxInputPerCall || (size != 0 && data == nullptr) ||
        size > buffer_.size() - used_) {
      used_ = 0;
      ++errors_;
      return false;  // 호출자 작업량 위반. 잘라 받아 정상 프레임으로 처리하지 않는다.
    }
    if (size != 0) std::memcpy(buffer_.data() + used_, data, size);
    used_ += size;
    process(callback);
    return true;
  }

  // 상위 정책이 머리 후보를 손상으로 판정할 때(예: 조립 기한 초과) 사용한다. 새 바이트는 붙이지 않고
  // 머리 2 B(남은 1 B)를 버린 뒤 CRC 실패 때와 같은 규칙으로 남은 바이트를 다시 검사한다.
  template<class Callback>
  void reject_head(Callback&& callback) {
    if (used_ == 0) return;
    consume(used_ < 2 ? used_ : 2);
    ++errors_;
    process(callback);
  }

  void reset() { used_ = 0; }
  std::size_t buffered_bytes() const { return used_; }
  const std::uint8_t* buffered_data() const { return buffer_.data(); }
  std::uint64_t errors() const { return errors_; }

 private:
  template<class Callback>
  void process(Callback& callback) {
    while (used_ != 0) {
      std::size_t start = 0;
      while (start + 1 < used_ &&
             !(buffer_[start] == 'I' && buffer_[start+1] == 'A')) ++start;
      if (start + 1 >= used_) {
        const bool keep_i = buffer_[used_-1] == 'I';
        if (keep_i) buffer_[0] = 'I';
        used_ = keep_i ? 1 : 0;
        break;
      }
      if (start != 0) { consume(start); ++errors_; }
      if (used_ < 4) break;
      const auto payload_size = static_cast<std::size_t>(buffer_[2]) |
                                (static_cast<std::size_t>(buffer_[3]) << 8);
      if (payload_size == 0 || payload_size > kMaxPayload) {
        consume(2); ++errors_; continue;
      }
      const auto total = payload_size + kFrameOverhead;
      if (used_ < total) break;
      std::uint32_t expected = 0;
      for (unsigned i = 0; i < 4; ++i)
        expected |= static_cast<std::uint32_t>(buffer_[4+payload_size+i]) << (8*i);
      if (crc32(buffer_.data()+2, payload_size+2) != expected) {
        consume(2); ++errors_; continue;
      }
      callback(buffer_.data()+4, payload_size);
      consume(total);
    }
  }

  void consume(std::size_t size) {
    used_ -= size;
    if (used_ != 0) std::memmove(buffer_.data(), buffer_.data()+size, used_);
  }
  std::array<std::uint8_t, kMaxFrame + kMaxInputPerCall> buffer_{};
  std::size_t used_ = 0;
  std::uint64_t errors_ = 0;
};
}  // namespace arena_actuation
