#pragma once
// 보드 독립 명령 표현 parser(PC 검증 후보). CRC가 맞은 payload 다음 단계로서 기존 Python
// actuation_wire.Decoder의 JSON 판정(엄격 UTF-8·JSON 문법·중복 키·NaN/Infinity 거절·최상위 객체)과
// ActuationReceiver.receive가 읽는 명령 필드의 타입/값을 고정 메모리로 표현한다.
// 수신 상태기계(시계·세대·임대·소유권·순번·만료·상태 전이)는 포함하지 않는다. 출력은 구동 허가가 아니다.
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <system_error>

#include "actuation_wire.hpp"

namespace arena_actuation {

// 0 이외의 값은 Python Decoder가 receive에 전달하지 않는 payload다(오류 종류는 Python 판정 순서를 따른다).
enum class ParseStatus : std::uint8_t {
  kObject = 0, kInvalidSize = 1, kInvalidUtf8 = 2, kSyntax = 3, kNonFiniteConstant = 4,
  kDuplicateKey = 5, kNotObject = 6,
};
// kInt: int64 범위 정수, kBigInt: 그 밖의 JSON 정수(부호만 보존). Python bool은 int와 구분한다.
enum class ValueType : std::uint8_t {
  kAbsent = 0, kNull = 1, kBool = 2, kInt = 3, kBigInt = 4, kFloat = 5, kString = 6, kArray = 7, kObject = 8,
};
enum class CommandField : std::uint8_t {
  kV = 0, kKind, kBoot, kGeneration, kLease, kClient, kSeq, kSpeed, kSteering, kSourceAge, kTtl, kCount,
};
enum class KindName : std::uint8_t { kNotString = 0, kOther = 1, kStop, kArm, kClear, kDisarm, kDrive };
enum class PayloadLayout : std::uint8_t { kBareCommand = 0, kEnvelope = 1 };
// receive가 상태를 보기 전 내용만으로 갈리는 분기. 그 뒤 판정(세션 일치·임대·준비·소유권·순번·
// DRIVE 값→invalid_drive 고장·원천 만료)은 상태기계가 기존 순서대로 수행해야 한다.
enum class CommandClass : std::uint8_t {
  kNotDelivered = 0, kEnvelopeIgnored = 1, kInvalidMessage = 2, kStop = 3, kSchema = 4,
  kArm = 5, kClear = 6, kDisarm = 7, kDrive = 8,
};

constexpr std::size_t kFieldCount = static_cast<std::size_t>(CommandField::kCount);
constexpr std::size_t kMaxStoredCodePoints = 64;

struct JsonValue {
  ValueType type = ValueType::kAbsent;
  bool boolean = false;
  bool negative = false;      // kInt/kBigInt 부호. -0 정수는 0이므로 false.
  std::int64_t integer = 0;   // kInt만 유효. double을 거치지 않는다.
  double real = 0.0;          // kFloat만 유효. Python float()과 같은 IEEE binary64 값.
  std::uint16_t length = 0;   // kString의 Python len()(escape 해석·surrogate 결합 뒤 코드포인트 수).
};

// 64 코드포인트 이하일 때만 전체를 저장한다. 짝 없는 surrogate도 Python처럼 값으로 보존한다.
struct StoredString {
  bool is_string = false;
  bool complete = false;
  std::uint16_t length = 0;
  std::array<std::uint32_t, kMaxStoredCodePoints> code_points{};

  bool equals(const std::uint32_t* other, std::size_t size) const {
    if (!is_string || !complete || size != length) return false;
    for (std::size_t i = 0; i < size; ++i) if (code_points[i] != other[i]) return false;
    return true;
  }
};

// 결과는 payload를 가리키는 포인터를 갖지 않는다(프레임 콜백 이후에도 유효).
struct Command {
  ParseStatus status = ParseStatus::kInvalidSize;
  PayloadLayout layout = PayloadLayout::kBareCommand;
  bool envelope_type_command = false;   // envelope의 "type" == "command"
  bool envelope_command_object = false; // envelope의 "command"가 객체
  StoredString request;                 // envelope "request"(ack 회신용)
  std::uint16_t present = 0;            // 명령 객체에 있는 알려진 키 비트
  bool unknown_key = false;             // 명령 객체에 알려지지 않은 키 존재
  KindName kind = KindName::kNotString;
  std::array<JsonValue, kFieldCount> fields{};
  StoredString boot;
  StoredString client;

  const JsonValue& field(CommandField f) const { return fields[static_cast<std::size_t>(f)]; }
};

struct DriveLimits {
  double max_speed_mps;
  double max_steering_rad;
  std::int64_t lease_us;
};

namespace detail {

constexpr const char* kFieldNames[kFieldCount] = {
  "v", "kind", "boot", "generation", "lease", "client", "seq",
  "speed_mps", "steering_rad", "source_age_us", "ttl_us",
};

inline bool is_ws(std::uint8_t c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
inline bool is_digit(std::uint8_t c) { return c >= '0' && c <= '9'; }
inline int hex_value(std::uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Python bytes.decode('utf-8') strict와 같은 판정(overlong·surrogate·U+10FFFF 초과·잘림 거절).
inline bool valid_utf8(const std::uint8_t* s, std::size_t n) {
  std::size_t i = 0;
  while (i < n) {
    const std::uint8_t c = s[i];
    if (c < 0x80) { ++i; continue; }
    std::size_t extra = 0;
    std::uint8_t low = 0x80, high = 0xBF;
    if (c >= 0xC2 && c <= 0xDF) extra = 1;
    else if (c == 0xE0) { extra = 2; low = 0xA0; }
    else if ((c >= 0xE1 && c <= 0xEC) || c == 0xEE || c == 0xEF) extra = 2;
    else if (c == 0xED) { extra = 2; high = 0x9F; }
    else if (c == 0xF0) { extra = 3; low = 0x90; }
    else if (c >= 0xF1 && c <= 0xF3) extra = 3;
    else if (c == 0xF4) { extra = 3; high = 0x8F; }
    else return false;
    if (n - i <= extra) return false;
    if (s[i+1] < low || s[i+1] > high) return false;
    for (std::size_t k = 2; k <= extra; ++k) if (s[i+k] < 0x80 || s[i+k] > 0xBF) return false;
    i += extra + 1;
  }
  return true;
}

// 문법 검사가 끝난 문자열 본문(따옴표 제외)을 Python str 코드포인트로 읽는다.
class StringCursor {
 public:
  StringCursor(const std::uint8_t* s, std::size_t begin, std::size_t end) : s_(s), pos_(begin), end_(end) {}
  bool next(std::uint32_t& cp) {
    if (pos_ >= end_) return false;
    const std::uint8_t c = s_[pos_];
    if (c == '\\') {
      const std::uint8_t e = s_[pos_+1];
      pos_ += 2;
      switch (e) {
        case 'b': cp = 0x08; return true;
        case 'f': cp = 0x0C; return true;
        case 'n': cp = 0x0A; return true;
        case 'r': cp = 0x0D; return true;
        case 't': cp = 0x09; return true;
        case 'u': break;
        default: cp = e; return true;  // '"', '\\', '/'
      }
      cp = hex4(pos_);
      pos_ += 4;
      // Python: 높은 surrogate 바로 뒤의 \u 낮은 surrogate만 결합한다. 그 밖에는 단독 값으로 둔다.
      if (cp >= 0xD800 && cp <= 0xDBFF && pos_ + 6 <= end_ && s_[pos_] == '\\' && s_[pos_+1] == 'u') {
        const std::uint32_t low = hex4(pos_ + 2);
        if (low >= 0xDC00 && low <= 0xDFFF) {
          cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
          pos_ += 6;
        }
      }
      return true;
    }
    if (c < 0x80) { cp = c; ++pos_; return true; }
    std::size_t extra = c >= 0xF0 ? 3 : c >= 0xE0 ? 2 : 1;
    cp = c & (0x3F >> extra);
    for (std::size_t k = 1; k <= extra; ++k) cp = (cp << 6) | (s_[pos_+k] & 0x3F);
    pos_ += extra + 1;
    return true;
  }

 private:
  std::uint32_t hex4(std::size_t at) const {
    std::uint32_t v = 0;
    for (std::size_t k = 0; k < 4; ++k) v = (v << 4) | static_cast<std::uint32_t>(hex_value(s_[at+k]));
    return v;
  }
  const std::uint8_t* s_;
  std::size_t pos_, end_;
};

inline bool string_equals_ascii(const std::uint8_t* s, std::size_t begin, std::size_t end, const char* ascii) {
  StringCursor cursor(s, begin, end);
  std::uint32_t cp = 0;
  std::size_t i = 0;
  while (cursor.next(cp)) {
    if (ascii[i] == '\0' || cp != static_cast<std::uint8_t>(ascii[i])) return false;
    ++i;
  }
  return ascii[i] == '\0';
}

inline bool strings_equal(const std::uint8_t* s, std::size_t a0, std::size_t a1, std::size_t b0, std::size_t b1) {
  StringCursor a(s, a0, a1), b(s, b0, b1);
  std::uint32_t x = 0, y = 0;
  while (true) {
    const bool more_a = a.next(x), more_b = b.next(y);
    if (more_a != more_b) return false;
    if (!more_a) return true;
    if (x != y) return false;
  }
}

// 정수 i <= 유한 double d 의 정확한 비교(double 변환 반올림 없음).
inline bool int_le_double(std::int64_t i, double d) {
  if (d >= 9223372036854775808.0) return true;
  if (d < -9223372036854775808.0) return false;
  return i <= static_cast<std::int64_t>(std::floor(d));
}
inline bool uint_le_double(std::uint64_t u, double d) {
  if (d < 0) return false;
  if (d >= 18446744073709551616.0) return true;
  return u <= static_cast<std::uint64_t>(std::floor(d));
}

}  // namespace detail

class CommandParser {
 public:
  // payload는 이 호출 동안만 읽는다. out은 매 호출 처음에 초기화되므로 실패 결과에 이전 명령이 남지 않는다.
  ParseStatus parse(const std::uint8_t* data, std::size_t size, PayloadLayout layout, Command& out) {
    out = Command{};
    out.layout = layout;
    out.status = scan(data, size, layout);
    if (out.status == ParseStatus::kObject) extract(data, out);
    else { const auto status = out.status; out = Command{}; out.layout = layout; out.status = status; }
    return out.status;
  }

 private:
  static constexpr std::uint16_t kMarker = 0xFFFF;
  static constexpr std::size_t kNone = static_cast<std::size_t>(-1);
  // 객체 표지(최소 2 B)와 키(최소 4 B)의 합은 payload 1536 B 안에서 768을 넘지 않는다.
  static constexpr std::size_t kMaxKeyEntries = kMaxPayload / 2;
  struct KeyEntry { std::uint16_t begin; std::uint16_t end; };
  struct Span { std::size_t begin = 0, end = 0; ValueType type = ValueType::kAbsent; bool is_float = false;
                std::uint16_t length = 0; };
  enum class Expect : std::uint8_t { kValue, kValueOrClose, kKeyOrClose, kKey, kColon, kCommaOrClose, kEnd };
  enum Slot : std::size_t { kEnvelopeType = kFieldCount, kEnvelopeRequest, kEnvelopeCommand, kSlotCount };

  void set_bit(std::array<std::uint8_t, kMaxPayload/8>& bits, std::size_t i, bool v) {
    if (v) bits[i/8] = static_cast<std::uint8_t>(bits[i/8] | (1u << (i%8)));
    else bits[i/8] = static_cast<std::uint8_t>(bits[i/8] & ~(1u << (i%8)));
  }
  static bool get_bit(const std::array<std::uint8_t, kMaxPayload/8>& bits, std::size_t i) {
    return (bits[i/8] >> (i%8)) & 1u;
  }

  void reset_scratch() {
    depth_ = 0; key_count_ = 0; key_base_ = 0; command_depth_ = 0; slot_ = kNone; top_type_ = ValueType::kAbsent;
    for (auto& span : spans_) span = Span{};
    present_ = 0; unknown_ = false;
  }

  // 1단계: Python json.loads와 같은 순서로 문법·상수·중복 키 오류를 판정하고 관심 값의 위치만 기록한다.
  ParseStatus scan(const std::uint8_t* s, std::size_t n, PayloadLayout layout) {
    reset_scratch();
    if (s == nullptr || n == 0 || n > kMaxPayload) return ParseStatus::kInvalidSize;
    if (!detail::valid_utf8(s, n)) return ParseStatus::kInvalidUtf8;
    Expect expect = Expect::kValue;
    std::size_t pos = 0;
    std::size_t key_begin = 0, key_end = 0;
    while (true) {
      while (pos < n && detail::is_ws(s[pos])) ++pos;
      if (pos >= n) {
        if (expect != Expect::kEnd) return ParseStatus::kSyntax;
        return top_type_ == ValueType::kObject ? ParseStatus::kObject : ParseStatus::kNotObject;
      }
      const std::uint8_t c = s[pos];
      switch (expect) {
        case Expect::kEnd:
          return ParseStatus::kSyntax;  // 최상위 값 뒤의 잔여 데이터
        case Expect::kColon:
          if (c != ':') return ParseStatus::kSyntax;
          ++pos;
          expect = Expect::kValue;
          continue;
        case Expect::kCommaOrClose: {
          const bool object = get_bit(container_, depth_ - 1);
          if (c == ',') { ++pos; expect = object ? Expect::kKey : Expect::kValue; continue; }
          if (c == (object ? '}' : ']')) {
            ++pos;
            const auto status = close(object);
            if (status != ParseStatus::kObject) return status;
            expect = depth_ == 0 ? Expect::kEnd : Expect::kCommaOrClose;
            continue;
          }
          return ParseStatus::kSyntax;
        }
        case Expect::kKeyOrClose:
        case Expect::kKey:
          if (c == '}' && expect == Expect::kKeyOrClose) {
            ++pos;
            const auto status = close(true);
            if (status != ParseStatus::kObject) return status;
            expect = depth_ == 0 ? Expect::kEnd : Expect::kCommaOrClose;
            continue;
          }
          if (c != '"') return ParseStatus::kSyntax;
          {
            std::uint16_t ignored = 0;
            if (!scan_string(s, n, pos, key_begin, key_end, ignored)) return ParseStatus::kSyntax;
          }
          if (!on_key(s, key_begin, key_end, layout)) return ParseStatus::kSyntax;
          expect = Expect::kColon;
          continue;
        case Expect::kValueOrClose:
          if (c == ']') {
            ++pos;
            const auto status = close(false);
            if (status != ParseStatus::kObject) return status;
            expect = depth_ == 0 ? Expect::kEnd : Expect::kCommaOrClose;
            continue;
          }
          [[fallthrough]];
        case Expect::kValue: {
          const std::size_t slot = slot_;
          slot_ = kNone;
          if (c == '{' || c == '[') {
            const bool object = c == '{';
            if (depth_ == 0) top_type_ = object ? ValueType::kObject : ValueType::kArray;
            if (slot != kNone) spans_[slot].type = object ? ValueType::kObject : ValueType::kArray;
            set_bit(container_, depth_, object);
            set_bit(duplicate_, depth_, false);
            ++depth_;
            ++pos;
            if (object) {
              // 객체별 키 목록의 시작 표지. begin에는 바깥 객체의 시작 위치를 보관한다.
              if (key_count_ >= keys_.size()) return ParseStatus::kSyntax;  // 분석상 도달 불가(768 이하)
              keys_[key_count_++] = KeyEntry{static_cast<std::uint16_t>(key_base_), kMarker};
              key_base_ = key_count_;
              if ((layout == PayloadLayout::kBareCommand && depth_ == 1) ||
                  (layout == PayloadLayout::kEnvelope && slot == kEnvelopeCommand && depth_ == 2))
                command_depth_ = depth_;
            }
            expect = object ? Expect::kKeyOrClose : Expect::kValueOrClose;
            continue;
          }
          Span value;
          value.begin = pos;
          if (c == '"') {
            std::size_t begin = 0, end = 0;
            if (!scan_string(s, n, pos, begin, end, value.length)) return ParseStatus::kSyntax;
            value.type = ValueType::kString;
            value.begin = begin;
            value.end = end;
          } else if (c == 'N' || c == 'I' || (c == '-' && pos + 1 < n && s[pos+1] == 'I')) {
            // Python은 NaN/Infinity/-Infinity 철자가 맞으면 즉시 parse_constant를 호출해 거절한다.
            const char* word = c == 'N' ? "NaN" : c == 'I' ? "Infinity" : "-Infinity";
            std::size_t k = 0;
            while (word[k] != '\0' && pos + k < n && s[pos+k] == static_cast<std::uint8_t>(word[k])) ++k;
            return word[k] == '\0' ? ParseStatus::kNonFiniteConstant : ParseStatus::kSyntax;
          } else if (c == '-' || detail::is_digit(c)) {
            if (!scan_number(s, n, pos, value.is_float)) return ParseStatus::kSyntax;
            value.type = value.is_float ? ValueType::kFloat : ValueType::kInt;
            value.end = pos;
          } else if (c == 't' || c == 'f' || c == 'n') {
            const char* word = c == 't' ? "true" : c == 'f' ? "false" : "null";
            std::size_t k = 0;
            while (word[k] != '\0') {
              if (pos + k >= n || s[pos+k] != static_cast<std::uint8_t>(word[k])) return ParseStatus::kSyntax;
              ++k;
            }
            pos += k;
            value.type = c == 'n' ? ValueType::kNull : ValueType::kBool;
            value.end = pos;
          } else {
            return ParseStatus::kSyntax;
          }
          if (depth_ == 0) top_type_ = value.type;
          if (slot != kNone) spans_[slot] = value;
          expect = depth_ == 0 ? Expect::kEnd : Expect::kCommaOrClose;
          continue;
        }
      }
    }
  }

  // 객체가 닫힐 때 Python object_pairs_hook이 중복 키를 거절하므로 그 시점에 보고한다.
  ParseStatus close(bool object) {
    --depth_;
    if (object) {
      if (get_bit(duplicate_, depth_)) return ParseStatus::kDuplicateKey;
      key_count_ = key_base_ - 1;
      key_base_ = keys_[key_count_].begin;
      if (command_depth_ == depth_ + 1) command_depth_ = 0;
    }
    return ParseStatus::kObject;
  }

  bool on_key(const std::uint8_t* s, std::size_t begin, std::size_t end, PayloadLayout layout) {
    if (key_count_ >= keys_.size()) return false;  // 분석상 도달 불가(768 이하)
    for (std::size_t i = key_base_; i < key_count_; ++i) {
      if (detail::strings_equal(s, keys_[i].begin, keys_[i].end, begin, end)) {
        set_bit(duplicate_, depth_ - 1, true);
        break;
      }
    }
    keys_[key_count_++] = KeyEntry{static_cast<std::uint16_t>(begin), static_cast<std::uint16_t>(end)};
    slot_ = kNone;
    if (command_depth_ != 0 && depth_ == command_depth_) {
      for (std::size_t f = 0; f < kFieldCount; ++f) {
        if (detail::string_equals_ascii(s, begin, end, detail::kFieldNames[f])) {
          slot_ = f;
          present_ = static_cast<std::uint16_t>(present_ | (1u << f));
          return true;
        }
      }
      unknown_ = true;
    } else if (layout == PayloadLayout::kEnvelope && depth_ == 1) {
      if (detail::string_equals_ascii(s, begin, end, "type")) slot_ = kEnvelopeType;
      else if (detail::string_equals_ascii(s, begin, end, "request")) slot_ = kEnvelopeRequest;
      else if (detail::string_equals_ascii(s, begin, end, "command")) slot_ = kEnvelopeCommand;
    }
    return true;
  }

  // Python scanstring(strict=True): 제어문자 거절, 허용 escape만 인정, \u는 정확히 4 hex.
  static bool scan_string(const std::uint8_t* s, std::size_t n, std::size_t& pos,
                          std::size_t& begin, std::size_t& end, std::uint16_t& length) {
    ++pos;
    begin = pos;
    std::size_t count = 0;
    while (true) {
      if (pos >= n) return false;
      const std::uint8_t c = s[pos];
      if (c == '"') { end = pos; ++pos; length = static_cast<std::uint16_t>(count); return true; }
      if (c < 0x20) return false;
      if (c == '\\') {
        if (pos + 1 >= n) return false;
        const std::uint8_t e = s[pos+1];
        if (e == '"' || e == '\\' || e == '/' || e == 'b' || e == 'f' || e == 'n' || e == 'r' || e == 't') {
          pos += 2; ++count; continue;
        }
        if (e != 'u') return false;
        if (!hex4(s, n, pos + 2)) return false;
        const std::uint32_t cp = value4(s, pos + 2);
        pos += 6;
        if (cp >= 0xD800 && cp <= 0xDBFF && pos + 1 < n && s[pos] == '\\' && s[pos+1] == 'u') {
          if (!hex4(s, n, pos + 2)) return false;
          const std::uint32_t low = value4(s, pos + 2);
          if (low >= 0xDC00 && low <= 0xDFFF) pos += 6;
        }
        ++count;
        continue;
      }
      if (c >= 0x80) {  // UTF-8은 이미 검증했다. 연속 바이트는 따옴표/역슬래시가 아니다.
        pos += c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : 2;
      } else {
        ++pos;
      }
      ++count;
    }
  }
  static bool hex4(const std::uint8_t* s, std::size_t n, std::size_t at) {
    if (at + 4 > n) return false;
    for (std::size_t k = 0; k < 4; ++k) if (detail::hex_value(s[at+k]) < 0) return false;
    return true;
  }
  static std::uint32_t value4(const std::uint8_t* s, std::size_t at) {
    std::uint32_t v = 0;
    for (std::size_t k = 0; k < 4; ++k) v = (v << 4) | static_cast<std::uint32_t>(detail::hex_value(s[at+k]));
    return v;
  }

  // Python _match_number: -?(0|[1-9][0-9]*)(.[0-9]+)?([eE][-+]?[0-9]+)? , 지수 숫자가 없으면 지수 앞에서 끝난다.
  static bool scan_number(const std::uint8_t* s, std::size_t n, std::size_t& pos, bool& is_float) {
    is_float = false;
    if (s[pos] == '-') { ++pos; if (pos >= n) return false; }
    if (s[pos] >= '1' && s[pos] <= '9') { ++pos; while (pos < n && detail::is_digit(s[pos])) ++pos; }
    else if (s[pos] == '0') ++pos;
    else return false;
    if (pos + 1 < n && s[pos] == '.' && detail::is_digit(s[pos+1])) {
      is_float = true;
      pos += 2;
      while (pos < n && detail::is_digit(s[pos])) ++pos;
    }
    if (pos < n && (s[pos] == 'e' || s[pos] == 'E')) {
      const std::size_t mark = pos;
      ++pos;
      if (pos < n && (s[pos] == '+' || s[pos] == '-')) ++pos;
      const std::size_t digits = pos;
      while (pos < n && detail::is_digit(s[pos])) ++pos;
      if (pos > digits) is_float = true; else pos = mark;
    }
    return true;
  }

  // 2단계: 문법이 확정된 뒤 관심 값만 변환한다.
  void extract(const std::uint8_t* s, Command& out) const {
    if (out.layout == PayloadLayout::kEnvelope) {
      const Span& type = spans_[kEnvelopeType];
      out.envelope_type_command = type.type == ValueType::kString &&
                                  detail::string_equals_ascii(s, type.begin, type.end, "command");
      out.envelope_command_object = spans_[kEnvelopeCommand].type == ValueType::kObject;
      store(s, spans_[kEnvelopeRequest], out.request);
    }
    out.present = present_;
    out.unknown_key = unknown_;
    for (std::size_t f = 0; f < kFieldCount; ++f) convert(s, spans_[f], out.fields[f]);
    const Span& kind = spans_[static_cast<std::size_t>(CommandField::kKind)];
    if (kind.type == ValueType::kString) {
      out.kind = KindName::kOther;
      const char* names[] = {"STOP", "ARM", "CLEAR", "DISARM", "DRIVE"};
      const KindName kinds[] = {KindName::kStop, KindName::kArm, KindName::kClear, KindName::kDisarm, KindName::kDrive};
      for (std::size_t i = 0; i < 5; ++i)
        if (detail::string_equals_ascii(s, kind.begin, kind.end, names[i])) out.kind = kinds[i];
    }
    store(s, spans_[static_cast<std::size_t>(CommandField::kBoot)], out.boot);
    store(s, spans_[static_cast<std::size_t>(CommandField::kClient)], out.client);
  }

  static void store(const std::uint8_t* s, const Span& span, StoredString& out) {
    if (span.type != ValueType::kString) return;
    out.is_string = true;
    out.length = span.length;
    out.complete = span.length <= kMaxStoredCodePoints;
    if (!out.complete) return;
    detail::StringCursor cursor(s, span.begin, span.end);
    std::uint32_t cp = 0;
    std::size_t i = 0;
    // 1단계 길이와 같은 규칙이므로 넘치지 않지만, 어긋나도 배열 밖에 쓰지 않는다.
    while (i < kMaxStoredCodePoints && cursor.next(cp)) out.code_points[i++] = cp;
  }

  static void convert(const std::uint8_t* s, const Span& span, JsonValue& out) {
    out = JsonValue{};
    out.type = span.type;
    if (span.type == ValueType::kString) { out.length = span.length; return; }
    if (span.type == ValueType::kBool) { out.boolean = s[span.begin] == 't'; return; }
    if (span.type == ValueType::kInt) {
      // 10진 정수를 정확히 읽는다. int64를 넘으면 부호만 남기는 kBigInt.
      std::size_t i = span.begin;
      const bool negative = s[i] == '-';
      if (negative) ++i;
      std::uint64_t magnitude = 0;
      bool overflow = false;
      for (; i < span.end; ++i) {
        const std::uint64_t digit = static_cast<std::uint64_t>(s[i] - '0');
        if (magnitude > (UINT64_MAX - digit) / 10) { overflow = true; break; }
        magnitude = magnitude * 10 + digit;
      }
      const std::uint64_t limit = negative ? (static_cast<std::uint64_t>(INT64_MAX) + 1) : static_cast<std::uint64_t>(INT64_MAX);
      if (overflow || magnitude > limit) { out.type = ValueType::kBigInt; out.negative = negative; return; }
      out.negative = negative && magnitude != 0;
      out.integer = negative ? (magnitude == limit ? INT64_MIN : -static_cast<std::int64_t>(magnitude))
                             : static_cast<std::int64_t>(magnitude);
      return;
    }
    if (span.type == ValueType::kFloat) {
      const char* first = reinterpret_cast<const char*>(s + span.begin);
      const char* last = reinterpret_cast<const char*>(s + span.end);
      double value = 0.0;
      const auto result = std::from_chars(first, last, value);
      const bool negative = *first == '-';
      if (result.ec == std::errc::result_out_of_range) {
        // Python float()은 범위 밖을 ±inf, 0이 아닌 최소값 미만을 ±0.0으로 반올림한다.
        value = overflows(first, last) ? HUGE_VAL : 0.0;
        if (negative) value = -value;
      }
      out.real = value;
      out.negative = std::signbit(value);
    }
  }

  // 유효 숫자의 10진 자리(첫 0 아닌 숫자 위치 + 지수)로 범위 밖 방향을 정한다.
  static bool overflows(const char* first, const char* last) {
    const char* p = first;
    if (*p == '-') ++p;
    long long position = 0;  // 첫 0 아닌 숫자의 10진 자리(1 = 일의 자리)
    bool found = false, fraction = false;
    long long integer_digits = 0, leading_fraction_zeros = 0;
    for (; p < last && *p != 'e' && *p != 'E'; ++p) {
      if (*p == '.') { fraction = true; continue; }
      if (!fraction) { if (found || *p != '0') { found = true; ++integer_digits; } }
      else if (!found) { if (*p == '0') ++leading_fraction_zeros; else found = true; }
    }
    if (!found) return false;
    position = integer_digits > 0 ? integer_digits : -leading_fraction_zeros;
    long long exponent = 0;
    bool exponent_negative = false;
    if (p < last) {
      ++p;
      if (*p == '+' || *p == '-') { exponent_negative = *p == '-'; ++p; }
      for (; p < last; ++p) if (exponent < 1000000) exponent = exponent * 10 + (*p - '0');
    }
    return position + (exponent_negative ? -exponent : exponent) > 0;
  }

  std::array<KeyEntry, kMaxKeyEntries + 1> keys_{};
  std::array<std::uint8_t, kMaxPayload/8> container_{};  // 1 = 객체, 0 = 배열
  std::array<std::uint8_t, kMaxPayload/8> duplicate_{};  // 중복 키가 이미 나온 열린 객체
  std::array<Span, kSlotCount> spans_{};
  std::size_t depth_ = 0;
  std::size_t key_count_ = 0;
  std::size_t key_base_ = 0;
  std::size_t command_depth_ = 0;
  std::size_t slot_ = kNone;
  ValueType top_type_ = ValueType::kAbsent;
  std::uint16_t present_ = 0;
  bool unknown_ = false;
};

// ---- receive의 내용 분기(상태 비의존 부분). 상태기계 이관이 아니다. ----

inline std::uint16_t field_bit(CommandField f) { return static_cast<std::uint16_t>(1u << static_cast<unsigned>(f)); }

inline CommandClass classify(const Command& c) {
  if (c.status != ParseStatus::kObject) return CommandClass::kNotDelivered;
  if (c.layout == PayloadLayout::kEnvelope && (!c.envelope_type_command || !c.envelope_command_object))
    return CommandClass::kEnvelopeIgnored;
  const JsonValue& v = c.field(CommandField::kV);
  if (v.type != ValueType::kInt || v.integer != 1) return CommandClass::kInvalidMessage;
  const std::uint16_t base = field_bit(CommandField::kV) | field_bit(CommandField::kKind);
  if (c.kind == KindName::kStop && c.present == base && !c.unknown_key) return CommandClass::kStop;
  const std::uint16_t common = base | field_bit(CommandField::kBoot) | field_bit(CommandField::kGeneration) |
      field_bit(CommandField::kLease) | field_bit(CommandField::kClient) | field_bit(CommandField::kSeq);
  const std::uint16_t drive = common | field_bit(CommandField::kSpeed) | field_bit(CommandField::kSteering) |
      field_bit(CommandField::kSourceAge) | field_bit(CommandField::kTtl);
  CommandClass result;
  switch (c.kind) {
    case KindName::kArm: result = CommandClass::kArm; break;
    case KindName::kClear: result = CommandClass::kClear; break;
    case KindName::kDisarm: result = CommandClass::kDisarm; break;
    case KindName::kDrive: result = CommandClass::kDrive; break;
    default: return CommandClass::kSchema;
  }
  if (c.unknown_key || c.present != (result == CommandClass::kDrive ? drive : common)) return CommandClass::kSchema;
  return result;
}

// receive의 "session" 분기 중 값 형태만 본다. boot·generation 일치와 임대 조회는 상태기계 몫이다.
inline bool session_shape_ok(const Command& c) {
  const JsonValue& generation = c.field(CommandField::kGeneration);
  const JsonValue& lease = c.field(CommandField::kLease);
  const JsonValue& seq = c.field(CommandField::kSeq);
  return generation.type == ValueType::kInt && generation.integer >= 0 &&
         lease.type == ValueType::kInt && lease.integer >= 1 &&
         c.client.is_string && c.client.length >= 1 && c.client.length <= kMaxStoredCodePoints &&
         seq.type == ValueType::kInt && seq.integer >= 0 && seq.integer <= 0xFFFFFFFFLL;
}

// 한도는 유한하고 2^63 미만이어야 한다(그 밖에는 kBigInt 비교가 정확하지 않으므로 거절).
inline bool limits_supported(const DriveLimits& limits) {
  return std::isfinite(limits.max_speed_mps) && std::isfinite(limits.max_steering_rad) &&
         std::fabs(limits.max_speed_mps) < 9223372036854775808.0 &&
         std::fabs(limits.max_steering_rad) < 9223372036854775808.0;
}

// receive의 invalid_drive 조건 전체(어느 조건이든 같은 고장). false면 상태기계가 순서상 그 지점에서 고장을 고정해야 한다.
inline bool drive_values_ok(const Command& c, const DriveLimits& limits) {
  if (!limits_supported(limits)) return false;
  const JsonValue& speed = c.field(CommandField::kSpeed);
  const JsonValue& angle = c.field(CommandField::kSteering);
  const JsonValue& age = c.field(CommandField::kSourceAge);
  const JsonValue& ttl = c.field(CommandField::kTtl);
  bool speed_ok = false;
  if (speed.type == ValueType::kInt) speed_ok = speed.integer >= 0 && detail::int_le_double(speed.integer, limits.max_speed_mps);
  else if (speed.type == ValueType::kFloat) speed_ok = std::isfinite(speed.real) && 0.0 <= speed.real && speed.real <= limits.max_speed_mps;
  // kBigInt의 크기는 2^63 이상이므로 지원 한도 안에서는 항상 범위 밖이다.
  bool angle_ok = false;
  if (angle.type == ValueType::kInt) {
    const std::uint64_t magnitude = angle.integer < 0 ? static_cast<std::uint64_t>(0) - static_cast<std::uint64_t>(angle.integer)
                                                      : static_cast<std::uint64_t>(angle.integer);
    angle_ok = detail::uint_le_double(magnitude, limits.max_steering_rad);
  } else if (angle.type == ValueType::kFloat) {
    angle_ok = std::isfinite(angle.real) && !(std::fabs(angle.real) > limits.max_steering_rad);
  }
  const bool age_ok = age.type == ValueType::kInt && age.integer >= 0;
  const bool ttl_ok = ttl.type == ValueType::kInt && ttl.integer >= 1 && ttl.integer <= limits.lease_us;
  return speed_ok && angle_ok && age_ok && ttl_ok;
}

// drive_values_ok일 때 receive가 저장할 float(speed)/float(angle) 값.
inline double numeric_value(const JsonValue& v) {
  return v.type == ValueType::kInt ? static_cast<double>(v.integer) : v.real;
}

}  // namespace arena_actuation
