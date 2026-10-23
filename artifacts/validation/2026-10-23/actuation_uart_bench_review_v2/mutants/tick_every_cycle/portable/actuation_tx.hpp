#pragma once
// 구동 출력 없는 MCU -> 호스트 송신 정책 후보(PC 검증용). UART 드라이버·출력 허가·제어 task가 아니다.
// 단일 소유자 계약: receiver와 같은 작업(또는 같은 잠금)에서만 호출한다. ISR/통신 callback에서 직접 호출하지 않는다.
// 역할 분리:
//  - offer: 호스트 명령 생성에 필요한 필드만 담은 주기 임대. 임대는 프레임을 만드는(materialize) 순간에만 발행한다.
//  - ack: 요청 ID가 있는 명령의 판정 결과. 유한 큐, STOP 결과 우선, 넘치면 명시 계수 후 폐기한다.
//  - state: 세대/고정 상태 변화 알림. 최신 상태 하나로 병합(latest-wins)하며 병합 횟수를 싣는다.
//  - diag: 상세 진단. 저주기·최저 우선·최신 하나로 병합. 제어 판단에 쓰지 않는다.
// 이미 materialize한 프레임은 끝까지 그대로 보낸다(중간 교체 없음). 과거 임대/시각을 새 상태로 재포장하지 않는다.
// 이 객체는 명령을 만들거나 임대/명령 수명을 늘리지 않으며 CLEAR/ARM을 대신하지 않는다.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "actuation_ingress.hpp"
#include "actuation_receiver.hpp"
#include "actuation_wire.hpp"

namespace arena_actuation {

enum class TxKind : std::uint8_t { kNone = 0, kAck = 1, kState = 2, kOffer = 3, kDiagnostic = 4 };
enum class TxFault : std::uint8_t { kNone = 0, kInvalidConfig = 1, kInvalidClock = 2, kSinkContract = 3 };

constexpr std::size_t kTxAckCapacity = 8;          // 호스트 ActuationHost의 미결 요청 상한 8과 같은 크기
constexpr std::size_t kTxMaxFramesPerPump = 2;     // pump 호출당 materialize 상한(호출당 작업량 제한)
constexpr std::size_t kTxMaxTextChars = 32;        // state/reason/mode 문자열(코드 상수)의 상한
constexpr std::size_t kTxKinds = 5;

struct TxConfig {
  std::int64_t offer_period_us = 0;    // 주기 offer 간격. 운영값을 정하지 않으므로 호출자가 명시한다
  std::int64_t diag_period_us = 0;     // 0이면 상세 진단을 보내지 않는다
  std::uint8_t max_consecutive_events = 0;  // ack/state 연속 상한. offer 대기 시 그 뒤 offer를 먼저 보낸다
  // 실제 설정한 선로의 바이트당 시간(8N1이면 10/baud 초). 0이면 모름: 진단을 주기만으로 보낸다.
  // 0이 아니면 진단 프레임은 다음 offer 기한 전에 직렬화가 끝날 때만 시작한다(긴 진단이 offer를 가리지 않게).
  std::int64_t line_ns_per_byte = 0;

  bool valid() const {
    const auto in = [](std::int64_t v) { return v >= 1 && v <= kMaxConfigUs; };
    return in(offer_period_us) && (diag_period_us == 0 || in(diag_period_us)) &&
           max_consecutive_events >= 1 && max_consecutive_events <= kTxAckCapacity &&
           line_ns_per_byte >= 0 && line_ns_per_byte <= 1'000'000;
  }
};

struct TxFrameRecord {
  TxKind kind = TxKind::kNone;
  std::uint64_t frame_id = 0;
  std::size_t size = 0;            // CRC/헤더 포함 wire 크기
  std::int64_t trigger_us = 0;     // 이 프레임이 필요해진 시각(ack 수신·상태 변화·offer/diag 기한)
  std::int64_t materialize_us = 0; // 내용을 고정한 시각. offer 임대 발행 시각과 같다
  std::int64_t generation = 0;
  std::int64_t lease = 0;          // offer만
  bool urgent = false;             // 세대 변화로 앞당긴 offer
  bool stop_ack = false;
};

struct TxPumpResult {
  std::size_t bytes_written = 0;
  std::size_t materialized = 0;
  std::array<TxFrameRecord, kTxMaxFramesPerPump> frames{};
  std::size_t completed = 0;       // 이번 호출에서 sink가 마지막 바이트까지 받은 프레임 수
  bool backpressure = false;       // sink가 남은 바이트를 다 받지 못했다
};

struct TxStats {
  std::array<std::uint64_t, kTxKinds> frames{};      // materialize 수
  std::array<std::uint64_t, kTxKinds> completed{};   // sink 수락 완료 수
  std::array<std::uint64_t, kTxKinds> bytes{};
  std::array<std::int64_t, kTxKinds> max_dwell_us{};     // trigger -> materialize
  std::array<std::int64_t, kTxKinds> max_inflight_us{};  // materialize -> sink 수락 완료
  std::uint64_t ack_enqueued = 0, ack_dropped_overflow = 0, ack_evicted_for_stop = 0, ack_purged_session = 0;
  std::uint64_t ack_queue_max = 0;
  std::uint64_t state_changes = 0, state_merged = 0;
  std::uint64_t offers_urgent = 0, offer_unavailable = 0;
  std::uint64_t diag_skipped_periods = 0, diag_deferred_for_offer = 0;
  std::uint64_t aborted_frames = 0, aborted_bytes = 0;
  std::uint64_t encode_failures = 0, sink_zero_writes = 0, sink_partial_writes = 0;
  std::uint64_t starvation_guard_used = 0;
};

namespace tx_detail {

inline const char* state_name(ReceiverState s) {
  constexpr const char* names[] = {"DISARMED", "ARMED", "ACTIVE", "ESTOP_LATCHED", "FAULT_LATCHED"};
  return names[static_cast<std::size_t>(s)];
}
inline bool armed_state(ReceiverState s) { return s == ReceiverState::kArmed || s == ReceiverState::kActive; }
inline bool same_text(const char* a, const char* b) { return a == b || (a && b && std::strcmp(a, b) == 0); }

// Python json.dumps(..., ensure_ascii=True, separators=(",", ":"))와 같은 바이트를 쓴다.
class JsonWriter {
 public:
  JsonWriter(std::uint8_t* out, std::size_t capacity) : out_(out), capacity_(capacity) {}
  void lit(const char* s) { while (*s) put(static_cast<std::uint8_t>(*s++)); }
  void boolean(bool v) { lit(v ? "true" : "false"); }
  void null() { lit("null"); }
  void i64(std::int64_t v) {
    char digits[20];
    std::size_t n = 0;
    // INT64_MIN도 overflow 없이 크기를 uint64로 만든다.
    std::uint64_t m = v < 0 ? ~static_cast<std::uint64_t>(v) + 1u : static_cast<std::uint64_t>(v);
    do { digits[n++] = static_cast<char>('0' + m % 10u); m /= 10u; } while (m != 0);
    if (v < 0) put('-');
    while (n != 0) put(static_cast<std::uint8_t>(digits[--n]));
  }
  void u64_saturated(std::uint64_t v) { i64(v > static_cast<std::uint64_t>(INT64_MAX) ? INT64_MAX : static_cast<std::int64_t>(v)); }
  void opt_i64(bool has, std::int64_t v) { if (has) i64(v); else null(); }
  void code_points(const std::uint32_t* cp, std::size_t n) {
    put('"');
    for (std::size_t i = 0; i < n; ++i) escaped(cp[i]);
    put('"');
  }
  void stored(const StoredString& s) {
    if (!s.is_string || !s.complete) { ok_ = false; return; }  // 길이만 남은 문자열은 원문을 복원할 수 없다
    code_points(s.code_points.data(), s.length);
  }
  void opt_stored(const StoredString* s) { if (s != nullptr) stored(*s); else null(); }
  // 코드 상수(state/reason/mode) 전용: escape가 필요 없는 인쇄 가능 ASCII 1~32자만 허용한다.
  void text(const char* s) {
    put('"');
    std::size_t n = 0;
    for (; s != nullptr && *s; ++s, ++n) {
      const auto c = static_cast<std::uint8_t>(*s);
      if (c < 0x20 || c > 0x7e || c == '"' || c == '\\') ok_ = false;
      put(c);
    }
    if (s == nullptr || n == 0 || n > kTxMaxTextChars) ok_ = false;
    put('"');
  }
  bool ok() const { return ok_; }
  std::size_t size() const { return used_; }

 private:
  void put(std::uint8_t c) {
    if (used_ >= capacity_) { ok_ = false; return; }
    out_[used_++] = c;
  }
  void hex4(std::uint32_t v) {
    constexpr char digits[] = "0123456789abcdef";
    put('\\'); put('u');
    for (int shift = 12; shift >= 0; shift -= 4) put(static_cast<std::uint8_t>(digits[(v >> shift) & 0xFu]));
  }
  void escaped(std::uint32_t c) {
    if (c == '"' || c == '\\') { put('\\'); put(static_cast<std::uint8_t>(c)); return; }
    if (c == '\n') { put('\\'); put('n'); return; }
    if (c == '\r') { put('\\'); put('r'); return; }
    if (c == '\t') { put('\\'); put('t'); return; }
    if (c == '\b') { put('\\'); put('b'); return; }
    if (c == '\f') { put('\\'); put('f'); return; }
    if (c >= 0x20 && c <= 0x7e) { put(static_cast<std::uint8_t>(c)); return; }
    if (c > 0x10FFFF) { ok_ = false; return; }
    if (c < 0x10000) { hex4(c); return; }  // 짝 없는 surrogate도 Python처럼 그대로 \uXXXX로 쓴다
    const std::uint32_t v = c - 0x10000;
    hex4(0xD800u | (v >> 10));
    hex4(0xDC00u | (v & 0x3FFu));
  }

  std::uint8_t* out_;
  std::size_t capacity_;
  std::size_t used_ = 0;
  bool ok_ = true;
};

// 같은 인코더 코드를 따라가며 필드 종류별 최악 바이트를 더한다. 값은 읽지 않는다.
class WorstWriter {
 public:
  void lit(const char* s) { size_ += std::strlen(s); }
  void boolean(bool) { size_ += 5; }
  void null() { size_ += 4; }
  void i64(std::int64_t) { size_ += 20; }
  void u64_saturated(std::uint64_t) { size_ += 19; }
  void opt_i64(bool, std::int64_t) { size_ += 20; }
  void code_points(const std::uint32_t*, std::size_t) { size_ += 2 + kMaxStoredCodePoints; }  // boot: 인쇄 가능 ASCII 검사 후에만 쓴다
  void stored(const StoredString&) { size_ += 2 + 12 * kMaxStoredCodePoints; }  // astral = surrogate 쌍 12 B
  void opt_stored(const StoredString*) { size_ += 2 + 12 * kMaxStoredCodePoints; }
  void text(const char*) { size_ += 2 + kTxMaxTextChars; }
  bool ok() const { return true; }
  std::size_t size() const { return size_; }
 private:
  std::size_t size_ = 0;
};

// receiver.boot()가 JSON escape 없이 그대로 쓸 수 있는 ASCII인지. offer 최악 크기를 payload 상한 안에 둔다.
inline bool printable_boot(const StoredString& boot) {
  if (!boot.is_string || !boot.complete || boot.length == 0) return false;
  for (std::size_t i = 0; i < boot.length; ++i) {
    const auto c = boot.code_points[i];
    if (c < 0x20 || c > 0x7e || c == '"' || c == '\\') return false;
  }
  return true;
}

// double -> 정수 micro 단위. 범위 밖이면 null(진단 전용, 판정에 쓰지 않음).
inline bool micro_units(double v, std::int64_t& out) {
  if (!std::isfinite(v) || std::fabs(v) > 9.0e12) return false;
  out = static_cast<std::int64_t>(std::llround(v * 1e6));
  return true;
}

}  // namespace tx_detail

class ActuationTxScheduler {
 public:
  // receiver는 이 객체보다 오래 살아야 한다. 구성/boot 형식이 잘못되면 영구 고장으로 두고 아무것도 보내지 않는다.
  // offer 임대 발행 때문에 receiver를 변경하므로 비상수 참조를 받는다(같은 소유자 안에서만 호출).
  ActuationTxScheduler(const TxConfig& config, ActuationReceiverCore& receiver)
      : config_(config), receiver_(receiver) {
    if (!config_.valid() || !tx_detail::printable_boot(receiver_.boot())) fault_ = TxFault::kInvalidConfig;
    seen_generation_ = receiver_.generation();
    seen_state_ = receiver_.state();
    seen_reason_ = receiver_.reason();
  }
  ActuationTxScheduler(const ActuationTxScheduler&) = delete;
  ActuationTxScheduler& operator=(const ActuationTxScheduler&) = delete;

  // 출력 단계(독립 감시)가 알려 주는 준비 신호를 offer에 그대로 싣는다. 이 객체가 출력을 허가하지 않는다.
  void set_output_ready(bool ready) { output_ready_ = ready; }
  // 링크가 끊긴 동안에는 materialize/쓰기를 하지 않는다(UART처럼 연결 개념이 없으면 항상 true).
  void set_link_up(bool up) { link_up_ = up; }

  // ingress observer에서 받은 판정. 요청 ID가 있는 수신 명령만 ack 대상이다(빈 request의 DRIVE는 ack 없음).
  void on_delivery(const IngressDelivery& delivery, const Command& command, std::int64_t now) {
    if (!time_ok(now)) return;
    const StoredString& request = command.request;
    if ((!delivery.receiver_called && !delivery.output_gate_rejected) ||
        !request.is_string || !request.complete || request.length == 0) return;
    const bool stop = delivery.classification == CommandClass::kStop;
    if (ack_count_ == kTxAckCapacity) {
      std::size_t victim = kTxAckCapacity;
      if (stop) {
        for (std::size_t i = 0; i < ack_count_; ++i) if (!acks_[i].stop) { victim = i; break; }
      }
      if (victim == kTxAckCapacity) { ++stats_.ack_dropped_overflow; return; }  // 호스트는 timeout으로 미확인 처리
      erase_ack(victim);
      ++stats_.ack_evicted_for_stop;
    }
    acks_[ack_count_++] = AckEntry{request, delivery.result, stop, now};
    ++stats_.ack_enqueued;
    if (ack_count_ > stats_.ack_queue_max) stats_.ack_queue_max = ack_count_;
    observe(now);
  }

  // 링크 세션 경계. 이미 보내던 프레임 꼬리는 새 세션에 이어 붙이지 않고 버린다. 이전 세션의 ack도 버린다.
  // 수신 쪽(ingress)이 넣은 로컬 STOP은 다음 observe에서 state/offer로 알린다. CLEAR/ARM을 만들지 않는다.
  void on_session_break(std::int64_t now) {
    if (!time_ok(now)) return;
    if (frame_size_ != 0) {
      ++stats_.aborted_frames;
      stats_.aborted_bytes += frame_size_ - frame_offset_;
      frame_size_ = frame_offset_ = 0;
    }
    stats_.ack_purged_session += ack_count_;
    ack_count_ = 0;
    consecutive_events_ = 0;
    observe(now);
  }

  // receiver 상태 변화를 확인한다(조회만). 같은 세대 안의 ARMED<->ACTIVE는 알림 대상이 아니다.
  void observe(std::int64_t now) {
    if (!time_ok(now)) return;
    const auto generation = receiver_.generation();
    const auto state = receiver_.state();
    const char* reason = receiver_.reason();
    const bool generation_changed = generation != seen_generation_;
    const bool latched_changed = (state != seen_state_ || !tx_detail::same_text(reason, seen_reason_)) &&
                                 !(tx_detail::armed_state(state) && tx_detail::armed_state(seen_state_));
    // 첫 관측 때 현재 상태를 한 번 알린다(부팅 알림). 첫 offer는 첫 pump의 기한 도래로 처리한다.
    if (generation_changed || latched_changed || !announced_) {
      announced_ = true;
      if (!state_dirty_) { state_dirty_ = true; state_since_ = now; }
      ++state_changes_pending_;
      ++stats_.state_changes;
    }
    if (generation_changed && !urgent_offer_) { urgent_offer_ = true; urgent_since_ = now; }
    seen_generation_ = generation;
    seen_state_ = state;
    seen_reason_ = reason;
  }

  // sink(const uint8_t* data, size_t size) -> 이번에 받은 바이트 수(0..size). 0/부분 수락은 backpressure다.
  // receiver.offer(now)를 호출할 수 있으므로 receiver 소유 작업에서만 부른다(offer도 receiver 시계 검사를 수행).
  template<class Sink>
  TxPumpResult pump(std::int64_t now, Sink&& sink) {
    TxPumpResult result;
    if (!time_ok(now) || !link_up_) return result;
    observe(now);
    for (;;) {
      if (frame_size_ != 0) {
        const std::size_t remaining = frame_size_ - frame_offset_;
        const std::size_t accepted = sink(frame_.data() + frame_offset_, remaining);
        if (accepted > remaining) {  // 호출자 계약 위반: 몇 바이트가 나갔는지 알 수 없다
          fault_ = TxFault::kSinkContract;
          ++stats_.aborted_frames;
          stats_.aborted_bytes += remaining;
          frame_size_ = frame_offset_ = 0;
          return result;
        }
        frame_offset_ += accepted;
        result.bytes_written += accepted;
        if (frame_offset_ != frame_size_) {
          if (accepted == 0) ++stats_.sink_zero_writes; else ++stats_.sink_partial_writes;
          result.backpressure = true;
          return result;
        }
        const auto k = static_cast<std::size_t>(current_.kind);
        ++stats_.completed[k];
        if (now - current_.materialize_us > stats_.max_inflight_us[k]) stats_.max_inflight_us[k] = now - current_.materialize_us;
        frame_size_ = frame_offset_ = 0;
        ++result.completed;
      }
      if (result.materialized == kTxMaxFramesPerPump) return result;
      if (!materialize(now)) return result;
      result.frames[result.materialized++] = current_;
    }
  }

  // 최악 payload 크기. 실제 인코더와 같은 함수를 값 무시 writer로 따라가 계산한다. 모두 kMaxPayload 이하여야 한다.
  std::size_t worst_payload(TxKind kind) const {
    tx_detail::WorstWriter w;
    const AckEntry ack{};
    switch (kind) {
      case TxKind::kAck: write_ack(w, ack); break;
      case TxKind::kState: write_state_fields(w, nullptr, 0, 0, ReceiverState::kDisarmed, "", 0); break;
      case TxKind::kOffer: write_offer_fields(w, nullptr, 0, 0, ReceiverState::kDisarmed, nullptr, Lease{}, false); break;
      case TxKind::kDiagnostic: write_diag_fields(w, 0); break;
      case TxKind::kNone: return 0;
    }
    return w.size();
  }

  TxFault fault() const { return fault_; }
  const TxStats& stats() const { return stats_; }
  std::size_t ack_count() const { return ack_count_; }
  bool state_pending() const { return state_dirty_; }
  bool urgent_offer_pending() const { return urgent_offer_; }
  bool frame_in_flight() const { return frame_size_ != 0; }
  std::size_t in_flight_remaining() const { return frame_size_ - frame_offset_; }
  const TxFrameRecord& current_frame() const { return current_; }

 private:
  struct AckEntry {
    StoredString request;
    ReceiveResult result{false, ReceiveReason::kInvalidMessage};
    bool stop = false;
    std::int64_t enqueued_us = 0;
  };

  bool time_ok(std::int64_t now) {
    if (fault_ != TxFault::kNone) return false;
    if (now < 0 || now > kMaxNowUs || (has_now_ && now < now_)) { fault_ = TxFault::kInvalidClock; return false; }
    now_ = now;
    has_now_ = true;
    return true;
  }
  void count_event() { if (consecutive_events_ < UINT8_MAX) ++consecutive_events_; }
  void erase_ack(std::size_t index) {
    for (std::size_t i = index + 1; i < ack_count_; ++i) acks_[i-1] = acks_[i];
    --ack_count_;
  }
  bool offer_due(std::int64_t now) const { return urgent_offer_ || !has_last_offer_ || now - last_offer_ >= config_.offer_period_us; }
  bool diag_due(std::int64_t now) const {
    return config_.diag_period_us != 0 && (!has_last_diag_ || now - last_diag_ >= config_.diag_period_us);
  }
  std::int64_t offer_trigger(std::int64_t now) const {
    std::int64_t due = has_last_offer_ ? last_offer_ + config_.offer_period_us : now;
    if (due > now) due = now;
    return urgent_offer_ && urgent_since_ < due ? urgent_since_ : due;
  }

  // 우선순위: ack(STOP 결과 먼저) > 세대 변화 offer > state > 주기 offer > diag.
  // 세대가 바뀌면 이전 임대가 모두 무효이므로 새 임대를 실은 offer를 state 알림보다 먼저 보낸다(offer도 state/세대를 싣는다).
  // ack/state가 연속 상한에 닿았고 offer가 기다리면 offer를 먼저 보낸다(기아 방지).
  bool materialize(std::int64_t now) {
    const bool offer_wait = offer_due(now);
    const bool events_wait = ack_count_ != 0 || state_dirty_;
    const bool guard = offer_wait && events_wait && consecutive_events_ >= config_.max_consecutive_events;
    if (guard) ++stats_.starvation_guard_used;
    if (!guard && ack_count_ != 0) return emit_ack(now);
    if (urgent_offer_ && emit_offer(now)) return true;
    if (!guard && state_dirty_) return emit_state(now);
    if (offer_due(now)) {
      if (emit_offer(now)) return true;
      // 임대를 만들 수 없으면(시계/세대 소진) 다음 순위로 넘어간다. 이 offer는 다음 주기까지 다시 시도하지 않는다.
      if (state_dirty_) return emit_state(now);
    }
    if (diag_due(now) && !diag_wait_offer_) return emit_diag(now);
    return false;
  }

  bool emit_ack(std::int64_t now) {
    std::size_t index = 0;
    for (std::size_t i = 0; i < ack_count_; ++i) if (acks_[i].stop) { index = i; break; }
    const AckEntry entry = acks_[index];
    erase_ack(index);
    tx_detail::JsonWriter w(frame_.data() + 4, kMaxPayload);
    write_ack(w, entry);
    TxFrameRecord record;
    record.kind = TxKind::kAck;
    record.trigger_us = entry.enqueued_us;
    record.generation = receiver_.generation();
    record.stop_ack = entry.stop;
    count_event();
    return seal(w, record, now);
  }
  bool emit_state(std::int64_t now) {
    tx_detail::JsonWriter w(frame_.data() + 4, kMaxPayload);
    const auto& boot = receiver_.boot();
    write_state_fields(w, boot.code_points.data(), boot.length, receiver_.generation(), receiver_.state(),
                       receiver_.reason(), state_changes_pending_);
    TxFrameRecord record;
    record.kind = TxKind::kState;
    record.trigger_us = state_since_;
    record.generation = receiver_.generation();
    if (state_changes_pending_ > 1) stats_.state_merged += state_changes_pending_ - 1;
    state_dirty_ = false;
    state_changes_pending_ = 0;
    count_event();
    return seal(w, record, now);
  }
  bool emit_offer(std::int64_t now) {
    const std::int64_t trigger = offer_trigger(now);
    const bool urgent = urgent_offer_;
    // 임대는 지금 발행한다. offer()가 시계 검사로 상태를 바꿀 수 있으므로 그 뒤의 현재 상태를 싣는다.
    const OfferResult lease = receiver_.offer(now);
    observe(now);
    has_last_offer_ = true;
    last_offer_ = now;
    if (!lease.ok) { urgent_offer_ = false; ++stats_.offer_unavailable; return false; }
    urgent_offer_ = false;  // 방금 만든 offer가 현재 세대를 싣는다
    diag_wait_offer_ = false;
    if (urgent) ++stats_.offers_urgent;
    tx_detail::JsonWriter w(frame_.data() + 4, kMaxPayload);
    const auto& boot = receiver_.boot();
    write_offer_fields(w, boot.code_points.data(), boot.length, receiver_.generation(), receiver_.state(),
                       receiver_.has_owner() ? &receiver_.owner() : nullptr, lease.lease, output_ready_);
    TxFrameRecord record;
    record.kind = TxKind::kOffer;
    record.trigger_us = trigger;
    record.generation = receiver_.generation();
    record.lease = lease.lease.id;
    record.urgent = urgent;
    consecutive_events_ = 0;
    return seal(w, record, now);
  }
  bool emit_diag(std::int64_t now) {
    tx_detail::JsonWriter w(frame_.data() + 4, kMaxPayload);
    write_diag_fields(w, now);
    if (config_.line_ns_per_byte != 0 && has_last_offer_ && w.ok()) {
      // 아직 아무 바이트도 내보내지 않은 내용이므로 버려도 프레임 교체가 아니다. 다음 offer 뒤 다시 시도한다.
      const std::int64_t room_us = last_offer_ + config_.offer_period_us - now;
      const std::int64_t need_us = static_cast<std::int64_t>((w.size() + kFrameOverhead)) * config_.line_ns_per_byte / 1000;
      if (need_us > room_us) { diag_wait_offer_ = true; ++stats_.diag_deferred_for_offer; return false; }
    }
    std::int64_t trigger = now;
    if (has_last_diag_) {
      trigger = last_diag_ + config_.diag_period_us;
      const std::int64_t missed = (now - last_diag_) / config_.diag_period_us;
      if (missed > 1) stats_.diag_skipped_periods += static_cast<std::uint64_t>(missed - 1);
    }
    has_last_diag_ = true;
    last_diag_ = now;
    TxFrameRecord record;
    record.kind = TxKind::kDiagnostic;
    record.trigger_us = trigger;
    record.generation = receiver_.generation();
    consecutive_events_ = 0;
    return seal(w, record, now);
  }

  // payload가 이미 frame_[4..]에 있다. 헤더/CRC를 붙여 in-flight로 고정한다(encode_frame과 같은 형식).
  bool seal(const tx_detail::JsonWriter& w, TxFrameRecord record, std::int64_t now) {
    const std::size_t size = w.size();
    if (!w.ok() || size == 0 || size > kMaxPayload) { ++stats_.encode_failures; return false; }
    frame_[0] = 'I'; frame_[1] = 'A';
    frame_[2] = static_cast<std::uint8_t>(size & 0xFFu);
    frame_[3] = static_cast<std::uint8_t>((size >> 8) & 0xFFu);
    const auto crc = crc32(frame_.data() + 2, size + 2);
    for (unsigned i = 0; i < 4; ++i) frame_[4 + size + i] = static_cast<std::uint8_t>((crc >> (8*i)) & 0xFFu);
    frame_size_ = size + kFrameOverhead;
    frame_offset_ = 0;
    record.frame_id = ++frame_counter_;
    record.size = frame_size_;
    record.materialize_us = now;
    const auto k = static_cast<std::size_t>(record.kind);
    ++stats_.frames[k];
    stats_.bytes[k] += frame_size_;
    const std::int64_t dwell = now - record.trigger_us;
    if (dwell > stats_.max_dwell_us[k]) stats_.max_dwell_us[k] = dwell;
    current_ = record;
    return true;
  }

  template<class W>
  static void write_ack(W& w, const AckEntry& e) {
    w.lit("{\"type\":\"ack\",\"request\":");
    w.stored(e.request);
    w.lit(",\"accepted\":");
    w.boolean(e.result.accepted);
    w.lit(",\"reason\":");
    w.text(receive_reason_name(e.result.reason));
    w.lit("}");
  }
  template<class W>
  static void write_state_fields(W& w, const std::uint32_t* boot, std::size_t boot_length, std::int64_t generation,
                                 ReceiverState state, const char* reason, std::uint64_t changes) {
    w.lit("{\"type\":\"state\",\"v\":1,\"boot\":");
    w.code_points(boot, boot_length);
    w.lit(",\"generation\":");
    w.i64(generation);
    w.lit(",\"state\":");
    w.text(tx_detail::state_name(state));
    w.lit(",\"reason\":");
    w.text(reason);
    w.lit(",\"changes\":");
    w.u64_saturated(changes);
    w.lit("}");
  }
  // 호스트(ActuationHost.on_rx, CommandProducer)가 읽는 필드만. 진단 필드는 diag로 분리한다.
  template<class W>
  static void write_offer_fields(W& w, const std::uint32_t* boot, std::size_t boot_length, std::int64_t generation,
                                 ReceiverState state, const StoredString* owner, const Lease& lease, bool output_ready) {
    w.lit("{\"type\":\"offer\",\"v\":1,\"boot\":");
    w.code_points(boot, boot_length);
    w.lit(",\"generation\":");
    w.i64(generation);
    w.lit(",\"state\":");
    w.text(tx_detail::state_name(state));
    w.lit(",\"owner\":");
    w.opt_stored(owner);
    w.lit(",\"lease\":");
    w.i64(lease.id);
    w.lit(",\"lease_issued_us\":");
    w.i64(lease.issued_us);
    w.lit(",\"lease_expires_us\":");
    w.i64(lease.expires_us);
    w.lit(",\"output_ready\":");
    w.boolean(output_ready);
    w.lit("}");
  }
  template<class W>
  void write_diag_fields(W& w, std::int64_t now) const {
    const auto& r = receiver_;
    const auto& boot = r.boot();
    std::int64_t micro = 0;
    w.lit("{\"type\":\"diag\",\"v\":1,\"boot\":");
    w.code_points(boot.code_points.data(), boot.length);
    w.lit(",\"now_us\":");
    w.i64(now);
    w.lit(",\"generation\":");
    w.i64(r.generation());
    w.lit(",\"state\":");
    w.text(tx_detail::state_name(r.state()));
    w.lit(",\"reason\":");
    w.text(r.reason());
    w.lit(",\"accepted_seq\":");
    w.i64(r.accepted_seq());
    w.lit(",\"target_speed_umps\":");
    // 인수 평가 순서가 정해져 있지 않으므로 변환을 먼저 끝낸 뒤 쓴다.
    bool has_micro = tx_detail::micro_units(r.target_speed(), micro);
    w.opt_i64(has_micro, micro);
    w.lit(",\"steering_urad\":");
    has_micro = tx_detail::micro_units(r.steering(), micro);
    w.opt_i64(has_micro, micro);
    w.lit(",\"deadline_us\":");
    w.opt_i64(r.has_deadline(), r.deadline());
    w.lit(",\"estop_held\":");
    w.boolean(r.estop_held());
    w.lit(",\"feedback_mode\":");
    w.text(r.motor_mode() ? "drive_motor_shaft" : "rear_wheel_pair_legacy");
    w.lit(",\"feedback_valid\":");
    w.boolean(r.fresh_feedback(now));
    w.lit(",\"feedback_seq\":");
    w.opt_i64(r.has_feedback(), r.feedback_seq());
    w.lit(",\"feedback_capture_us\":");
    w.opt_i64(r.has_feedback(), r.feedback_capture());
    w.lit(",\"motor_counts\":");
    w.opt_i64(r.has_feedback() && r.motor_mode(), r.feedback_count(0));
    w.lit(",\"left_ticks\":");
    w.opt_i64(r.has_feedback() && !r.motor_mode(), r.feedback_count(0));
    w.lit(",\"right_ticks\":");
    w.opt_i64(r.has_feedback() && !r.motor_mode(), r.feedback_count(1));
    w.lit(",\"mean_wheel_umps\":");
    has_micro = tx_detail::micro_units(r.mean_mps(), micro);
    w.opt_i64(has_micro, micro);
    w.lit(",\"output_ready\":");
    w.boolean(output_ready_);
    w.lit(",\"deadline_miss_count\":");
    w.u64_saturated(r.deadline_miss_count());
    w.lit(",\"lease_count\":");
    w.i64(static_cast<std::int64_t>(r.lease_count()));
    w.lit(",\"tx\":{\"ack_dropped\":");
    w.u64_saturated(stats_.ack_dropped_overflow);
    w.lit(",\"ack_evicted\":");
    w.u64_saturated(stats_.ack_evicted_for_stop);
    w.lit(",\"ack_purged\":");
    w.u64_saturated(stats_.ack_purged_session);
    w.lit(",\"state_merged\":");
    w.u64_saturated(stats_.state_merged);
    w.lit(",\"diag_skipped\":");
    w.u64_saturated(stats_.diag_skipped_periods);
    w.lit(",\"aborted_frames\":");
    w.u64_saturated(stats_.aborted_frames);
    w.lit(",\"encode_failures\":");
    w.u64_saturated(stats_.encode_failures);
    w.lit("}}");
  }

  TxConfig config_;
  ActuationReceiverCore& receiver_;
  TxFault fault_ = TxFault::kNone;
  bool output_ready_ = false;
  bool link_up_ = true;
  std::int64_t now_ = 0;
  bool has_now_ = false;

  std::array<AckEntry, kTxAckCapacity> acks_{};
  std::size_t ack_count_ = 0;
  std::uint8_t consecutive_events_ = 0;

  bool state_dirty_ = false;
  std::int64_t state_since_ = 0;
  std::uint64_t state_changes_pending_ = 0;
  std::int64_t seen_generation_ = 0;
  ReceiverState seen_state_ = ReceiverState::kDisarmed;
  const char* seen_reason_ = nullptr;
  bool announced_ = false;

  bool urgent_offer_ = false;
  std::int64_t urgent_since_ = 0;
  bool has_last_offer_ = false;
  std::int64_t last_offer_ = 0;
  bool has_last_diag_ = false;
  bool diag_wait_offer_ = false;
  std::int64_t last_diag_ = 0;

  std::array<std::uint8_t, kMaxFrame> frame_{};
  std::size_t frame_size_ = 0;
  std::size_t frame_offset_ = 0;
  std::uint64_t frame_counter_ = 0;
  TxFrameRecord current_;
  TxStats stats_;
};

}  // namespace arena_actuation
