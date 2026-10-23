#pragma once
// 출력 비활성 UART bring-up 주기 어댑터. 실제 SDK 드라이버/GPIO/PWM/타이머 구현이 아니다.
// ActuationBenchEndpoint와 같은 소유 작업에서만 run_cycle()을 호출한다. ISR/다른 task 재진입 금지.
#include "actuation_bench.hpp"

namespace arena_actuation {

// ---- 외부 I/O 계획 -------------------------------------------------------------------------
// 실물 배선(납품 판본·핀·포트·baud)이 확인되지 않았으므로 기본값은 비활성이다. 여기의 ESP32-S3 표는
// 칩 자료의 공통 제약만 담으며 VND018 실제 배선을 확정하지 않는다.
namespace esp32s3 {
constexpr int kUartPorts = 3;                       // UART0..2
constexpr std::uint32_t kMinBaud = 9600;
constexpr std::uint32_t kMaxBaud = 5'000'000;
inline bool gpio_exists(int gpio) { return (gpio >= 0 && gpio <= 21) || (gpio >= 26 && gpio <= 48); }
// 직접 USB(19/20), SPI flash·octal PSRAM(26~37, N16R8), strapping(0/3/45/46)은 통신 핀으로 쓰지 않는다.
inline bool gpio_reserved(int gpio) {
  return gpio == 0 || gpio == 3 || gpio == 19 || gpio == 20 || (gpio >= 26 && gpio <= 37) ||
         gpio == 45 || gpio == 46;
}
}  // namespace esp32s3

struct UartPortPlan {
  bool enabled = false;          // false면 어떤 UART/GPIO도 초기화하지 않는다
  bool wiring_confirmed = false; // 사람이 실물 판본·배선·상대 장치를 확인했다는 명시 표시
  int port = -1;
  int tx_gpio = -1;
  int rx_gpio = -1;
  std::uint32_t baud = 0;        // 기본값 없음. 8N1·흐름 제어 없음
  // 진단 콘솔. sdkconfig에서 채운다(-1: UART 콘솔 없음). 프로토콜과 같은 포트/핀을 쓰면 거절한다.
  int console_port = -1;
  int console_tx_gpio = -1;
  int console_rx_gpio = -1;
  // UART0는 ROM/부트로더 로그가 리셋 직후 나갈 수 있다. 실물에서 침묵을 확인하기 전에는 거절한다.
  bool uart0_boot_output_quiet_confirmed = false;
  // 모터/엔코더/서보/IMU 등 다른 기능에 이미 배정한 GPIO. 확정 배선표가 생기면 호출자가 채운다.
  const int* assigned_gpio = nullptr;
  std::size_t assigned_count = 0;
};

enum class UartPlanStatus : std::uint8_t {
  kDisabled = 0, kReady, kNotConfirmed, kInvalidPort, kConsoleConflict, kBootOutputUnconfirmed,
  kInvalidPin, kReservedPin, kPinConflict, kInvalidBaud,
};

inline const char* plan_status_name(UartPlanStatus s) {
  switch (s) {
    case UartPlanStatus::kDisabled: return "disabled";
    case UartPlanStatus::kReady: return "ready";
    case UartPlanStatus::kNotConfirmed: return "wiring_not_confirmed";
    case UartPlanStatus::kInvalidPort: return "invalid_port";
    case UartPlanStatus::kConsoleConflict: return "console_conflict";
    case UartPlanStatus::kBootOutputUnconfirmed: return "uart0_boot_output_unconfirmed";
    case UartPlanStatus::kInvalidPin: return "invalid_pin";
    case UartPlanStatus::kReservedPin: return "reserved_pin";
    case UartPlanStatus::kPinConflict: return "pin_conflict";
    case UartPlanStatus::kInvalidBaud: return "invalid_baud";
  }
  return "unknown";
}

// kReady일 때만 드라이버를 설치할 수 있다. 통과해도 전기적 적합성·상대 장치 설정을 확인한 것은 아니다.
inline UartPlanStatus check_uart_plan(const UartPortPlan& p) {
  if (!p.enabled) return UartPlanStatus::kDisabled;
  if (!p.wiring_confirmed) return UartPlanStatus::kNotConfirmed;
  if (p.port < 0 || p.port >= esp32s3::kUartPorts) return UartPlanStatus::kInvalidPort;
  if (p.console_port == p.port) return UartPlanStatus::kConsoleConflict;
  if (p.port == 0 && !p.uart0_boot_output_quiet_confirmed) return UartPlanStatus::kBootOutputUnconfirmed;
  if (!esp32s3::gpio_exists(p.tx_gpio) || !esp32s3::gpio_exists(p.rx_gpio) || p.tx_gpio == p.rx_gpio)
    return UartPlanStatus::kInvalidPin;
  if (esp32s3::gpio_reserved(p.tx_gpio) || esp32s3::gpio_reserved(p.rx_gpio)) return UartPlanStatus::kReservedPin;
  if (p.console_port >= 0) {
    for (int pin : {p.console_tx_gpio, p.console_rx_gpio})
      if (pin == p.tx_gpio || pin == p.rx_gpio) return UartPlanStatus::kConsoleConflict;
  }
  if (p.assigned_count != 0 && p.assigned_gpio == nullptr) return UartPlanStatus::kPinConflict;
  for (std::size_t i = 0; i < p.assigned_count; ++i)
    if (p.assigned_gpio[i] == p.tx_gpio || p.assigned_gpio[i] == p.rx_gpio) return UartPlanStatus::kPinConflict;
  if (p.baud < esp32s3::kMinBaud || p.baud > esp32s3::kMaxBaud) return UartPlanStatus::kInvalidBaud;
  return UartPlanStatus::kReady;
}

// ---- 주기 어댑터 ---------------------------------------------------------------------------
enum class UartIoEvent : std::uint8_t {
  kData = 0, kRxFifoOverflow, kRxBufferFull, kFrameError, kParityError, kBreak, kOther,
};
enum class UartBenchFault : std::uint8_t { kNone = 0, kInvalidConfig, kClockRegressed, kIoError, kLinkFault };

constexpr std::size_t kUartMaxRxPerCycle = 4096;
constexpr std::uint8_t kUartMaxEventsPerCycle = 32;

struct UartBenchConfig {
  // 모두 호출자가 명시한다(운영값 아님). 0이면 구성 오류.
  std::int64_t control_period_us = 0;       // 제어 tick 간격. receiver max_control_gap_us보다 작아야 한다
  // 새 읽기를 시작하지 않는 시간 상한(<= 제어 간격). 주기 전체 마감 보장이 아니다(tick·송신·폐기 시간 별도).
  std::int64_t cycle_budget_us = 0;
  std::size_t max_rx_bytes_per_cycle = 0;   // 1..kUartMaxRxPerCycle
  std::uint8_t max_events_per_cycle = 0;    // 한 run_cycle 전체에서 꺼낼 사건 수 2..kUartMaxEventsPerCycle
  // 새 읽기를 시작하려면 남아 있어야 하는 사건 예산(읽은 뒤 확인용). 1..max_events_per_cycle-1
  std::uint8_t post_read_event_reserve = 0;
  std::size_t event_queue_capacity = 0;     // 드라이버 이벤트 큐 길이. 가득 차 있으면 유실 가능으로 본다

  bool valid() const {
    return control_period_us >= 1 && control_period_us <= kMaxConfigUs && cycle_budget_us >= 1 &&
           cycle_budget_us <= control_period_us && max_rx_bytes_per_cycle >= 1 &&
           max_rx_bytes_per_cycle <= kUartMaxRxPerCycle && max_events_per_cycle >= 2 &&
           max_events_per_cycle <= kUartMaxEventsPerCycle && post_read_event_reserve >= 1 &&
           post_read_event_reserve < max_events_per_cycle && event_queue_capacity >= 1 &&
           event_queue_capacity <= 1024;
  }
};

struct UartCycleReport {
  UartBenchFault fault = UartBenchFault::kNone;
  std::int64_t start_us = -1;          // -1: 이번 주기 시각을 얻지 못함
  std::int64_t end_us = -1;            // 마지막 작업(송신·폐기·단절 포함) 뒤 다시 읽은 시각
  bool ticked = false;                 // 이번 주기에 control_tick을 정확히 한 번 호출했다
  std::int64_t tick_gap_us = 0;        // 직전 control_tick과의 실제 간격(따라잡기 없음)
  std::uint64_t missed_periods = 0;    // 이번 tick 직전까지 지나간 기한 수(재생하지 않음)
  std::size_t rx_bytes = 0;            // 연결부에 전달한 바이트
  std::size_t rx_discarded = 0;        // 오류/미확인 사건 때문에 버린 바이트(이미 고정 버퍼로 읽은 것만 셈)
  std::size_t events = 0;              // 이번 주기에 꺼낸 사건 수(<= max_events_per_cycle)
  bool session_break = false;
  bool break_deferred = false;         // 두 번째 경계를 다음 주기 시작으로 미뤘다
  bool rx_byte_budget_exhausted = false;
  bool rx_time_budget_exhausted = false;
  bool rx_event_budget_exhausted = false;  // 사건을 다 확인하지 못해 읽기를 멈췄거나 읽은 조각을 버렸다
  std::size_t tx_bytes = 0;
  bool tx_backpressure = false;
};

struct UartBenchStats {
  std::uint64_t cycles = 0, control_ticks = 0, missed_periods = 0, overrun_ticks = 0;
  std::int64_t max_tick_gap_us = 0, max_tick_lateness_us = 0;
  std::int64_t max_cycle_us = 0;                      // start_us→end_us 최대값(주기 실행시간)
  std::uint64_t cycles_over_budget = 0, cycles_over_period = 0;
  std::uint64_t rx_bytes = 0, rx_reads = 0, rx_discarded = 0, rx_unconfirmed_discarded = 0;
  std::uint64_t rx_byte_budget_cycles = 0, rx_time_budget_cycles = 0, rx_event_budget_cycles = 0;
  std::uint64_t events = 0, error_events = 0, event_queue_saturated = 0, session_breaks = 0, deferred_breaks = 0;
  std::uint64_t tx_bytes = 0, tx_zero_writes = 0, tx_partial_writes = 0;
  std::int64_t last_rx_buffered_us = -1;  // 마지막 바이트가 고정 버퍼에 들어온 직후 읽은 시각(주기 종료와 별개)
};

// Io 개념(모두 비차단·같은 소유 작업에서만, 사건 큐의 소비자는 이 작업 하나뿐):
//   int read(std::uint8_t* dst, std::size_t max)        -> -1 오류, 0..max 바이트
//   int write(const std::uint8_t* src, std::size_t n)   -> -1 오류, 0..n 수락(TX FIFO 수락이지 선로 송출 완료 아님)
//   bool next_event(UartIoEvent& out)                    -> 대기 없이 하나 꺼냄, 없으면 false
//   std::size_t pending_events()                         -> 큐에 남은 사건 수
//   bool discard_input()                                 -> 사건 큐 비움 뒤 드라이버 RX 버퍼 폐기
// Clock 개념: std::int64_t now_us() (부팅 이후 단조 µs, 0 이상)
template<class Io, class Clock>
class UartBenchAdapter {
 public:
  // endpoint/io/clock은 이 객체보다 오래 살아야 한다. io_enabled=false면 Io를 한 번도 호출하지 않는다.
  UartBenchAdapter(ActuationBenchEndpoint& endpoint, const UartBenchConfig& config, Io& io, Clock& clock,
                   bool io_enabled)
      : endpoint_(endpoint), config_(config), io_(io), clock_(clock), io_enabled_(io_enabled) {
    if (!config_.valid()) fault_ = UartBenchFault::kInvalidConfig;
  }
  UartBenchAdapter(const UartBenchAdapter&) = delete;
  UartBenchAdapter& operator=(const UartBenchAdapter&) = delete;

  // 한 번의 유한 주기: 시각 → (기한이면) control_tick 1회 → (미룬 경계) → [사건 확인 → 읽기 → 사건 확인 → feed]
  // 반복 → 조립 기한 → 송신 1회 → 종료 시각. 고장 뒤에는 아무 I/O·tick도 하지 않는다(침묵). 재개는 새 객체로만.
  UartCycleReport run_cycle() {
    UartCycleReport r;
    r.fault = fault_;
    if (fault_ != UartBenchFault::kNone) return r;
    std::int64_t now = 0;
    if (!read_clock(now)) return finish(r);
    r.start_us = now;
    // 제어 tick은 통신 도착과 무관하게 시간 기한으로만 호출한다. 늦어도 한 번만 호출하고 기한을 다시 잡는다.
    if (!has_tick_ || now >= next_tick_) control_tick(now, r);
    if (!io_enabled_) return finish(r);
    if (!connected_) {
      if (!link_ok(endpoint_.connect(now))) return finish(r);
      connected_ = true;
    }
    if (break_pending_) {
      break_pending_ = false;
      if (!session_break(now, r)) return finish(r);
    }
    // 사건 예산은 이 주기 전체의 상한이다. 수신은 바이트·시간·사건 예산 모두 유한하다.
    // 첫 읽기(최대 256 B)는 시간 상한과 무관하게 한 번 허용해 느린 tick 때문에 수신이 영구히 굶지 않게 한다.
    std::uint32_t events_left = 4u * config_.max_events_per_cycle;
    bool first_read = true;
    while (r.rx_bytes + r.rx_discarded < config_.max_rx_bytes_per_cycle) {
      if (!read_clock(now)) return finish(r);
      if (!first_read && now - r.start_us >= config_.cycle_budget_us) { r.rx_time_budget_exhausted = true; break; }
      // 드라이버에서 바이트를 꺼내기 전에 그때까지 쌓인 사건을 모두 확인한다.
      const auto before = check_events(r, events_left);
      if (before == EventCheck::kError) {
        if (!on_error(now, r)) return finish(r);
        if (r.break_deferred) break;
        continue;
      }
      if (before == EventCheck::kUnconfirmed || events_left < config_.post_read_event_reserve) {
        r.rx_event_budget_exhausted = true;  // 바이트는 드라이버 버퍼에 남는다(이번 주기에는 꺼내지 않음)
        break;
      }
      first_read = false;
      std::size_t want = config_.max_rx_bytes_per_cycle - r.rx_bytes - r.rx_discarded;
      if (want > rx_.size()) want = rx_.size();
      const int got = io_.read(rx_.data(), want);
      ++stats_.rx_reads;
      if (got < 0 || static_cast<std::size_t>(got) > want) { io_fault(now); return finish(r); }
      if (got == 0) break;
      // 알 수 있는 시각은 바이트가 이 고정 버퍼에 들어온 뒤다. 선로 도착·원천 취득 시각이 아니다.
      std::int64_t buffered = 0;
      if (!read_clock(buffered)) return finish(r);
      const auto n = static_cast<std::size_t>(got);
      const auto after = check_events(r, events_left);
      if (after != EventCheck::kClean) {
        // 오류 사건 또는 예산 안에서 확인하지 못한 사건이 이 바이트보다 먼저인지 알 수 없다. 넣지 않고 세션을 끊는다.
        r.rx_discarded += n;
        stats_.rx_discarded += n;
        if (after == EventCheck::kUnconfirmed) {
          r.rx_event_budget_exhausted = true;
          stats_.rx_unconfirmed_discarded += n;
        }
        if (!on_error(buffered, r)) return finish(r);
        break;
      }
      if (!link_ok(endpoint_.feed(buffered, rx_.data(), n))) return finish(r);
      r.rx_bytes += n;
      stats_.rx_bytes += n;
      stats_.last_rx_buffered_us = buffered;
      if (n < want) break;  // 드라이버 버퍼가 비었다
    }
    if (r.rx_bytes + r.rx_discarded >= config_.max_rx_bytes_per_cycle) r.rx_byte_budget_exhausted = true;
    if (r.rx_byte_budget_exhausted) ++stats_.rx_byte_budget_cycles;
    if (r.rx_time_budget_exhausted) ++stats_.rx_time_budget_cycles;
    if (r.rx_event_budget_exhausted) ++stats_.rx_event_budget_cycles;
    if (!read_clock(now)) return finish(r);
    if (!link_ok(endpoint_.poll(now))) return finish(r);
    // 송신: 주기당 pump 한 번(pump 자체가 materialize 2개 이하). 0/부분 수락은 다음 주기로 넘긴다.
    bool write_error = false;
    const auto pumped = endpoint_.pump(now, [&](const std::uint8_t* data, std::size_t size) -> std::size_t {
      if (write_error) return 0;
      const int written = io_.write(data, size);
      if (written < 0 || static_cast<std::size_t>(written) > size) { write_error = true; return 0; }
      if (written == 0) ++stats_.tx_zero_writes;
      else if (static_cast<std::size_t>(written) < size) ++stats_.tx_partial_writes;
      return static_cast<std::size_t>(written);
    });
    r.tx_bytes = pumped.bytes_written;
    r.tx_backpressure = pumped.backpressure;
    stats_.tx_bytes += pumped.bytes_written;
    if (write_error) io_fault(now);
    return finish(r);
  }

  // 조회만 제공한다. mutable endpoint·피드백 주입·준비값 설정 API는 없다.
  const ActuationBenchEndpoint& endpoint() const { return endpoint_; }
  const UartBenchStats& stats() const { return stats_; }
  UartBenchFault fault() const { return fault_; }
  bool io_enabled() const { return io_enabled_; }
  static constexpr bool output_ready() { return false; }

 private:
  enum class EventCheck : std::uint8_t { kClean, kError, kUnconfirmed };

  bool read_clock(std::int64_t& out) {
    const std::int64_t now = clock_.now_us();
    if (now < 0 || now > kMaxNowUs || (has_now_ && now < last_now_)) {
      fault_ = UartBenchFault::kClockRegressed;
      // 수신기도 시계 고장을 기록하게 한다(구동 허가를 만들지 않는 기존 경로). 이후 침묵한다.
      if (now >= 0 && now <= kMaxNowUs) endpoint_.control_tick(now);
      return false;
    }
    last_now_ = now;
    has_now_ = true;
    out = now;
    return true;
  }
  void control_tick(std::int64_t now, UartCycleReport& r) {
    if (has_tick_) {
      r.tick_gap_us = now - last_tick_;
      r.missed_periods = static_cast<std::uint64_t>((now - next_tick_) / config_.control_period_us);
      if (r.missed_periods != 0) ++stats_.overrun_ticks;
      if (now - next_tick_ > stats_.max_tick_lateness_us) stats_.max_tick_lateness_us = now - next_tick_;
      if (r.tick_gap_us > stats_.max_tick_gap_us) stats_.max_tick_gap_us = r.tick_gap_us;
      stats_.missed_periods += r.missed_periods;
    }
    endpoint_.control_tick(now);
    ++stats_.control_ticks;
    r.ticked = true;
    // 지난 기한을 재생하지 않는다. 제때면 위상을 유지하고, 한 주기 이상 늦으면 현재 시각에서 다시 잡는다.
    next_tick_ = has_tick_ && now - next_tick_ < config_.control_period_us ? next_tick_ + config_.control_period_us
                                                                           : now + config_.control_period_us;
    last_tick_ = now;
    has_tick_ = true;
  }
  // 사건 확인(꺼내기는 left 예산 안에서만). 큐 소비자가 이 작업 하나뿐이고 ISR은 넣기만 하므로, 직전 확인 뒤
  // 큐가 가득 찼다면 다음 진입 때도 가득 차 있다(진입 검사). 꺼내는 도중 다시 가득 차 있으면 종료 검사가 본다.
  // 진입 검사와 종료 검사 사이에 가득 찼다가 우리가 꺼내 다시 내려간 순간 포화는 볼 수 없다. 모든 유실 검출이 아니다.
  EventCheck check_events(UartCycleReport& r, std::uint32_t& left) {
    if (io_.pending_events() >= config_.event_queue_capacity) {
      ++stats_.event_queue_saturated;
      return EventCheck::kError;  // 직전 확인 이후 ISR이 사건을 버렸을 수 있다. 세션 경계가 큐를 비운다
    }
    bool error = false;
    while (left != 0) {
      UartIoEvent event{};
      if (!io_.next_event(event)) break;
      --left;
      ++r.events;
      ++stats_.events;
      if (event != UartIoEvent::kData) { error = true; ++stats_.error_events; }
    }
    const std::size_t pending = io_.pending_events();
    if (pending >= config_.event_queue_capacity) {
      ++stats_.event_queue_saturated;
      error = true;
    }
    if (error) return EventCheck::kError;
    // 큐가 비었다고 본 직후 ISR이 넣은 사건도 마지막 조회에 보이면 내용을 모르는 사건이다. 비어 있을 때만 Clean.
    return pending == 0 ? EventCheck::kClean : EventCheck::kUnconfirmed;
  }
  // 한 주기에 세션 경계는 한 번만 수행한다. 두 번째 오류는 다음 주기 시작에서 먼저 끊도록 미룬다(잊지 않음).
  bool on_error(std::int64_t now, UartCycleReport& r) {
    if (!r.session_break) return session_break(now, r);
    break_pending_ = true;
    r.break_deferred = true;
    ++stats_.deferred_breaks;
    return true;
  }
  // 부분 프레임·송신 꼬리·ack를 버리고(연결부가 필요하면 로컬 STOP), 드라이버 입력을 비운 뒤 새 세션.
  // 재연결은 CLEAR/ARM을 만들지 않는다. 이미 TX FIFO에 들어간 바이트는 회수할 수 없다. 잘린 프레임은 호스트
  // CRC/재동기화가 버리지만, FIFO에 남은 완전한 이전 프레임은 CRC를 통과하므로 호스트의 요청/세션/세대 계약으로 배제해야 한다.
  bool session_break(std::int64_t now, UartCycleReport& r) {
    r.session_break = true;
    ++stats_.session_breaks;
    if (!link_ok(endpoint_.disconnect(now))) return false;
    connected_ = false;
    if (!io_.discard_input()) { io_fault(now); return false; }
    if (!link_ok(endpoint_.connect(now))) return false;
    connected_ = true;
    return true;
  }
  bool link_ok(const IngressResult& result) {
    if (result.link == LinkResult::kOk && result.fault == IngressFault::kNone) return true;
    fault_ = UartBenchFault::kLinkFault;
    return false;
  }
  void io_fault(std::int64_t now) {
    fault_ = UartBenchFault::kIoError;
    if (connected_) endpoint_.disconnect(now);  // 로컬 STOP·꼬리 폐기. 이후 Io를 다시 호출하지 않는다
    connected_ = false;
  }
  // 종료 시각은 마지막 작업(송신·입력 폐기·단절) 뒤 다시 읽는다. 이 읽기에서 역행해도 시계 고장으로 고정한다.
  UartCycleReport& finish(UartCycleReport& r) {
    if (r.start_us >= 0 && fault_ != UartBenchFault::kClockRegressed) {
      std::int64_t end = 0;
      if (read_clock(end)) r.end_us = end;
    }
    if (r.start_us >= 0) {
      if (r.end_us < 0) r.end_us = last_now_;  // 종료 읽기 실패: 마지막 정상 시각(실행시간 하한일 뿐)
      const std::int64_t elapsed = r.end_us - r.start_us;
      if (elapsed > stats_.max_cycle_us) stats_.max_cycle_us = elapsed;
      if (elapsed > config_.cycle_budget_us) ++stats_.cycles_over_budget;
      if (elapsed > config_.control_period_us) ++stats_.cycles_over_period;
    }
    r.fault = fault_;
    ++stats_.cycles;
    return r;
  }

  ActuationBenchEndpoint& endpoint_;
  const UartBenchConfig config_;
  Io& io_;
  Clock& clock_;
  const bool io_enabled_;
  UartBenchFault fault_ = UartBenchFault::kNone;
  std::array<std::uint8_t, kMaxInputPerCall> rx_{};
  bool connected_ = false;
  bool break_pending_ = false;
  bool has_now_ = false;
  std::int64_t last_now_ = 0;
  bool has_tick_ = false;
  std::int64_t last_tick_ = 0;
  std::int64_t next_tick_ = 0;
  UartBenchStats stats_{};
};

}  // namespace arena_actuation
