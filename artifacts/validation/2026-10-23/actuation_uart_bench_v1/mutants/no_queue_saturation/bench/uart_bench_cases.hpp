#pragma once
// 출력 비활성 UART 주기 어댑터의 공유 반례. 호스트와 ESP-IDF에서 같은 가짜 I/O·합성 시각만 쓴다.
// 실제 UART/GPIO를 열지 않으며 결과를 보드 실행이나 실물 배선 검증으로 해석하지 않는다.
#include "bench_cases.hpp"
#include "actuation_uart_bench.hpp"
#include <new>

namespace arena_smoke {

struct FakeClock {
  std::int64_t now = 0;
  std::int64_t step = 0;            // 각 읽기 뒤 증가(호출 사이 경과 시간 모사)
  long regress_on_call = -1;        // 이 번째 호출에서 직전보다 1 µs 작은 값을 돌려준다
  long calls = 0;
  std::int64_t now_us() {
    ++calls;
    if (calls == regress_on_call) return now - 1;
    const auto value = now;
    now += step;
    return value;
  }
};

struct FakeIo {
  std::array<std::uint8_t, 8192> rx{};
  std::size_t rx_begin = 0, rx_end = 0;
  bool endless_rx = false, read_fail = false, read_over = false;
  std::array<aa::UartIoEvent, 16> queue{};
  std::size_t queued = 0;
  bool endless_events = false;
  long inject_after_read = -1;      // 이 번째 read가 데이터를 돌려준 직후 사건이 보이게 한다
  aa::UartIoEvent inject_event = aa::UartIoEvent::kRxFifoOverflow;
  std::array<std::uint8_t, 16384> wire{};
  std::size_t wire_used = 0;
  std::size_t write_limit = static_cast<std::size_t>(-1);
  bool write_fail = false, write_over = false, discard_fail = false;
  unsigned long reads = 0, writes = 0, event_calls = 0, pending_calls = 0, discards = 0;

  unsigned long calls() const { return reads + writes + event_calls + pending_calls + discards; }
  bool push_rx(const std::uint8_t* data, std::size_t size) {
    if (size > rx.size() - rx_end) return false;
    std::memcpy(rx.data() + rx_end, data, size);
    rx_end += size;
    return true;
  }
  void push_event(aa::UartIoEvent event) {
    if (queued < queue.size()) queue[queued++] = event;
  }
  int read(std::uint8_t* dst, std::size_t max) {
    ++reads;
    if (read_fail) return -1;
    if (read_over) return static_cast<int>(max) + 1;
    std::size_t n = max;
    if (endless_rx) {
      std::memset(dst, 0x55, n);
    } else {
      if (n > rx_end - rx_begin) n = rx_end - rx_begin;
      std::memcpy(dst, rx.data() + rx_begin, n);
      rx_begin += n;
    }
    if (n != 0 && static_cast<long>(reads) == inject_after_read) push_event(inject_event);
    return static_cast<int>(n);
  }
  int write(const std::uint8_t* src, std::size_t size) {
    ++writes;
    if (write_fail) return -1;
    if (write_over) return static_cast<int>(size) + 1;
    std::size_t n = size < write_limit ? size : write_limit;
    if (n > wire.size() - wire_used) n = wire.size() - wire_used;
    std::memcpy(wire.data() + wire_used, src, n);
    wire_used += n;
    return static_cast<int>(n);
  }
  bool next_event(aa::UartIoEvent& out) {
    ++event_calls;
    if (endless_events) { out = aa::UartIoEvent::kData; return true; }
    if (queued == 0) return false;
    out = queue[0];
    for (std::size_t i = 1; i < queued; ++i) queue[i-1] = queue[i];
    --queued;
    return true;
  }
  std::size_t pending_events() {
    ++pending_calls;
    return endless_events ? 1000 : queued;
  }
  bool discard_input() {
    ++discards;
    queued = 0;
    rx_begin = rx_end;
    return !discard_fail;
  }
};

using UartAdapter = aa::UartBenchAdapter<FakeIo, FakeClock>;

// 경계 계약: 조회 전용·준비값 false·mutable endpoint/피드백 주입 없음.
template<class T, class = void> struct HasMutableEndpoint : std::false_type {};
template<class T> struct HasMutableEndpoint<T, std::void_t<decltype(std::declval<T&>().endpoint().control_tick(0))>>
    : std::true_type {};
template<class T, class = void> struct HasFeedbackInjection : std::false_type {};
template<class T> struct HasFeedbackInjection<T, std::void_t<decltype(std::declval<T&>().update_motor_encoder(
    0, aa::IntInput{}, aa::IntInput{}, aa::IntInput{}))>> : std::true_type {};
template<class T, class = void> struct HasReadySetter : std::false_type {};
template<class T> struct HasReadySetter<T, std::void_t<decltype(std::declval<T&>().set_output_ready(true))>>
    : std::true_type {};
static_assert(std::is_same<decltype(std::declval<UartAdapter&>().endpoint()),
                           const aa::ActuationBenchEndpoint&>::value, "adapter must expose a const endpoint");
static_assert(!HasMutableEndpoint<UartAdapter>::value, "endpoint must stay read-only through the adapter");
static_assert(!HasFeedbackInjection<UartAdapter>::value && !HasFeedbackInjection<aa::ActuationBenchEndpoint>::value,
              "no feedback injection path");
static_assert(!HasReadySetter<UartAdapter>::value && !HasReadySetter<aa::ActuationBenchEndpoint>::value,
              "no output readiness setter");
static_assert(!UartAdapter::output_ready() && !aa::ActuationBenchEndpoint::output_ready(), "output stays disabled");
static_assert(!aa::UartPortPlan{}.enabled, "external I/O is disabled by default");

inline aa::UartBenchConfig uart_config() {
  aa::UartBenchConfig c;                 // 합성 시험값. 실물 운영값 아님
  c.control_period_us = 5000;
  c.cycle_budget_us = 2000;
  c.max_rx_bytes_per_cycle = 1024;
  c.max_events_per_cycle = 8;
  c.event_queue_capacity = 16;
  return c;
}

// 큰 객체는 정적 저장 공간에서 사례마다 새로 만든다(호스트/대상 main이 소유).
struct UartScratch {
  alignas(aa::ActuationBenchEndpoint) unsigned char endpoint_storage[sizeof(aa::ActuationBenchEndpoint)];
  alignas(UartAdapter) unsigned char adapter_storage[sizeof(UartAdapter)];
  aa::ActuationBenchEndpoint* endpoint = nullptr;
  UartAdapter* adapter = nullptr;
  FakeIo io;
  FakeClock clock;
  aa::FrameDecoder decoder;
  unsigned wire_frames = 0;

  UartAdapter& make(bool io_enabled, const aa::UartBenchConfig& config = uart_config()) {
    destroy();
    io = FakeIo{};
    clock = FakeClock{};
    endpoint = new (endpoint_storage) aa::ActuationBenchEndpoint(motor_config(), kSmokeBoot, 10, 30000,
                                                                 aa::PayloadLayout::kEnvelope, tx_config());
    adapter = new (adapter_storage) UartAdapter(*endpoint, config, io, clock, io_enabled);
    return *adapter;
  }
  void destroy() {
    if (adapter) adapter->~UartAdapter();
    if (endpoint) endpoint->~ActuationBenchEndpoint();
    adapter = nullptr;
    endpoint = nullptr;
  }
  ~UartScratch() { destroy(); }
};

inline bool push_frame(Scratch& scratch, FakeIo& io, const char* payload, std::size_t keep = 0) {
  const auto size = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(payload), std::strlen(payload),
                                     scratch.frame.data(), scratch.frame.size());
  if (size == 0) return false;
  return io.push_rx(scratch.frame.data(), keep == 0 || keep > size ? size : keep);
}
inline bool push_rest(Scratch& scratch, FakeIo& io, const char* payload, std::size_t from) {
  const auto size = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(payload), std::strlen(payload),
                                     scratch.frame.data(), scratch.frame.size());
  return size > from && io.push_rx(scratch.frame.data() + from, size - from);
}
// 호스트 쪽 복호: CRC 통과 프레임 중 token을 담은 것이 있는가. 전체 wire를 처음부터 다시 읽는다.
inline bool wire_contains(UartScratch& f, const char* token) {
  f.decoder.reset();
  f.wire_frames = 0;
  bool seen = false;
  for (std::size_t i = 0; i < f.io.wire_used; i += aa::kMaxInputPerCall) {
    const auto n = f.io.wire_used - i < aa::kMaxInputPerCall ? f.io.wire_used - i : aa::kMaxInputPerCall;
    f.decoder.feed(f.io.wire.data() + i, n, [&](const std::uint8_t* p, std::size_t size) {
      ++f.wire_frames;
      seen = contains(p, size, token) || seen;
    });
  }
  return seen;
}
inline void control_text(Scratch& scratch, const char* request, const char* kind, std::int64_t generation,
                         std::int64_t lease, int seq) {
  std::snprintf(scratch.text, sizeof(scratch.text),
      "{\"type\":\"command\",\"request\":\"%s\",\"command\":{\"v\":1,\"kind\":\"%s\",\"boot\":\"smoke-only\","
      "\"generation\":%lld,\"lease\":%lld,\"client\":\"smoke-only\",\"seq\":%d}}",
      request, kind, static_cast<long long>(generation), static_cast<long long>(lease), seq);
}
inline aa::UartCycleReport cycle_at(UartScratch& f, std::int64_t now) {
  f.clock.now = now;
  return f.adapter->run_cycle();
}

constexpr unsigned kUartCases = 23;
inline const char* uart_case_name(unsigned i) {
  static const char* const names[kUartCases] = {
    "plan_default_disabled", "plan_rejections", "disabled_io_untouched", "invalid_config_silent",
    "tick_independent_of_rx", "late_tick_no_catch_up", "arm_and_drive_rejected", "stop_then_clear_rejected",
    "rx_byte_budget", "rx_time_budget", "event_flood_bounded", "overflow_discards_partial",
    "frame_error_after_read", "tx_zero_then_partial", "tx_error_latches", "tx_overreport_latches",
    "read_error_latches", "clock_regress_silent", "session_break_aborts_tail", "silence_is_not_detach",
    "discard_failure_latches", "stale_partial_expires", "rx_stamp_after_buffer",
  };
  return i < kUartCases ? names[i] : "unknown";
}

inline bool uart_case(unsigned index, Scratch& scratch, UartScratch& f) {
  using aa::UartPlanStatus;
  switch (index) {
    case 0: {  // 기본 계획은 비활성이며 콘솔/배선 확인 없이는 준비되지 않는다
      return aa::check_uart_plan(aa::UartPortPlan{}) == UartPlanStatus::kDisabled;
    }
    case 1: {
      static const int assigned[] = {5, 6};
      aa::UartPortPlan p;  // 합성 예: 콘솔 UART0(43/44), 프로토콜 UART1. 실제 배선 아님
      p.enabled = true; p.port = 1; p.tx_gpio = 17; p.rx_gpio = 18; p.baud = 460800;
      p.console_port = 0; p.console_tx_gpio = 43; p.console_rx_gpio = 44;
      bool ok = aa::check_uart_plan(p) == UartPlanStatus::kNotConfirmed;
      p.wiring_confirmed = true;
      ok = ok && aa::check_uart_plan(p) == UartPlanStatus::kReady;
      auto q = p; q.port = 0;
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kConsoleConflict;
      q.console_port = -1;  // 콘솔을 옮겨도 UART0 부팅 출력 침묵을 확인하기 전에는 거절
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kBootOutputUnconfirmed;
      q = p; q.tx_gpio = 43;
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kConsoleConflict;
      for (int pin : {0, 3, 19, 20, 26, 33, 37, 45, 46}) { q = p; q.rx_gpio = pin;
        ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kReservedPin; }
      for (int pin : {-1, 22, 25, 49}) { q = p; q.tx_gpio = pin;
        ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kInvalidPin; }
      q = p; q.rx_gpio = q.tx_gpio;
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kInvalidPin;
      q = p; q.port = 3;
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kInvalidPort;
      q = p; q.assigned_gpio = assigned; q.assigned_count = 2; q.tx_gpio = 6;
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kPinConflict;
      q = p; q.assigned_count = 1;
      ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kPinConflict;
      for (std::uint32_t baud : {0u, 9599u, 5'000'001u}) { q = p; q.baud = baud;
        ok = ok && aa::check_uart_plan(q) == UartPlanStatus::kInvalidBaud; }
      return ok;
    }
    case 2: {  // 외부 I/O 비활성: Io를 한 번도 부르지 않고 제어 tick만 시간 기한으로 돈다
      auto& a = f.make(false);
      for (std::int64_t t = 0; t <= 50000; t += 1000) cycle_at(f, t);
      const auto& e = a.endpoint();
      return f.io.calls() == 0 && a.stats().control_ticks == 11 && a.fault() == aa::UartBenchFault::kNone &&
             e.ingress().link().state() == aa::LinkState::kDisconnected && f.io.wire_used == 0 &&
             e.receiver().state() == aa::ReceiverState::kDisarmed && !e.receiver().armed();
    }
    case 3: {  // 잘못된 구성은 영구 침묵. tick/I/O 없음
      auto bad = uart_config();
      bad.cycle_budget_us = bad.control_period_us + 1;
      auto& a = f.make(true, bad);
      cycle_at(f, 0);
      cycle_at(f, 10000);
      return a.fault() == aa::UartBenchFault::kInvalidConfig && f.io.calls() == 0 && a.stats().control_ticks == 0 &&
             f.clock.calls == 0;
    }
    case 4: {  // 수신 바이트가 많아도 제어 tick은 시간 기한으로만. 통신 주기가 watchdog을 갱신하지 않는다
      auto& a = f.make(true);
      bool ok = true;
      for (std::int64_t t = 0; t < 20000; t += 1000) {
        const std::uint8_t noise[] = {0x00, 0x13, 0x37};
        ok = ok && f.io.push_rx(noise, sizeof(noise));
        const auto r = cycle_at(f, t);
        ok = ok && r.ticked == (t % 5000 == 0) && r.rx_bytes == sizeof(noise);
      }
      return ok && a.stats().control_ticks == 4 && a.stats().rx_bytes == 60 && a.fault() == aa::UartBenchFault::kNone;
    }
    case 5: {  // 37 ms 멈춤 뒤 tick은 한 번뿐이고 실제 공백·놓친 기한을 그대로 기록한다
      auto& a = f.make(true);
      cycle_at(f, 0);
      const auto late = cycle_at(f, 37000);
      const auto next = cycle_at(f, 38000);
      const auto due = cycle_at(f, 42000);
      return late.ticked && late.tick_gap_us == 37000 && late.missed_periods == 6 && !next.ticked && due.ticked &&
             due.tick_gap_us == 5000 && a.stats().control_ticks == 3 && a.stats().max_tick_gap_us == 37000 &&
             a.stats().missed_periods == 6 && a.stats().overrun_ticks == 1;
    }
    case 6: {  // UART로 ARM/DRIVE를 보내도 준비값 false 관문에서 거절·0목표
      auto& a = f.make(true);
      cycle_at(f, 220000);
      const auto& r = a.endpoint().receiver();
      if (r.lease_count() == 0) return false;
      control_text(scratch, "r-arm", "ARM", 0, r.lease_at(0).id, 1);
      bool ok = push_frame(scratch, f.io, scratch.text);
      cycle_at(f, 221000);
      cycle_at(f, 222000);
      ok = ok && wire_contains(f, "\"r-arm\"") && wire_contains(f, "output_not_ready") &&
           wire_contains(f, "\"output_ready\":false") && r.state() == aa::ReceiverState::kDisarmed &&
           r.generation() == 0;
      ok = ok && push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-drive\",\"command\":{\"v\":1,"
           "\"kind\":\"DRIVE\",\"boot\":\"smoke-only\",\"generation\":0,\"lease\":1,\"client\":\"smoke-only\","
           "\"seq\":2,\"speed_mps\":0.5,\"steering_rad\":0.2,\"source_age_us\":0,\"ttl_us\":100000}}");
      cycle_at(f, 223000);
      cycle_at(f, 224000);
      return ok && wire_contains(f, "\"r-drive\"") && r.target_speed() == 0. && !r.armed() &&
             a.fault() == aa::UartBenchFault::kNone;
    }
    case 7: {  // STOP은 고정, 피드백 없는 CLEAR는 거절
      auto& a = f.make(true);
      cycle_at(f, 220000);
      const auto& r = a.endpoint().receiver();
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-stop\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      cycle_at(f, 221000);
      cycle_at(f, 222000);
      ok = ok && r.state() == aa::ReceiverState::kEstop && wire_contains(f, "\"r-stop\"") && r.lease_count() > 0;
      control_text(scratch, "r-clear", "CLEAR", r.generation(), r.lease_at(r.lease_count()-1).id, 3);
      ok = ok && push_frame(scratch, f.io, scratch.text);
      cycle_at(f, 223000);
      cycle_at(f, 224000);
      return ok && wire_contains(f, "\"r-clear\"") && r.state() == aa::ReceiverState::kEstop && !r.armed();
    }
    case 8: {  // 끝없는 수신에도 주기당 바이트 상한에서 반환(남은 바이트는 드라이버 버퍼)
      auto& a = f.make(true);
      f.io.endless_rx = true;
      bool ok = true;
      for (std::int64_t t = 0; t < 5000; t += 1000) {
        const auto r = cycle_at(f, t);
        ok = ok && r.rx_bytes == 1024 && r.rx_byte_budget_exhausted && !r.rx_time_budget_exhausted;
      }
      return ok && f.io.reads == 5 * 4 && a.stats().rx_byte_budget_cycles == 5 &&
             a.fault() == aa::UartBenchFault::kNone && !a.endpoint().receiver().armed();
    }
    case 9: {  // 읽기마다 시간이 많이 지나면 시간 상한에서 멈춘다(CPU 독점 방지)
      auto& a = f.make(true);
      f.io.endless_rx = true;
      f.clock.step = 400;
      const auto r = cycle_at(f, 0);
      const bool bounded = r.rx_time_budget_exhausted && r.rx_bytes < 1024 && r.rx_bytes > 0 && f.io.reads <= 3 &&
                           a.stats().rx_time_budget_cycles == 1 && a.fault() == aa::UartBenchFault::kNone;
      // 주기 앞부분이 이미 상한을 넘겨도 첫 읽기 하나(최대 256 B)는 허용해 수신이 영구히 굶지 않게 한다.
      auto& b = f.make(true);
      f.io.endless_rx = true;
      f.clock.step = 3000;
      const auto slow = cycle_at(f, 0);
      return bounded && slow.rx_bytes == 256 && slow.rx_time_budget_exhausted && f.io.reads == 1 &&
             b.fault() == aa::UartBenchFault::kNone;
    }
    case 10: {  // 끝없는 사건도 주기당 상한까지만 꺼내고, 포화는 유실 가능으로 보아 세션을 끊는다
      auto& a = f.make(true);
      cycle_at(f, 0);
      f.io.endless_events = true;
      const auto before = f.io.event_calls;
      const auto r = cycle_at(f, 1000);
      return r.events <= 2u * 8u && f.io.event_calls - before <= 2u * 8u && r.session_break &&
             a.stats().event_queue_saturated >= 1 && f.io.discards == 1 &&
             a.endpoint().receiver().state() == aa::ReceiverState::kEstop && a.fault() == aa::UartBenchFault::kNone;
    }
    case 11: {  // 넘침 뒤에는 앞 조각과 뒤 조각을 이어 붙이지 않는다. 이후 새 프레임은 정상 처리
      auto& a = f.make(true);
      cycle_at(f, 220000);
      const char* stop = "{\"type\":\"command\",\"request\":\"r-split\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}";
      bool ok = push_frame(scratch, f.io, stop, 20);
      cycle_at(f, 221000);
      ok = ok && a.endpoint().ingress().link().buffered_bytes() == 20;
      f.io.push_event(aa::UartIoEvent::kRxFifoOverflow);
      const auto broken = cycle_at(f, 222000);
      ok = ok && broken.session_break && f.io.discards == 1 && a.endpoint().ingress().link().buffered_bytes() == 0;
      ok = ok && push_rest(scratch, f.io, stop, 20);
      cycle_at(f, 223000);
      cycle_at(f, 224000);
      ok = ok && !wire_contains(f, "\"r-split\"") && a.stats().session_breaks == 1;
      ok = ok && push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-after\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      cycle_at(f, 225000);
      cycle_at(f, 226000);
      return ok && wire_contains(f, "\"r-after\"") && a.endpoint().receiver().state() == aa::ReceiverState::kEstop &&
             a.fault() == aa::UartBenchFault::kNone;
    }
    case 12: {  // 읽은 직후 프레임 오류가 보이면 그 조각은 순서를 알 수 없으므로 연결부에 넣지 않는다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-err\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      f.io.inject_after_read = static_cast<long>(f.io.reads) + 1;
      f.io.inject_event = aa::UartIoEvent::kFrameError;
      const auto r = cycle_at(f, 221000);
      cycle_at(f, 222000);
      return ok && r.session_break && r.rx_bytes == 0 && r.rx_discarded > 0 && !wire_contains(f, "\"r-err\"") &&
             a.stats().rx_discarded == r.rx_discarded && a.stats().error_events == 1 && f.io.discards == 1;
    }
    case 13: {  // 0 B 수락은 다음 주기로, 부분 수락은 이어 쓰기. 결과 wire의 모든 프레임은 CRC 정상
      auto& a = f.make(true);
      f.io.write_limit = 0;
      bool ok = true;
      for (std::int64_t t = 0; t < 3000; t += 1000) {
        const auto r = cycle_at(f, t);
        ok = ok && r.tx_bytes == 0 && r.tx_backpressure;
      }
      ok = ok && a.stats().tx_zero_writes >= 3 && f.io.wire_used == 0;
      f.io.write_limit = 7;
      for (std::int64_t t = 3000; t < 400000; t += 1000) cycle_at(f, t);
      ok = ok && a.stats().tx_partial_writes > 0 && wire_contains(f, "\"output_ready\":false") && f.wire_frames >= 2;
      return ok && f.decoder.errors() == 0 && a.fault() == aa::UartBenchFault::kNone;
    }
    case 14: {  // 송신 오류는 I/O 고장: 연결부 단절(로컬 STOP)·이후 Io 호출 없음
      auto& a = f.make(true);
      f.io.write_fail = true;
      const auto r = cycle_at(f, 0);
      const auto calls = f.io.calls();
      cycle_at(f, 5000);
      cycle_at(f, 10000);
      return r.fault == aa::UartBenchFault::kIoError && f.io.calls() == calls && a.stats().control_ticks == 1 &&
             a.endpoint().ingress().link().state() == aa::LinkState::kDisconnected &&
             a.endpoint().receiver().state() == aa::ReceiverState::kEstop;
    }
    case 15: {  // 드라이버가 요청보다 많이 받았다고 보고하면 고장(몇 바이트가 나갔는지 알 수 없음)
      auto& a = f.make(true);
      f.io.write_over = true;
      const auto r = cycle_at(f, 0);
      return r.fault == aa::UartBenchFault::kIoError && a.endpoint().tx().fault() == aa::TxFault::kNone &&
             a.endpoint().receiver().state() == aa::ReceiverState::kEstop;
    }
    case 16: {  // 수신 오류/과대 보고도 고장 뒤 침묵
      f.make(true);
      f.io.read_fail = true;
      const auto r = cycle_at(f, 0);
      const auto calls = f.io.calls();
      cycle_at(f, 5000);
      const bool first = r.fault == aa::UartBenchFault::kIoError && f.io.calls() == calls &&
                         f.adapter->endpoint().receiver().state() == aa::ReceiverState::kEstop;
      auto& b = f.make(true);
      f.io.read_over = true;
      const auto o = cycle_at(f, 0);
      return first && o.fault == aa::UartBenchFault::kIoError && o.rx_bytes == 0 && b.stats().rx_bytes == 0;
    }
    case 17: {  // 시각 역행: 어댑터 고정·수신기도 시계 고장, 이후 Io/tick 없음
      auto& a = f.make(true);
      cycle_at(f, 10000);
      f.clock.regress_on_call = f.clock.calls + 2;  // 다음 주기의 첫 수신 시각
      f.io.push_rx(reinterpret_cast<const std::uint8_t*>("IA"), 2);
      const auto r = cycle_at(f, 15000);
      const auto calls = f.io.calls();
      const auto ticks = a.stats().control_ticks;
      cycle_at(f, 20000);
      return r.fault == aa::UartBenchFault::kClockRegressed && f.io.calls() == calls &&
             a.stats().control_ticks == ticks && a.endpoint().receiver().state() == aa::ReceiverState::kFault;
    }
    case 18: {  // 송신 도중 세션이 끊기면 꼬리는 버리고 다음 프레임은 처음부터. 호스트는 CRC로 잘린 조각을 버린다
      auto& a = f.make(true);
      f.io.write_limit = 10;
      cycle_at(f, 0);
      bool ok = a.endpoint().tx().frame_in_flight() && f.io.wire_used == 10;
      f.io.push_event(aa::UartIoEvent::kBreak);
      f.io.write_limit = static_cast<std::size_t>(-1);
      const auto r = cycle_at(f, 1000);
      ok = ok && r.session_break && a.endpoint().tx().stats().aborted_frames == 1;
      for (std::int64_t t = 2000; t < 60000; t += 1000) cycle_at(f, t);
      return ok && wire_contains(f, "\"state\":\"ESTOP_LATCHED\"") && f.wire_frames >= 1 &&
             f.decoder.errors() >= 1 && a.fault() == aa::UartBenchFault::kNone;
    }
    case 19: {  // 무수신을 USB 단절로 추정하지 않는다(세션/STOP을 만들지 않음). 만료/독립 watchdog의 몫
      auto& a = f.make(true);
      for (std::int64_t t = 0; t <= 2000000; t += 5000) cycle_at(f, t);
      return a.stats().session_breaks == 0 && a.endpoint().ingress().link().session_breaks() == 0 &&
             a.endpoint().receiver().state() == aa::ReceiverState::kDisarmed && a.stats().control_ticks == 401;
    }
    case 20: {  // 드라이버 입력 폐기 실패는 I/O 고장
      auto& a = f.make(true);
      cycle_at(f, 0);
      f.io.discard_fail = true;
      f.io.push_event(aa::UartIoEvent::kRxBufferFull);
      const auto r = cycle_at(f, 1000);
      const auto calls = f.io.calls();
      cycle_at(f, 2000);
      return r.fault == aa::UartBenchFault::kIoError && f.io.calls() == calls &&
             a.endpoint().receiver().state() == aa::ReceiverState::kEstop;
    }
    case 21: {  // 매 주기 poll로 조립 기한이 지난 부분 프레임을 버린다(늦은 꼬리와 결합 안 함)
      auto& a = f.make(true);
      cycle_at(f, 220000);
      const char* stop = "{\"type\":\"command\",\"request\":\"r-late\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}";
      bool ok = push_frame(scratch, f.io, stop, 12);
      cycle_at(f, 221000);
      ok = ok && a.endpoint().ingress().link().buffered_bytes() == 12;
      cycle_at(f, 251000);  // 조립 기한 30 ms(시험값) 도달
      ok = ok && a.endpoint().ingress().link().buffered_bytes() == 0 &&
           a.endpoint().ingress().link().assembly_timeouts() == 1;
      ok = ok && push_rest(scratch, f.io, stop, 12);
      cycle_at(f, 252000);
      cycle_at(f, 253000);
      return ok && !wire_contains(f, "\"r-late\"") && a.endpoint().receiver().state() == aa::ReceiverState::kDisarmed;
    }
    case 22: {  // 연결부에 주는 시각은 바이트가 고정 버퍼에 들어온 뒤 읽은 값. 주기 시작 시각으로 당기지 않는다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-t\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}", 9);
      f.clock.step = 100;
      const auto r = cycle_at(f, 221000);
      const auto stamp = a.stats().last_rx_buffered_us;
      return ok && r.rx_bytes == 9 && stamp > r.start_us && stamp <= r.end_us &&
             a.endpoint().ingress().link().pending_since() == static_cast<std::uint64_t>(stamp);
    }
  }
  return false;
}

inline Report run_uart(Scratch& scratch, UartScratch& f, std::uint64_t* names_mask = nullptr) {
  Report report;
  for (unsigned i = 0; i < kUartCases; ++i) report.record(uart_case(i, scratch, f));
  f.destroy();
  if (names_mask) *names_mask = report.failure_mask;
  return report;
}

}  // namespace arena_smoke
