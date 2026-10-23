#pragma once
// 출력 비활성 UART 주기 어댑터의 공유 반례. 호스트와 ESP-IDF에서 같은 가짜 I/O·합성 시각만 쓴다.
// 실제 UART/GPIO를 열지 않으며 결과를 보드 실행이나 실물 배선 검증으로 해석하지 않는다.
#include "uart_bench_fakes.hpp"

namespace arena_smoke {

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

constexpr unsigned kUartCases = 33;
inline const char* uart_case_name(unsigned i) {
  static const char* const names[kUartCases] = {
    "plan_default_disabled", "plan_rejections", "disabled_io_untouched", "invalid_config_silent",
    "tick_independent_of_rx", "late_tick_no_catch_up", "arm_and_drive_rejected", "stop_then_clear_rejected",
    "rx_byte_budget", "rx_time_budget", "event_flood_bounded", "overflow_discards_partial",
    "frame_error_after_read", "tx_zero_then_partial", "tx_error_latches", "tx_overreport_latches",
    "read_error_latches", "clock_regress_silent", "session_break_aborts_tail", "silence_is_not_detach",
    "discard_failure_latches", "stale_partial_expires", "rx_stamp_after_buffer",
    "static_full_data_queue", "late_error_after_partial", "event_budget_per_cycle", "error_tail_after_budget",
    "unconfirmed_after_read_discarded", "cycle_end_after_last_action", "cycle_end_clock_regress",
    "refill_during_drain_detected", "transient_saturation_not_seen", "pending_after_empty_post_read",
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
      const bool bounded = r.events <= 8u && f.io.event_calls - before <= 8u && r.session_break &&
                           r.break_deferred && a.stats().event_queue_saturated >= 1 && f.io.discards == 1 &&
                           a.endpoint().receiver().state() == aa::ReceiverState::kEstop &&
                           a.fault() == aa::UartBenchFault::kNone;
      // 같은 주기의 두 번째 경계는 잊지 않고 다음 주기 시작에서 먼저 수행한다.
      f.io.endless_events = false;
      const auto next = cycle_at(f, 2000);
      return bounded && next.session_break && a.stats().session_breaks == 2 && a.stats().deferred_breaks == 1;
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
    case 23: {  // 진입 때 이미 가득 찬 DATA 큐(정상 사건뿐)도 유실 가능으로 본다. 8개만 꺼내 8로 내려간 뒤 보면 놓친다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      for (int i = 0; i < 16; ++i) f.io.push_event(aa::UartIoEvent::kData);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-full\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      const auto r = cycle_at(f, 221000);
      cycle_at(f, 222000);
      cycle_at(f, 223000);
      return ok && r.session_break && r.events == 0 && a.stats().event_queue_saturated >= 1 &&
             !wire_contains(f, "\"r-full\"") && a.fault() == aa::UartBenchFault::kNone;
    }
    case 24: {  // 이미 전달한 완전 프레임은 되돌릴 수 없고, 늦은 오류는 남은 부분 프레임만 버린다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-done\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      cycle_at(f, 221000);
      cycle_at(f, 222000);
      ok = ok && wire_contains(f, "\"r-done\"");
      const char* late = "{\"type\":\"command\",\"request\":\"r-late2\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}";
      ok = ok && push_frame(scratch, f.io, late, 20);
      cycle_at(f, 223000);
      ok = ok && a.endpoint().ingress().link().buffered_bytes() == 20;
      f.io.push_event(aa::UartIoEvent::kFrameError);
      ok = ok && push_rest(scratch, f.io, late, 20);
      const auto r = cycle_at(f, 224000);
      cycle_at(f, 225000);
      cycle_at(f, 226000);
      return ok && r.session_break && a.endpoint().ingress().link().buffered_bytes() == 0 &&
             wire_contains(f, "\"r-done\"") && !wire_contains(f, "\"r-late2\"") && a.stats().session_breaks == 1;
    }
    case 25: {  // 사건 예산은 주기 전체 상한. 읽기마다 DATA가 생겨도 8개를 넘지 않고, 예약분이 모자라면 읽기를 멈춘다
      auto& a = f.make(true);
      f.io.endless_rx = true;
      f.io.data_events_per_read = 3;
      const auto r = cycle_at(f, 0);
      return r.events == 6 && r.rx_bytes == 512 && r.rx_event_budget_exhausted && !r.session_break &&
             r.rx_discarded == 0 && f.io.queued == 0 && a.stats().rx_event_budget_cycles == 1 &&
             a.fault() == aa::UartBenchFault::kNone;
    }
    case 26: {  // 예산을 다 써도 남은 사건에 오류가 숨어 있을 수 있다. 확인 전에는 바이트를 꺼내지 않는다
      auto cfg = uart_config();
      cfg.event_queue_capacity = 32;
      auto& a = f.make(true, cfg);
      cycle_at(f, 220000);
      for (int i = 0; i < 16; ++i) f.io.push_event(aa::UartIoEvent::kData);
      f.io.push_event(aa::UartIoEvent::kFrameError);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-tail\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      const auto r1 = cycle_at(f, 221000);
      const auto r2 = cycle_at(f, 222000);
      const auto r3 = cycle_at(f, 223000);
      cycle_at(f, 224000);
      cycle_at(f, 225000);
      return ok && r1.events == 8 && r1.rx_bytes == 0 && r1.rx_discarded == 0 && !r1.session_break &&
             r1.rx_event_budget_exhausted && r2.events == 8 && r2.rx_bytes == 0 && !r2.session_break &&
             r3.session_break && a.stats().error_events == 1 && !wire_contains(f, "\"r-tail\"");
    }
    case 27: {  // 읽은 뒤 예산 안에서 사건을 다 확인하지 못하면 그 조각은 넣지 않고 세션을 끊는다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-unconf\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      f.io.data_events_per_read = 10;
      const auto r = cycle_at(f, 221000);
      f.io.data_events_per_read = 0;
      cycle_at(f, 222000);
      cycle_at(f, 223000);
      return ok && r.rx_bytes == 0 && r.rx_discarded > 0 && r.session_break && r.rx_event_budget_exhausted &&
             a.stats().rx_unconfirmed_discarded == r.rx_discarded && r.events == 8 && !wire_contains(f, "\"r-unconf\"");
    }
    case 28: {  // 주기 실행시간은 송신·입력 폐기 뒤 다시 읽은 종료 시각까지 포함한다. 수신 버퍼 시각과 구별
      auto& a = f.make(true);
      f.io.write_cost_us = 3000;
      const auto r = cycle_at(f, 0);
      f.io.write_cost_us = 0;
      bool ok = r.end_us - r.start_us >= 3000 && a.stats().max_cycle_us >= 3000 && a.stats().cycles_over_budget == 1 &&
                a.stats().last_rx_buffered_us == -1;
      f.io.discard_cost_us = 2500;
      f.io.push_event(aa::UartIoEvent::kRxFifoOverflow);
      const auto d = cycle_at(f, 20000);
      return ok && d.session_break && d.end_us - d.start_us >= 2500 && a.fault() == aa::UartBenchFault::kNone;
    }
    case 29: {  // 마지막 작업 뒤 종료 시각이 역행해도 이번 주기에서 시계 고장으로 고정하고 이후 침묵
      auto& a = f.make(true);
      cycle_at(f, 10000);
      f.io.regress_after_write = true;
      const auto r = cycle_at(f, 30000);
      const auto calls = f.io.calls();
      cycle_at(f, 40000);
      return f.io.writes > 0 && r.fault == aa::UartBenchFault::kClockRegressed && f.io.calls() == calls &&
             a.endpoint().receiver().state() == aa::ReceiverState::kFault;
    }
    case 30: {  // 꺼내는 동안 ISR이 다시 가득 채우고 그대로면 종료 검사가 본다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      for (int i = 0; i < 10; ++i) f.io.push_event(aa::UartIoEvent::kData);
      f.io.refill_on_event_call = static_cast<long>(f.io.event_calls) + 8;
      f.io.refill_to = 16;
      const auto r = cycle_at(f, 221000);
      return r.session_break && r.events == 8 && a.stats().event_queue_saturated == 1;
    }
    case 31: {  // 한계 고정: 가득 찼다가 우리가 다시 꺼내 내려간 순간 포화는 볼 수 없다(모든 유실 검출이 아님)
      auto& a = f.make(true);
      cycle_at(f, 220000);
      for (int i = 0; i < 10; ++i) f.io.push_event(aa::UartIoEvent::kData);
      f.io.refill_on_event_call = static_cast<long>(f.io.event_calls) + 1;
      f.io.refill_to = 16;
      const auto r = cycle_at(f, 221000);
      return !r.session_break && a.stats().event_queue_saturated == 0 && r.rx_event_budget_exhausted &&
             r.rx_bytes == 0 && r.events == 8 && f.io.queued == 9;
    }
    case 32: {  // 읽은 뒤 '큐 비어 있음' 직후 도착해 마지막 조회에 보인 사건은 내용 미확인이다. 조각을 넣지 않는다
      auto& a = f.make(true);
      cycle_at(f, 220000);
      bool ok = push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-race\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      f.io.enqueue_after_empty_on_next_read = true;
      const auto r = cycle_at(f, 221000);
      cycle_at(f, 222000);
      cycle_at(f, 223000);
      return ok && r.rx_bytes == 0 && r.rx_discarded > 0 && r.session_break && r.rx_event_budget_exhausted &&
             a.stats().rx_unconfirmed_discarded == r.rx_discarded && !wire_contains(f, "\"r-race\"");
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
