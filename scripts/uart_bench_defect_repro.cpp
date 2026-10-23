// 10/23 최종 검토(두 차례)가 찾은 UART 주기 어댑터 계약 불일치의 재현기. 가짜 I/O만 쓰며 실제 UART 실행이 아니다.
// 이전 판본(actuation_uart_bench_v1·review_v2 스냅샷)과 수정본 헤더 모두로 컴파일되도록 두 판본 공통 API만 사용한다.
// 수정본에서는 모두 통과해야 하고, 이전 판본에서는 결함 때문에 실패해야 한다(실패 이름을 출력).
#include "uart_bench_fakes.hpp"
#include <inttypes.h>

namespace {
namespace aa = arena_actuation;
using arena_smoke::cycle_at;
using arena_smoke::push_frame;
using arena_smoke::wire_contains;

const char* const kNames[] = {
  "static_full_queue_missed", "event_budget_per_call", "error_tail_after_budget_fed",
  "unconfirmed_bytes_fed", "cycle_end_excludes_last_action", "end_clock_regress_unseen",
  "observed_pending_after_empty_fed",
};

bool repro(unsigned index, arena_smoke::Scratch& scratch, arena_smoke::UartScratch& f) {
  switch (index) {
    case 0: {  // 진입 때 가득 찬 DATA 큐: 꺼낸 뒤 점유만 보면 포화를 놓친다
      f.make(true);
      cycle_at(f, 220000);
      for (int i = 0; i < 16; ++i) f.io.push_event(aa::UartIoEvent::kData);
      push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-full\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      const auto r = cycle_at(f, 221000);
      cycle_at(f, 222000);
      cycle_at(f, 223000);
      return r.session_break && f.adapter->stats().event_queue_saturated >= 1 && !wire_contains(f, "\"r-full\"");
    }
    case 1: {  // 사건 예산 8이 확인 호출마다 초기화되면 한 주기에 8개를 넘게 처리한다
      f.make(true);
      f.io.endless_rx = true;
      f.io.data_events_per_read = 3;
      const auto r = cycle_at(f, 0);
      return r.events <= 8;
    }
    case 2: {  // 예산 뒤에 숨은 오류 사건을 확인하기 전에 바이트를 연결부로 넣으면 안 된다
      auto cfg = arena_smoke::uart_config();
      cfg.event_queue_capacity = 32;
      f.make(true, cfg);
      cycle_at(f, 220000);
      for (int i = 0; i < 16; ++i) f.io.push_event(aa::UartIoEvent::kData);
      f.io.push_event(aa::UartIoEvent::kFrameError);
      push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-tail\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      for (std::int64_t t = 221000; t <= 225000; t += 1000) cycle_at(f, t);
      return !wire_contains(f, "\"r-tail\"");
    }
    case 3: {  // 읽은 뒤 예산 안에서 사건을 다 확인하지 못한 조각을 넣으면 안 된다
      f.make(true);
      cycle_at(f, 220000);
      push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-unconf\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      f.io.data_events_per_read = 10;
      cycle_at(f, 221000);
      f.io.data_events_per_read = 0;
      cycle_at(f, 222000);
      cycle_at(f, 223000);
      return !wire_contains(f, "\"r-unconf\"");
    }
    case 4: {  // 송신에 걸린 시간이 주기 실행시간에서 빠지면 안 된다
      f.make(true);
      f.io.write_cost_us = 3000;
      const auto r = cycle_at(f, 0);
      return r.end_us - r.start_us >= 3000 && f.adapter->stats().max_cycle_us >= 3000;
    }
    case 5: {  // 마지막 작업 뒤 시각 역행을 이번 주기에서 보지 못하면 안 된다
      f.make(true);
      cycle_at(f, 10000);
      f.io.regress_after_write = true;
      const auto r = cycle_at(f, 30000);
      return f.io.writes > 0 && r.fault == aa::UartBenchFault::kClockRegressed;
    }
    case 6: {  // 읽은 뒤 '비어 있음' 직후 도착해 마지막 조회에 보인 사건을 무시하고 조각을 넣으면 안 된다
      f.make(true);
      cycle_at(f, 220000);
      push_frame(scratch, f.io, "{\"type\":\"command\",\"request\":\"r-race\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}");
      f.io.enqueue_after_empty_on_next_read = true;
      for (std::int64_t t = 221000; t <= 223000; t += 1000) cycle_at(f, t);
      return !wire_contains(f, "\"r-race\"");
    }
  }
  return false;
}
}  // namespace

int main() {
  static arena_smoke::Scratch scratch;
  static arena_smoke::UartScratch uart;
  unsigned failures = 0;
  std::printf("{\"repro_checks\":7,\"failed\":[");
  for (unsigned i = 0; i < 7; ++i) {
    if (repro(i, scratch, uart)) continue;
    std::printf("%s\"%s\"", failures ? "," : "", kNames[i]);
    ++failures;
  }
  uart.destroy();
  std::printf("],\"repro_failures\":%u,\"hardware_executed\":false}\n", failures);
  return failures == 0 ? 0 : 1;
}
