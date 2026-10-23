// 출력 비활성 UART 주기 어댑터 공유 사례의 호스트 실행. 가짜 I/O이며 실제 UART/대상 MCU 실행이 아니다.
#include "uart_bench_cases.hpp"
#include <inttypes.h>

int main() {
  static arena_smoke::Scratch scratch;
  static arena_smoke::UartScratch uart;
  const auto report = arena_smoke::run_uart(scratch, uart);
  std::printf("{\"uart_checks\":%u,\"uart_failures\":%u,\"uart_failure_mask\":\"%016" PRIx64 "\",\"failed\":[",
              report.checks, report.failures, report.failure_mask);
  bool first = true;
  for (unsigned i = 0; i < report.checks; ++i) {
    if (!(report.failure_mask & (UINT64_C(1) << i))) continue;
    std::printf("%s\"%s\"", first ? "" : ",", arena_smoke::uart_case_name(i));
    first = false;
  }
  std::printf("],\"hardware_executed\":false,\"output_ready\":false}\n");
  return report.checks == arena_smoke::kUartCases && report.failures == 0 ? 0 : 1;
}
