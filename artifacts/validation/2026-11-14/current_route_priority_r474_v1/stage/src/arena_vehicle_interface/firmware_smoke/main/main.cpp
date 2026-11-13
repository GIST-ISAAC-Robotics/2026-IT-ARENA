// 검사 전용 앱. UART 명령 수신·송신, GPIO/PWM/서보/PCNT/센서 구동은 구현하지 않는다(송신 정책은 배열 sink만 사용).
#include "smoke_cases.hpp"
#include "ingress_cases.hpp"
#include "tx_cases.hpp"
#include "bench_cases.hpp"
#include "pi_cases.hpp"
#include "motor_cycle_cases.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

namespace {
arena_smoke::Scratch scratch;
arena_smoke::IngressScratch ingress_scratch;
arena_smoke::TxScratch tx_scratch;
arena_smoke::BenchScratch bench_scratch;
}

extern "C" void app_main() {
  const std::int64_t begin = esp_timer_get_time();
  const auto result = arena_smoke::run(scratch);
  const auto ingress = arena_smoke::run_ingress(scratch, ingress_scratch);
  const auto tx = arena_smoke::run_tx(scratch, tx_scratch);
  const auto bench = arena_smoke::run_bench(scratch, bench_scratch);
  const auto pi = arena_smoke::run_pi();
  const auto cycle = arena_smoke::run_motor_cycle(scratch);
  const std::int64_t elapsed = esp_timer_get_time() - begin;
  std::printf("{\"kind\":\"actuation_smoke\",\"synthetic_inputs\":true,\"checks\":%u,\"failures\":%u,"
              "\"failure_mask\":\"%016" PRIx64 "\",\"ingress_checks\":%u,\"ingress_failures\":%u,"
              "\"ingress_failure_mask\":\"%016" PRIx64 "\",\"tx_checks\":%u,\"tx_failures\":%u,"
              "\"tx_failure_mask\":\"%016" PRIx64 "\",\"elapsed_us\":%" PRId64 ","
              "\"bench_checks\":%u,\"bench_failures\":%u,\"bench_failure_mask\":\"%016" PRIx64 "\","
              "\"pi_checks\":%u,\"pi_failures\":%u,\"pi_failure_mask\":\"%016" PRIx64 "\","
              "\"cycle_checks\":%u,\"cycle_failures\":%u,\"cycle_failure_mask\":\"%016" PRIx64 "\","
              "\"stack_free_bytes\":%u,\"firmware_verified\":false,\"ready_to_drive\":false}\n",
              result.checks, result.failures, result.failure_mask,
              ingress.checks, ingress.failures, ingress.failure_mask, tx.checks, tx.failures, tx.failure_mask, elapsed,
              bench.checks, bench.failures, bench.failure_mask,
              pi.checks, pi.failures, pi.failure_mask,
              cycle.checks, cycle.failures, cycle.failure_mask,
              static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
  // app_main 반환 뒤 어떤 출력 작업도 시작하지 않는다. 이 JSON이 성공해도 실물 구동 검증은 아니다.
}
