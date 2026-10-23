// ESP-IDF와 같은 사례의 호스트 검증. 실제 대상 MCU 실행으로 해석하지 않는다.
#include "smoke_cases.hpp"
#include "ingress_cases.hpp"
#include "tx_cases.hpp"
#include "bench_cases.hpp"
#include "pi_cases.hpp"
#include "motor_cycle_cases.hpp"
#include <inttypes.h>

int main() {
  static arena_smoke::Scratch scratch;
  static arena_smoke::IngressScratch ingress_scratch;
  static arena_smoke::TxScratch tx_scratch;
  static arena_smoke::BenchScratch bench_scratch;
  const auto result = arena_smoke::run(scratch);
  const auto ingress = arena_smoke::run_ingress(scratch, ingress_scratch);
  const auto tx = arena_smoke::run_tx(scratch, tx_scratch);
  const auto bench = arena_smoke::run_bench(scratch, bench_scratch);
  const auto pi = arena_smoke::run_pi();
  const auto cycle = arena_smoke::run_motor_cycle(scratch);
  std::printf("{\"checks\":%u,\"failures\":%u,\"failure_mask\":\"%016" PRIx64 "\","
              "\"ingress_checks\":%u,\"ingress_failures\":%u,\"ingress_failure_mask\":\"%016" PRIx64 "\","
              "\"tx_checks\":%u,\"tx_failures\":%u,\"tx_failure_mask\":\"%016" PRIx64 "\","
              "\"bench_checks\":%u,\"bench_failures\":%u,\"bench_failure_mask\":\"%016" PRIx64 "\","
              "\"pi_checks\":%u,\"pi_failures\":%u,\"pi_failure_mask\":\"%016" PRIx64 "\","
              "\"cycle_checks\":%u,\"cycle_failures\":%u,\"cycle_failure_mask\":\"%016" PRIx64 "\","
              "\"hardware_executed\":false}\n",
              result.checks, result.failures, result.failure_mask, ingress.checks, ingress.failures, ingress.failure_mask,
              tx.checks, tx.failures, tx.failure_mask, bench.checks, bench.failures, bench.failure_mask,
              pi.checks, pi.failures, pi.failure_mask, cycle.checks, cycle.failures, cycle.failure_mask);
  return result.failures == 0 && result.checks == 20 && ingress.checks == 12 && ingress.failures == 0 &&
         tx.checks == 9 && tx.failures == 0 && bench.checks == 15 && bench.failures == 0 &&
         pi.checks == 12 && pi.failures == 0 && cycle.checks == 25 && cycle.failures == 0 ? 0 : 1;
}
