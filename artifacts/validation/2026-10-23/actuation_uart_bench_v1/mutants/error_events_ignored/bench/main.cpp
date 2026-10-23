// 출력 비활성 UART bring-up 검사 앱. 모터/서보/PWM/DIR/PCNT/IMU 구동과 업로드 스크립트는 없다.
// 기본 계획은 외부 I/O 비활성: 공유 가짜 I/O 사례만 실행하고 UART/GPIO를 초기화하지 않는다.
#include "uart_bench_cases.hpp"
#include "esp_uart_io.hpp"
#include "esp_random.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>

namespace aa = arena_actuation;
using Adapter = aa::UartBenchAdapter<arena_bench::EspUartIo, arena_bench::EspClock>;

// 실물 판본·배선·콘솔·부팅 출력 확인 전에는 비활성으로 둔다. 외부 연결 이름이라 컴파일러가 아래 경로를
// 지우지 않으며(대상 링크 확인용), 이 값을 바꾸는 것은 별도 확인을 거친 변경이어야 한다.
aa::UartPortPlan arena_bench_protocol_plan{};

namespace {
arena_smoke::Scratch scratch;
arena_smoke::UartScratch uart_scratch;
alignas(aa::ActuationBenchEndpoint) unsigned char endpoint_storage[sizeof(aa::ActuationBenchEndpoint)];
alignas(Adapter) unsigned char adapter_storage[sizeof(Adapter)];
arena_bench::EspUartIo uart_io;
arena_bench::EspClock clock_source;
std::uint32_t boot_id[16];
Adapter* adapter = nullptr;

constexpr TickType_t kLoopTicks = 1;  // 소유 작업 주기(CONFIG_FREERTOS_HZ=1000에서 1 ms). 검사값

aa::UartBenchConfig bench_config() {
  aa::UartBenchConfig c;  // 검사값. receiver max_control_gap_us(50 ms)보다 작은 제어 간격
  c.control_period_us = 5000;
  c.cycle_budget_us = 500;
  c.max_rx_bytes_per_cycle = 1024;
  c.max_events_per_cycle = 16;
  c.event_queue_capacity = 32;
  return c;
}

aa::UartPortPlan with_console(aa::UartPortPlan plan) {
#if CONFIG_ESP_CONSOLE_UART
  plan.console_port = CONFIG_ESP_CONSOLE_UART_NUM;
#if defined(CONFIG_ESP_CONSOLE_UART_TX_GPIO) && defined(CONFIG_ESP_CONSOLE_UART_RX_GPIO)
  plan.console_tx_gpio = CONFIG_ESP_CONSOLE_UART_TX_GPIO;
  plan.console_rx_gpio = CONFIG_ESP_CONSOLE_UART_RX_GPIO;
#else
  plan.console_tx_gpio = 43;  // ESP32-S3 기본 U0TXD/U0RXD
  plan.console_rx_gpio = 44;
#endif
#else
  plan.console_port = -1;
#endif
  return plan;
}

// 부팅별 세션 식별자(보안/인증 아님). RF 비활성 상태의 esp_random 품질은 SDK 문서의 제한을 따른다.
void make_boot_id() {
  static const char hex[] = "0123456789abcdef";
  const std::uint32_t words[2] = {esp_random(), esp_random()};
  for (int i = 0; i < 16; ++i) boot_id[i] = static_cast<std::uint8_t>(hex[(words[i / 8] >> ((i % 8) * 4)) & 0xF]);
}

void owner_task(void*) {
  TickType_t last = xTaskGetTickCount();
  aa::UartCycleReport report;
  for (;;) {
    report = adapter->run_cycle();
    if (report.fault != aa::UartBenchFault::kNone) break;
    // 늦은 주기를 연속 실행으로 따라잡지 않는다. 기한을 지금으로 다시 잡고 한 tick 양보한다.
    if (xTaskDelayUntil(&last, kLoopTicks) == pdFALSE) {
      last = xTaskGetTickCount();
      vTaskDelay(1);
    }
  }
  // 고장 뒤에는 UART를 더 다루지 않는다. 진단 콘솔(프로토콜과 다른 포트)에 한 줄만 남긴다.
  const auto& s = adapter->stats();
  std::printf("{\"kind\":\"actuation_uart_bench_fault\",\"fault\":%u,\"cycles\":%" PRIu64 ",\"control_ticks\":%" PRIu64
              ",\"session_breaks\":%" PRIu64 ",\"firmware_verified\":false,\"ready_to_drive\":false}\n",
              static_cast<unsigned>(report.fault), s.cycles, s.control_ticks, s.session_breaks);
  vTaskDelete(nullptr);
}

bool start_owner(const aa::UartPortPlan& plan) {
  const auto config = bench_config();
  if (!uart_io.open(plan, config.event_queue_capacity, 4096)) return false;
  make_boot_id();
  aa::TxConfig tx;  // 검사값. line_ns_per_byte=0(모름)
  tx.offer_period_us = 20000;
  tx.max_consecutive_events = 2;
  auto* endpoint = new (endpoint_storage) aa::ActuationBenchEndpoint(
      arena_smoke::motor_config(), boot_id, 16, 30000, aa::PayloadLayout::kEnvelope, tx);
  adapter = new (adapter_storage) Adapter(*endpoint, config, uart_io, clock_source, true);
  return xTaskCreate(owner_task, "uart_bench", 6144, nullptr, 5, nullptr) == pdPASS;
}
}  // namespace

extern "C" void app_main() {
  const std::int64_t begin = esp_timer_get_time();
  const auto cases = arena_smoke::run_uart(scratch, uart_scratch);
  const std::int64_t elapsed = esp_timer_get_time() - begin;
  const auto plan = with_console(arena_bench_protocol_plan);
  const auto status = aa::check_uart_plan(plan);
  const bool started = status == aa::UartPlanStatus::kReady && start_owner(plan);
  std::printf("{\"kind\":\"actuation_uart_bench\",\"synthetic_inputs\":true,\"uart_checks\":%u,\"uart_failures\":%u,"
              "\"uart_failure_mask\":\"%016" PRIx64 "\",\"elapsed_us\":%" PRId64 ",\"protocol_plan\":\"%s\","
              "\"console_port\":%d,\"external_io_started\":%s,\"stack_free_bytes\":%u,"
              "\"output_ready\":false,\"firmware_verified\":false,\"ready_to_drive\":false}\n",
              cases.checks, cases.failures, cases.failure_mask, elapsed, aa::plan_status_name(status),
              plan.console_port, started ? "true" : "false",
              static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
}
