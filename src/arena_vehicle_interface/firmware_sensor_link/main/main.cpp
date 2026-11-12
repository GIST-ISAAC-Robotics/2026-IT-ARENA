// 실제 IMU + 출력 비활성 구동 계약. PWM/DIR/서보/엔코더 피드백 생성 코드가 없다.
#include "sensor_link_tx.hpp"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern "C" void arena_imu_init(std::uint32_t boot);
extern "C" void arena_imu_step(void);
extern "C" void arena_imu_stop(void);

namespace aa = arena_actuation;
static aa::SensorLinkTx sensor_tx;
extern "C" bool arena_imu_emit(const void* data, std::size_t size) {
  return sensor_tx.push(data, size);
}

extern "C" void app_main() {
  uart_config_t uart{};
  uart.baud_rate = 921600;
  uart.data_bits = UART_DATA_8_BITS;
  uart.parity = UART_PARITY_DISABLE;
  uart.stop_bits = UART_STOP_BITS_1;
  uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart.source_clk = UART_SCLK_DEFAULT;
  QueueHandle_t events = nullptr;
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 0, 32, &events, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  const std::uint32_t boot = esp_random();
  arena_imu_init(boot);  // IMU 초기화의 100 ms 지연 뒤 명령 계약 시계를 시작한다.
  std::uint32_t identity[8];
  const char hex[] = "0123456789abcdef";
  for (int i = 0; i < 8; ++i) identity[i] = hex[(boot >> ((7-i)*4)) & 15];
  aa::ReceiverConfig config;
  config.feedback_mode = aa::FeedbackMode::kMotorShaft;
  config.ticks_per_rev = 28;
  config.gear_ratio = 2.5 * 81. / 17.;
  config.wheel_radius_m = .0325;
  aa::TxConfig tx;
  tx.offer_status_v1 = true;  // 관측용 실제 상태/시각. output_ready는 false 유지.
  tx.offer_period_us = 20000;
  tx.diag_period_us = 1000000;
  tx.max_consecutive_events = 2;
  tx.line_ns_per_byte = 10851;  // 8N1/921600의 올림값. 측정치가 아니다.
  static aa::ActuationBenchEndpoint endpoint(config, identity, 8, 30000, aa::PayloadLayout::kEnvelope, tx);
  endpoint.connect(esp_timer_get_time());
  auto control_due = esp_timer_get_time();
  bool failed = false;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    auto now = esp_timer_get_time();
    if (now >= control_due) {
      endpoint.control_tick(now); control_due = now + 5000;
    }
    uart_event_t event{};
    for (int i = 0; i < 32 && xQueueReceive(events, &event, 0) == pdTRUE; ++i) {
      if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL ||
          event.type == UART_BREAK || event.type == UART_PARITY_ERR || event.type == UART_FRAME_ERR) failed = true;
    }
    if (uxQueueMessagesWaiting(events) >= 32) failed = true;
    if (!failed) {
      std::uint8_t rx[256];
      const int count = uart_read_bytes(UART_NUM_0, rx, sizeof(rx), 0);
      if (count < 0) failed = true;
      else if (count) endpoint.feed(esp_timer_get_time(), rx, count);
      endpoint.poll(esp_timer_get_time());
      arena_imu_step();  // FIFO 최대8항목. 원래 tag/센서 tick/read 시각·68 B CRC 보존.
      const auto sink = [&](const std::uint8_t* data, std::size_t size) -> std::size_t {
        const int count = uart_tx_chars(UART_NUM_0, reinterpret_cast<const char*>(data), size);
        if (count < 0) { failed = true; return size + 1; }
        return count;
      };
      if (!sensor_tx.pump(esp_timer_get_time(), endpoint, sink)) failed = true;
    }
    if (failed) {
      endpoint.disconnect(esp_timer_get_time());
      arena_imu_stop();
      uart_flush_input(UART_NUM_0);
      // UART 오류 후 자동 재개하지 않는다. 호스트는 수신 중단을 보고 보드를 재시작한다.
      vTaskSuspend(nullptr);
    }
    if (xTaskDelayUntil(&last, 1) == pdFALSE) {
      last = xTaskGetTickCount(); vTaskDelay(1);
    }
  }
}
