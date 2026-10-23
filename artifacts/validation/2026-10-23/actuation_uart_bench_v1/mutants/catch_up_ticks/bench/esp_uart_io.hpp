#pragma once
// ESP-IDF v6.1 UART 드라이버를 UartBenchAdapter의 Io 개념에 맞춘 얇은 연결. 대상 빌드 전용.
// check_uart_plan()이 kReady인 계획에서만 open()한다. 기본 계획은 비활성이므로 실행 경로에서 호출되지 않는다.
// 모든 함수는 어댑터 소유 작업에서만 호출한다. 같은 포트를 다른 작업/콘솔/VFS가 쓰지 않아야 한다.
#include "actuation_uart_bench.hpp"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace arena_bench {
namespace aa = arena_actuation;

struct EspClock {
  // 부팅 이후 단조 µs. 바이트의 선로 도착·센서 취득 시각이 아니라 호출 시점이다.
  std::int64_t now_us() { return esp_timer_get_time(); }
};

class EspUartIo {
 public:
  // rx_ring_bytes는 하드웨어 FIFO(128 B)보다 커야 한다(SDK 검사). TX ring은 0으로 두어
  // uart_tx_chars가 FIFO 빈 칸만 채우고 반환하게 한다(SDK 주석: TX buffer 비사용 시에만 사용).
  bool open(const aa::UartPortPlan& plan, std::size_t event_queue_capacity, int rx_ring_bytes) {
    if (open_ || aa::check_uart_plan(plan) != aa::UartPlanStatus::kReady) return false;
    port_ = static_cast<uart_port_t>(plan.port);
    if (uart_driver_install(port_, rx_ring_bytes, 0, static_cast<int>(event_queue_capacity), &queue_, 0) != ESP_OK)
      return false;
    uart_config_t config{};
    config.baud_rate = static_cast<int>(plan.baud);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    if (uart_param_config(port_, &config) != ESP_OK ||
        uart_set_pin(port_, plan.tx_gpio, plan.rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
        queue_ == nullptr) {
      uart_driver_delete(port_);
      queue_ = nullptr;
      return false;
    }
    open_ = true;
    return true;
  }

  // ticks_to_wait=0: RX ring에 이미 있는 바이트만 복사한다. rx 잠금 경합도 -1이다(단일 소유 계약 위반).
  int read(std::uint8_t* dst, std::size_t max) {
    if (!open_) return -1;
    return uart_read_bytes(port_, dst, static_cast<std::uint32_t>(max), 0);
  }
  // TX FIFO 빈 칸만큼만 넣고 반환한다(0..n). 선로 송출 완료가 아니다. 내부 tx 잠금은 portMAX_DELAY이므로
  // 같은 포트를 다른 작업이 쓰면 비차단이 깨진다. 이 경로에서는 uart_write_bytes를 쓰지 않는다.
  int write(const std::uint8_t* src, std::size_t size) {
    if (!open_) return -1;
    return uart_tx_chars(port_, reinterpret_cast<const char*>(src), static_cast<std::uint32_t>(size));
  }
  bool next_event(aa::UartIoEvent& out) {
    if (!open_) return false;
    uart_event_t event{};
    if (xQueueReceive(queue_, &event, 0) != pdTRUE) return false;
    switch (event.type) {
      case UART_DATA: out = aa::UartIoEvent::kData; break;
      case UART_FIFO_OVF: out = aa::UartIoEvent::kRxFifoOverflow; break;
      case UART_BUFFER_FULL: out = aa::UartIoEvent::kRxBufferFull; break;
      case UART_FRAME_ERR: out = aa::UartIoEvent::kFrameError; break;
      case UART_PARITY_ERR: out = aa::UartIoEvent::kParityError; break;
      case UART_BREAK: out = aa::UartIoEvent::kBreak; break;
      default: out = aa::UartIoEvent::kOther; break;  // 사용하지 않는 사건도 보수적으로 오류로 본다
    }
    return true;
  }
  std::size_t pending_events() { return open_ ? uxQueueMessagesWaiting(queue_) : 0; }
  // 사건 큐를 먼저 비운 뒤 RX ring을 비운다. 그 사이 새 사건은 다음 주기의 추가 단절로 처리된다.
  // 반대 순서면 비운 뒤 생긴 넘침 사건을 지울 수 있다. uart_flush_input의 rx 잠금 대기는 단일 소유로 경합 없음.
  bool discard_input() {
    if (!open_) return false;
    xQueueReset(queue_);
    return uart_flush_input(port_) == ESP_OK;
  }

 private:
  bool open_ = false;
  uart_port_t port_ = UART_NUM_MAX;
  QueueHandle_t queue_ = nullptr;
};

}  // namespace arena_bench
