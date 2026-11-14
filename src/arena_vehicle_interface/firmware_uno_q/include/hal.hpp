#pragma once
#include <cstddef>
#include <cstdint>
namespace unoq {
std::int64_t now_us();
bool hal_init();
bool uart_start();
std::size_t uart_read(std::uint8_t*,std::size_t);
std::size_t uart_write(const std::uint8_t*,std::size_t);
bool hal_failed();
void fail_hal();
bool motor_set(double duty,bool dir,std::int64_t absolute_deadline);
void motor_zero();
bool guard_tripped();
bool encoder_read(std::uint32_t& count,std::uint32_t& modulus);
std::uint32_t boot_random();
std::uint32_t uart_errors();
bool watchdog_progress();
struct __attribute__((packed)) ImuFrame {
  std::uint32_t magic; std::uint16_t length; std::uint8_t version,type;
  std::uint32_t boot,sequence; std::uint64_t begin_us,end_us;
  std::uint32_t irq_count; std::uint16_t fifo_level; std::uint8_t fifo_status,tag;
  std::uint8_t payload[16]; std::uint32_t spi_errors,tx_errors,crc32;
};
static_assert(sizeof(ImuFrame)==68);
void imu_start(std::uint32_t boot);
bool imu_ready();
bool imu_take(ImuFrame& frame);
std::uint32_t imu_dropped();
}
