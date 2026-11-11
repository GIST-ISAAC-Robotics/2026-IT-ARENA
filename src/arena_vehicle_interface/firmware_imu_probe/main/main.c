// 센서 진단 전용: 구동 출력 없음. 보정 계수 적용/저장 없음.
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm6dsox_reg.h"

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t length;
  uint8_t version, type;
  uint32_t boot, sequence;
  uint64_t begin_us, end_us;
  uint32_t irq_count;
  uint16_t fifo_level;
  uint8_t fifo_status, tag;
  uint8_t payload[16];
  uint32_t spi_errors, tx_errors;
  uint32_t crc32;
} frame_t;
_Static_assert(sizeof(frame_t) == 68, "wire size");

static spi_device_handle_t spi;
static stmdev_ctx_t ctx;
static uint32_t boot_id, sequence, spi_errors, tx_errors;
static uint32_t gyro_count, accel_count, overrun_observations;
static volatile uint32_t irq_count;
static bool running;
static uint8_t who;
static uint32_t flash_bytes, psram_bytes, memory_errors;
static int64_t run_start;

static uint32_t crc32(const void *data, size_t n) {
  uint32_t c = 0xffffffff;
  const uint8_t *p = data;
  while (n--) {
    c ^= *p++;
    for (int j=0; j<8; ++j) c = (c >> 1) ^ (0xedb88320u & (0u-(c&1u)));
  }
  return ~c;
}

static int32_t bus_read(void *handle, uint8_t reg, uint8_t *out, uint16_t n) {
  (void)handle;
  if (n>64) { ++spi_errors; return -1; }
  uint8_t tx[65]={0}, rx[65]={0};
  tx[0] = reg | 0x80;
  spi_transaction_t t={.length=(n+1)*8, .tx_buffer=tx, .rx_buffer=rx};
  if (spi_device_polling_transmit(spi,&t)!=ESP_OK) { ++spi_errors; return -1; }
  memcpy(out,rx+1,n);
  return 0;
}

static int32_t bus_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t n) {
  (void)handle;
  if (n>64) { ++spi_errors; return -1; }
  uint8_t tx[65]={0}; tx[0]=reg & 0x7f; memcpy(tx+1,data,n);
  spi_transaction_t t={.length=(n+1)*8, .tx_buffer=tx};
  if (spi_device_polling_transmit(spi,&t)!=ESP_OK) { ++spi_errors; return -1; }
  return 0;
}

static void IRAM_ATTR irq_handler(void *arg) { (void)arg; ++irq_count; }

static void emit(frame_t *f) {
  f->magic=0x31554d49; f->length=sizeof(*f); f->version=1;
  f->boot=boot_id; f->sequence=sequence++; f->irq_count=irq_count;
  f->spi_errors=spi_errors; f->tx_errors=tx_errors;
  f->crc32=crc32(f,offsetof(frame_t,crc32));
  int n=uart_write_bytes(UART_NUM_0,f,sizeof(*f));
  if (n!=sizeof(*f)) ++tx_errors;
}

static void report(void) {
  frame_t f={.type=3,.begin_us=esp_timer_get_time(),.tag=who};
  uint32_t info[4]={flash_bytes,psram_bytes,memory_errors,esp_reset_reason()};
  memcpy(f.payload,info,sizeof(info)); f.end_us=esp_timer_get_time(); emit(&f);
  const uint8_t bases[]={0x07,0x17,0x56};
  for (unsigned i=0;i<sizeof(bases);++i) {
    f=(frame_t){.type=4,.tag=bases[i],.begin_us=esp_timer_get_time()};
    if (bus_read(NULL,bases[i],f.payload,16)) break;
    f.end_us=esp_timer_get_time(); emit(&f);
  }
  f=(frame_t){.type=2,.begin_us=esp_timer_get_time()};
  uint32_t ts=0; int16_t temp=0;
  if (lsm6dsox_timestamp_raw_get(&ctx,&ts) || lsm6dsox_temperature_raw_get(&ctx,&temp)) return;
  memcpy(f.payload,&ts,4); memcpy(f.payload+4,&gyro_count,4);
  memcpy(f.payload+8,&accel_count,4); memcpy(f.payload+12,&temp,2);
  f.payload[14]=running;
  uint8_t s[2]={0};
  if (bus_read(NULL,LSM6DSOX_FIFO_STATUS1,s,2)) return;
  f.fifo_level=s[0]|((s[1]&3)<<8); f.fifo_status=s[1];
  f.end_us=esp_timer_get_time(); emit(&f);
  f=(frame_t){.type=5,.begin_us=esp_timer_get_time()};
  memcpy(f.payload,&overrun_observations,4);
  f.end_us=esp_timer_get_time(); emit(&f);
}

static int configure(void) {
#define CHECK(call) do { if ((call)!=0) return -1; } while(0)
  CHECK(lsm6dsox_reset_set(&ctx,1));
  uint8_t reset=1;
  for(int i=0;i<100 && reset;++i) {
    vTaskDelay(pdMS_TO_TICKS(2)); CHECK(lsm6dsox_reset_get(&ctx,&reset));
  }
  if(reset) return -1;
  CHECK(lsm6dsox_block_data_update_set(&ctx,1));
  CHECK(lsm6dsox_auto_increment_set(&ctx,1));
  CHECK(lsm6dsox_xl_full_scale_set(&ctx,LSM6DSOX_8g));
  CHECK(lsm6dsox_gy_full_scale_set(&ctx,LSM6DSOX_500dps));
  CHECK(lsm6dsox_xl_power_mode_set(&ctx,LSM6DSOX_HIGH_PERFORMANCE_MD));
  CHECK(lsm6dsox_gy_power_mode_set(&ctx,LSM6DSOX_GY_HIGH_PERFORMANCE));
  CHECK(lsm6dsox_timestamp_set(&ctx,1));
  CHECK(lsm6dsox_compression_algo_set(&ctx,LSM6DSOX_CMP_DISABLE));
  CHECK(lsm6dsox_fifo_watermark_set(&ctx,8));
  CHECK(lsm6dsox_fifo_xl_batch_set(&ctx,LSM6DSOX_XL_BATCHED_AT_208Hz));
  CHECK(lsm6dsox_fifo_gy_batch_set(&ctx,LSM6DSOX_GY_BATCHED_AT_208Hz));
  CHECK(lsm6dsox_fifo_timestamp_decimation_set(&ctx,LSM6DSOX_DEC_1));
  lsm6dsox_pin_int1_route_t route={0}; route.fifo_th=1;
  CHECK(lsm6dsox_pin_int1_route_set(&ctx,route));
  return 0;
}

static void stop(void) {
  running=false;
  if(who!=LSM6DSOX_ID) return;
  if (lsm6dsox_fifo_mode_set(&ctx,LSM6DSOX_BYPASS_MODE) ||
      lsm6dsox_xl_data_rate_set(&ctx,LSM6DSOX_XL_ODR_OFF) ||
      lsm6dsox_gy_data_rate_set(&ctx,LSM6DSOX_GY_ODR_OFF)) ++spi_errors;
}

static void start(void) {
  if(who!=LSM6DSOX_ID || memory_errors || spi_errors) return;
  stop();
  if(lsm6dsox_timestamp_rst(&ctx) ||
     lsm6dsox_xl_data_rate_set(&ctx,LSM6DSOX_XL_ODR_208Hz) ||
     lsm6dsox_gy_data_rate_set(&ctx,LSM6DSOX_GY_ODR_208Hz)) { ++spi_errors; return; }
  vTaskDelay(pdMS_TO_TICKS(100));
  if(lsm6dsox_fifo_mode_set(&ctx,LSM6DSOX_STREAM_MODE)) { ++spi_errors; return; }
  gyro_count=accel_count=overrun_observations=0;
  irq_count=0; run_start=esp_timer_get_time(); running=true;
}

void app_main(void) {
  boot_id=esp_random();
  uart_config_t uc={.baud_rate=921600,.data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,1024,4096,0,NULL,0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&uc));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,43,44,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(esp_flash_get_size(NULL,&flash_bytes));
  psram_bytes=esp_psram_get_size();
  uint32_t *test=heap_caps_malloc(256*1024,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(!test) ++memory_errors;
  else {
    for(unsigned i=0;i<256*1024/4;++i) test[i]=0x9e3779b9u*i;
    for(unsigned i=0;i<256*1024/4;++i) if(test[i]!=0x9e3779b9u*i) ++memory_errors;
    heap_caps_free(test);
  }
  spi_bus_config_t bc={.mosi_io_num=11,.miso_io_num=13,.sclk_io_num=12,
    .quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=65};
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&bc,SPI_DMA_DISABLED));
  spi_device_interface_config_t dc={.clock_speed_hz=1000000,.mode=3,
    .spics_io_num=10,.queue_size=1};
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&dc,&spi));
  ctx=(stmdev_ctx_t){.write_reg=bus_write,.read_reg=bus_read};
  gpio_config_t gc={.pin_bit_mask=1ULL<<9,.mode=GPIO_MODE_INPUT,
    .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,
    .intr_type=GPIO_INTR_POSEDGE};
  ESP_ERROR_CHECK(gpio_config(&gc));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(9,irq_handler,NULL));
  if(lsm6dsox_device_id_get(&ctx,&who)) ++spi_errors;
  if(who==LSM6DSOX_ID && configure()) ++spi_errors;
  int64_t next_report=0;
  while(true) {
    uint8_t command;
    while(uart_read_bytes(UART_NUM_0,&command,1,0)==1) {
      if(command=='S') start();
      else if(command=='X') stop();
      else if(command=='R') esp_restart();
    }
    if(running && esp_timer_get_time()-run_start>600000000) stop();
    if(running) {
      uint8_t s[2]={0};
      if(bus_read(NULL,LSM6DSOX_FIFO_STATUS1,s,2)) {stop(); continue;}
      uint16_t n=s[0]|((s[1]&3)<<8);
      if(s[1]&0x40) ++overrun_observations;
      if(n>128) n=128;  // 한 순회의 처리량을 제한한다.
      for(uint16_t i=0;i<n;++i) {
        frame_t f={.type=1,.fifo_level=n-i,.fifo_status=s[1],.begin_us=esp_timer_get_time()};
        uint8_t raw[7];
        if(bus_read(NULL,LSM6DSOX_FIFO_DATA_OUT_TAG,raw,7)) {stop(); break;}
        f.end_us=esp_timer_get_time(); f.tag=raw[0]; memcpy(f.payload,raw+1,6);
        uint8_t kind=raw[0]>>3;
        if(kind==LSM6DSOX_GYRO_NC_TAG) ++gyro_count;
        else if(kind==LSM6DSOX_XL_NC_TAG) ++accel_count;
        emit(&f);
      }
    }
    if(esp_timer_get_time()>=next_report) {
      report(); next_report=esp_timer_get_time()+1000000;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
