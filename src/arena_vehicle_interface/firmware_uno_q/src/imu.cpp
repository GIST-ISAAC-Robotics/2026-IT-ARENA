#include "hal.hpp"
#include "actuation_wire.hpp"
#include "lsm6dsox_reg.h"
#include <cstring>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
namespace unoq {
static const spi_dt_spec spi=SPI_DT_SPEC_GET(DT_NODELABEL(imu0),SPI_OP_MODE_MASTER|SPI_WORD_SET(8)|SPI_MODE_CPOL|SPI_MODE_CPHA,0);
static const gpio_dt_spec irq=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),imu_irq_gpios);
static gpio_callback callback;
K_MSGQ_DEFINE(imu_queue,sizeof(ImuFrame),128,4);
K_THREAD_STACK_DEFINE(imu_stack,4096);
static k_thread imu_thread;
static atomic_t ready,irq_count,tx_dropped;
static std::int64_t progress=0;
static std::uint32_t boot_id,sequence,spi_errors,gyro_count,accel_count,overruns;
static std::uint8_t who;
static stmdev_ctx_t ctx;
static std::int32_t bus_read(void*,std::uint8_t reg,std::uint8_t* out,std::uint16_t n){
  if(n>64){++spi_errors;return -1;}
  std::uint8_t tx[65]{},rx[65]{};tx[0]=reg|0x80;
  spi_buf tb{tx,std::size_t(n+1)},rb{rx,std::size_t(n+1)};
  spi_buf_set ts{&tb,1},rs{&rb,1};
  if(spi_transceive_dt(&spi,&ts,&rs)!=0){++spi_errors;return -1;}
  std::memcpy(out,rx+1,n);return 0;
}
static std::int32_t bus_write(void*,std::uint8_t reg,const std::uint8_t* data,std::uint16_t n){
  if(n>64){++spi_errors;return -1;}
  std::uint8_t tx[65]{};tx[0]=reg&0x7f;std::memcpy(tx+1,data,n);
  spi_buf tb{tx,std::size_t(n+1)};spi_buf_set ts{&tb,1};
  if(spi_write_dt(&spi,&ts)!=0){++spi_errors;return -1;}return 0;
}
static void on_irq(const device*,gpio_callback*,gpio_port_pins_t){atomic_inc(&irq_count);}
static void emit(ImuFrame& f){
  f.magic=0x31554d49;f.length=sizeof(f);f.version=1;f.boot=boot_id;f.sequence=sequence++;
  f.irq_count=atomic_get(&irq_count);f.spi_errors=spi_errors;f.tx_errors=atomic_get(&tx_dropped);
  f.crc32=arena_actuation::crc32(reinterpret_cast<const std::uint8_t*>(&f),offsetof(ImuFrame,crc32));
  if(k_msgq_put(&imu_queue,&f,K_NO_WAIT)!=0)atomic_inc(&tx_dropped);
}
static bool configure(){
#define CHECK(x) do {if((x)!=0)return false;}while(false)
  CHECK(lsm6dsox_reset_set(&ctx,1));std::uint8_t reset=1;
  for(int i=0;i<100 && reset;++i){k_msleep(2);CHECK(lsm6dsox_reset_get(&ctx,&reset));}
  if(reset)return false;
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
  lsm6dsox_pin_int1_route_t route{};route.fifo_th=1;
  CHECK(lsm6dsox_pin_int1_route_set(&ctx,route));
  CHECK(lsm6dsox_fifo_mode_set(&ctx,LSM6DSOX_BYPASS_MODE));
  CHECK(lsm6dsox_xl_data_rate_set(&ctx,LSM6DSOX_XL_ODR_208Hz));
  CHECK(lsm6dsox_gy_data_rate_set(&ctx,LSM6DSOX_GY_ODR_208Hz));
  k_msleep(100);CHECK(lsm6dsox_fifo_mode_set(&ctx,LSM6DSOX_STREAM_MODE));return true;
#undef CHECK
}
static void report(){
  ImuFrame f{};f.type=3;f.tag=who;f.begin_us=now_us();
  // Old ESP32-only flash/PSRAM fields are zero, not invented measurements.
  f.end_us=now_us();emit(f);
  for(std::uint8_t base:{0x07,0x17,0x56}){
    f={};f.type=4;f.tag=base;f.begin_us=now_us();
    if(bus_read(nullptr,base,f.payload,16))return;
    f.end_us=now_us();emit(f);
  }
  f={};f.type=2;f.begin_us=now_us();std::uint32_t ts=0;std::int16_t temp=0;
  if(lsm6dsox_timestamp_raw_get(&ctx,&ts)||lsm6dsox_temperature_raw_get(&ctx,&temp))return;
  std::memcpy(f.payload,&ts,4);std::memcpy(f.payload+4,&gyro_count,4);
  std::memcpy(f.payload+8,&accel_count,4);std::memcpy(f.payload+12,&temp,2);f.payload[14]=1;
  f.end_us=now_us();emit(f);f={};f.type=5;f.begin_us=now_us();
  std::memcpy(f.payload,&overruns,4);f.end_us=now_us();emit(f);
}
static void work(void*,void*,void*){
  ctx.write_reg=bus_write;ctx.read_reg=bus_read;
  if(!spi_is_ready_dt(&spi)||!gpio_is_ready_dt(&irq))return;
  if(gpio_pin_configure_dt(&irq,GPIO_INPUT)!=0)return;
  gpio_init_callback(&callback,on_irq,BIT(irq.pin));
  if(gpio_add_callback(irq.port,&callback)!=0 || gpio_pin_interrupt_configure_dt(&irq,GPIO_INT_EDGE_RISING)!=0)return;
  if(lsm6dsox_device_id_get(&ctx,&who)||who!=LSM6DSOX_ID||!configure())return;
  std::int64_t next_report=0,last_gyro=-1,last_accel=-1;
  for(;;){
    std::uint8_t status[2]{};
    if(bus_read(nullptr,LSM6DSOX_FIFO_STATUS1,status,2))break;
    unsigned count=status[0]|((status[1]&3)<<8);
    if(status[1]&0x40){++overruns;report();break;} // FIFO loss requires explicit recovery; no silent healthy state.
    const unsigned batch=count>8?8:count;
    for(unsigned i=0;i<batch;++i){
      ImuFrame f{};f.type=1;f.begin_us=now_us();f.fifo_level=count-i;f.fifo_status=status[1];
      std::uint8_t raw[7];if(bus_read(nullptr,LSM6DSOX_FIFO_DATA_OUT_TAG,raw,7))break;
      f.end_us=now_us();f.tag=raw[0];std::memcpy(f.payload,raw+1,6);
      if((raw[0]>>3)==LSM6DSOX_GYRO_NC_TAG){++gyro_count;last_gyro=f.end_us;}
      else if((raw[0]>>3)==LSM6DSOX_XL_NC_TAG){++accel_count;last_accel=f.end_us;}
      emit(f);
    }
    if(spi_errors)break;
    unsigned key=irq_lock();progress=now_us();
    atomic_set(&ready,last_gyro>=0 && last_accel>=0 && progress-last_gyro<50000 && progress-last_accel<50000);irq_unlock(key);
    if(now_us()>=next_report){report();next_report=now_us()+1000000;}
    k_msleep(1);
  }
  atomic_clear(&ready);
}
void imu_start(std::uint32_t boot){boot_id=boot;k_thread_create(&imu_thread,imu_stack,K_THREAD_STACK_SIZEOF(imu_stack),work,nullptr,nullptr,nullptr,6,0,K_NO_WAIT);}
bool imu_ready(){unsigned key=irq_lock();const bool ok=atomic_get(&ready) && now_us()-progress<50000;irq_unlock(key);return ok;}
bool imu_take(ImuFrame& f){return k_msgq_get(&imu_queue,&f,K_NO_WAIT)==0;}
std::uint32_t imu_dropped(){return atomic_get(&tx_dropped);}
}
