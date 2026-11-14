#include "hal.hpp"
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_gpio.h>
namespace unoq {
static const device* const uart=DEVICE_DT_GET(DT_NODELABEL(usart1));
static const device* const pwm=DEVICE_DT_GET(DT_NODELABEL(pwm4));
static const device* const qdec=DEVICE_DT_GET(DT_NODELABEL(qdec3));
static const device* const entropy=DEVICE_DT_GET(DT_NODELABEL(rng));
static const device* const wdt=DEVICE_DT_GET(DT_NODELABEL(iwdg));
static int wdt_channel=-1;
static const gpio_dt_spec direction=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),motor_dir_gpios);
static const gpio_dt_spec pwm_gpio=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),motor_pwm_gpios);
RING_BUF_DECLARE(rx_ring,4096);
RING_BUF_DECLARE(tx_ring,2048);
static atomic_t failed,trip,uart_error_count;
static std::int64_t output_deadline=0;
std::int64_t now_us(){return static_cast<std::int64_t>(k_ticks_to_us_floor64(k_uptime_ticks()));}
static void force_low(){
  LL_TIM_CC_DisableChannel(TIM4,LL_TIM_CHANNEL_CH3);
  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_8);
  LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_8,LL_GPIO_MODE_OUTPUT);
  output_deadline=0;
}
static void deadline_guard(k_timer*){
  if(output_deadline && now_us()>=output_deadline){atomic_set(&trip,1);force_low();}
}
K_TIMER_DEFINE(guard_timer,deadline_guard,nullptr);
void fail_hal(){unsigned key=irq_lock();atomic_set(&failed,1);force_low();irq_unlock(key);}
bool hal_failed(){return atomic_get(&failed)!=0;}
bool guard_tripped(){return atomic_cas(&trip,1,0);}
void motor_zero(){unsigned key=irq_lock();force_low();irq_unlock(key);}
bool motor_set(double duty,bool dir,std::int64_t deadline){
  if(!std::isfinite(duty) || duty<0 || duty>1){fail_hal();return false;}
  if(duty==0){motor_zero();return !hal_failed();}
  unsigned key=irq_lock();
  bool ok=!hal_failed() && !atomic_get(&trip) && deadline>now_us();
  if(ok)ok=gpio_pin_set_dt(&direction,dir?1:0)==0;
  if(ok){
    // Fixed verified board mapping: PB8 AF2 = TIM4_CH3, never PB9/IMU CS.
    LL_GPIO_SetAFPin_8_15(GPIOB,LL_GPIO_PIN_8,LL_GPIO_AF_2);
    LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE);
  }
  if(ok)ok=pwm_set(pwm,3,50000,static_cast<std::uint32_t>(duty*50000),0)==0;
  if(ok)output_deadline=deadline;
  else {atomic_set(&failed,1);force_low();}
  irq_unlock(key);return ok;
}
static void uart_irq(const device* dev,void*){
  if(!uart_irq_update(dev))return;
  if(uart_err_check(dev)!=0){atomic_inc(&uart_error_count);fail_hal();}
  std::uint8_t bytes[32];
  while(uart_irq_rx_ready(dev)){
    int n=uart_fifo_read(dev,bytes,sizeof(bytes));if(n<=0)break;
    if(ring_buf_put(&rx_ring,bytes,n)!=static_cast<std::uint32_t>(n)){
      atomic_inc(&uart_error_count);fail_hal();
    }
  }
  if(uart_irq_tx_ready(dev)){
    std::uint8_t* data=nullptr;const auto count=ring_buf_get_claim(&tx_ring,&data,32);
    if(count){int n=uart_fifo_fill(dev,data,count);ring_buf_get_finish(&tx_ring,n>0?n:0);if(n<0)fail_hal();}
    else uart_irq_tx_disable(dev);
  }
}
bool uart_start(){
  if(!device_is_ready(uart))return false;
  uart_config config{1000000,UART_CFG_PARITY_NONE,UART_CFG_STOP_BITS_1,UART_CFG_DATA_BITS_8,UART_CFG_FLOW_CTRL_NONE};
  if(uart_configure(uart,&config)!=0)return false;
  if(uart_irq_callback_user_data_set(uart,uart_irq,nullptr)!=0)return false;
  uart_irq_rx_enable(uart);uart_irq_err_enable(uart);return true;
}
std::size_t uart_read(std::uint8_t* p,std::size_t n){unsigned key=irq_lock();auto got=ring_buf_get(&rx_ring,p,n);irq_unlock(key);return got;}
std::size_t uart_write(const std::uint8_t* p,std::size_t n){
  unsigned key=irq_lock();const auto sent=ring_buf_put(&tx_ring,p,n);
  if(sent)uart_irq_tx_enable(uart);
  irq_unlock(key);return sent;
}
std::uint32_t uart_errors(){return atomic_get(&uart_error_count);}
bool hal_init(){
  if(!gpio_is_ready_dt(&direction)||!gpio_is_ready_dt(&pwm_gpio))return false;
  if(gpio_pin_configure_dt(&pwm_gpio,GPIO_OUTPUT_INACTIVE)!=0 ||
     gpio_pin_configure_dt(&direction,GPIO_OUTPUT_INACTIVE)!=0)return false;
  motor_zero();
  if(!device_is_ready(pwm)||!device_is_ready(qdec)||!device_is_ready(entropy)||!device_is_ready(wdt))return false;
  wdt_timeout_cfg timeout{};timeout.window.max=200;timeout.flags=WDT_FLAG_RESET_SOC;
  wdt_channel=wdt_install_timeout(wdt,&timeout);
  if(wdt_channel<0 || wdt_setup(wdt,0)!=0)return false;
  k_timer_start(&guard_timer,K_MSEC(1),K_MSEC(1));return uart_start();
}
bool watchdog_progress(){return wdt_channel>=0 && wdt_feed(wdt,wdt_channel)==0;}
bool encoder_read(std::uint32_t& count,std::uint32_t& modulus){
  sensor_value value{};
  if(sensor_sample_fetch_chan(qdec,SENSOR_CHAN_ENCODER_COUNT)!=0 ||
     sensor_channel_get(qdec,SENSOR_CHAN_ENCODER_COUNT,&value)!=0)return false;
  count=value.val1;modulus=LL_TIM_GetAutoReload(TIM3)+1;
  return modulus==65520 && count<modulus;
}
std::uint32_t boot_random(){std::uint32_t out=0;if(entropy_get_entropy(entropy,reinterpret_cast<std::uint8_t*>(&out),sizeof(out))!=0)fail_hal();return out;}
}
extern "C" void k_sys_fatal_error_handler(unsigned int,const struct arch_esf*) {
  unoq::fail_hal();for(;;){k_cpu_idle();}
}
