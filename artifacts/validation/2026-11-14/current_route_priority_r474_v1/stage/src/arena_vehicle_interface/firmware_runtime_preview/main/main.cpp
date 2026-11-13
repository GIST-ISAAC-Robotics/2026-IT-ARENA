// 배포/운영 앱 아님: 실제 peripheral 결합 빌드 후보. 출력은 코드상 항상 금지한다.
// PCNT count는 원시 관측만 보내며 미검증 0을 ActuationReceiver의 유효 속도로 넣지 않는다.
#include "runtime_tx.hpp"
#include "fixed_tick.hpp"
#include "stop_switch.hpp"
#include "st3215_transaction.hpp"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cstdio>

namespace aa=arena_actuation;
static RuntimeTx tx;
// 한 task만 사용하며 mux는 observation 본문을 즉시 복사한다. 프레임별 stack 중첩을 피한다.
static std::uint8_t host_rx[256],servo_rx[64],observation_frame[1024];
static char observation_json[850];
extern "C" void arena_imu_init(std::uint32_t boot);
extern "C" void arena_imu_step();
extern "C" void arena_imu_stop();
extern "C" bool arena_imu_emit(const void* data,std::size_t size){return tx.imu(data,size);}

#include "runtime_peripherals.hpp"

extern "C" void app_main(){
  pwm_zero(); // 센서 초기화의 기다림보다 먼저 출력0을 설정한다.
  gpio_config_t stop_pin{};stop_pin.pin_bit_mask=1ULL<<8;stop_pin.mode=GPIO_MODE_INPUT;
  stop_pin.pull_up_en=GPIO_PULLUP_ENABLE;stop_pin.pull_down_en=GPIO_PULLDOWN_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&stop_pin));
  StopSwitchInput stop_input;
  auto host_events=serial(UART_NUM_0,921600,43,44);
  auto servo_events=serial(UART_NUM_1,1000000,17,18);
  auto unit=encoder();const std::uint32_t boot=esp_random();
  arena_imu_init(boot);
  std::uint32_t identity[8];const char hex[]="0123456789abcdef";
  for(int i=0;i<8;++i)identity[i]=hex[(boot>>((7-i)*4))&15];
  aa::ReceiverConfig cfg;cfg.feedback_mode=aa::FeedbackMode::kMotorShaft;
  cfg.ticks_per_rev=28;cfg.gear_ratio=2.5*81./17.;cfg.wheel_radius_m=.0325;
  aa::TxConfig tc;tc.offer_status_v1=true;tc.offer_motor_feedback_v1=true;
  tc.offer_period_us=20000;tc.diag_period_us=1000000;tc.max_consecutive_events=2;tc.line_ns_per_byte=10851;
  static aa::ActuationMotorEndpoint endpoint(cfg,identity,8,30000,aa::PayloadLayout::kEnvelope,tc,.6,1.2,.05);
  endpoint.connect(esp_timer_get_time());
  arena_servo::Transaction servo;
  int mode=-1,position=-1,raw_count=0;bool servo_failed=false,failed=false,reading_mode=true;
  std::int64_t servo_reply_us=0,capture_begin=0,capture_end=0;
  std::uint64_t observation_seq=0;
  auto start_us=esp_timer_get_time(),servo_due=start_us,report_due=start_us;
  FixedTick control_clock(start_us);
  std::int64_t previous_tick=0,maximum_tick_gap=0;
  std::uint64_t control_ticks=0;
  unsigned minimum_stack_free=uxTaskGetStackHighWaterMark(nullptr);
  servo.read_mode(start_us,1);
  TickType_t last=xTaskGetTickCount();
  for(;;){
    auto now=esp_timer_get_time();
    stop_input.sample(gpio_get_level(GPIO_NUM_8),now,endpoint);
    // 단일 task에서 PCNT/계약5ms. UART 응답 때문에 대기하지 않는다.
    if(control_clock.poll(now)){
      capture_begin=esp_timer_get_time();
      if(previous_tick && capture_begin-previous_tick>maximum_tick_gap)maximum_tick_gap=capture_begin-previous_tick;
      previous_tick=capture_begin;++control_ticks;
      if(pcnt_unit_get_count(unit,&raw_count)!=ESP_OK)failed=true;
      capture_end=esp_timer_get_time();
      if(raw_count>1000000000 || raw_count< -1000000000)failed=true;
      // actual encoder sign/edge가 미검증이므로 endpoint.motor_feedback를 호출하지 않는다.
      auto intent=endpoint.control_tick(capture_end,false);
      if(intent.motor.effort!=0 || endpoint.receiver().armed())failed=true;
      // 다음 예약은 최초 위상에서 유지한다. 지연된 tick을 여러 번 따라잡지 않는다.
    }
    if(!events_ok(host_events))failed=true;
    if(!events_ok(servo_events))servo_failed=true;
    if(!failed){
      int n=uart_read_bytes(UART_NUM_0,host_rx,sizeof(host_rx),0);
      if(n<0)failed=true;else if(n)endpoint.feed(esp_timer_get_time(),host_rx,n,false);
      endpoint.poll(esp_timer_get_time(),false);
      if(!servo_failed){
        n=uart_read_bytes(UART_NUM_1,servo_rx,sizeof(servo_rx),0);
        if(n<0)servo_failed=true;else if(n)servo.feed(esp_timer_get_time(),servo_rx,n);
        servo.pump(esp_timer_get_time(),[](const std::uint8_t* p,std::size_t size)->std::size_t{
          int sent=uart_tx_chars(UART_NUM_1,reinterpret_cast<const char*>(p),size);
          return sent<0?size+1:static_cast<std::size_t>(sent);
        });
        arena_servo::Response response;
        if(servo.take(response)){
          servo_reply_us=response.completed_us;
          if(reading_mode){mode=response.data[0];reading_mode=false;servo_due=servo_reply_us;}
          else {position=response.data[0]|(static_cast<unsigned>(response.data[1])<<8);servo_due=servo_reply_us+50000;}
        }
        if(servo.phase()==arena_servo::Phase::Fault)servo_failed=true;
        if(servo.phase()==arena_servo::Phase::Idle && esp_timer_get_time()>=servo_due)
          if(!servo.read_position(esp_timer_get_time(),1))servo_failed=true;
      }
      // 서보가 없으면 해당 관측만 오류로 남긴다. 출력 불가인 IMU/PCNT 수신을 막지 않는다.
      arena_imu_step();now=esp_timer_get_time();
      if(now>=report_due){
        const unsigned stack_free=uxTaskGetStackHighWaterMark(nullptr);
        if(stack_free<minimum_stack_free)minimum_stack_free=stack_free;
        int size=std::snprintf(observation_json,sizeof(observation_json),
          "{\"type\":\"runtime_observation_v1\",\"boot\":%lu,\"seq\":%llu,\"mcu_us\":%lld,"
          "\"output_ready\":false,\"encoder_motion_verified\":false,\"pcnt_count\":%d,"
          "\"pcnt_begin_us\":%lld,\"pcnt_end_us\":%lld,\"servo_mode\":%d,\"servo_position\":%d,"
          "\"servo_reply_us\":%lld,\"servo_failed\":%s,\"imu_dropped\":%llu,\"observation_dropped\":%llu,\"control_ticks\":%llu,\"missed_control_ticks\":%llu,\"maximum_control_lateness_us\":%lld,\"maximum_tick_gap_us\":%lld,\"main_stack_free_min_bytes\":%u,\"pwm_gpio_level\":%d,\"dir_gpio_level\":%d}",
          static_cast<unsigned long>(boot),static_cast<unsigned long long>(observation_seq++),
          static_cast<long long>(now),raw_count,static_cast<long long>(capture_begin),static_cast<long long>(capture_end),
          mode,position,static_cast<long long>(servo_reply_us),servo_failed?"true":"false",
          static_cast<unsigned long long>(tx.imu_dropped()),static_cast<unsigned long long>(tx.observation_dropped()),static_cast<unsigned long long>(control_ticks),static_cast<unsigned long long>(control_clock.missed()),static_cast<long long>(control_clock.maximum_lateness()),
          static_cast<long long>(maximum_tick_gap),minimum_stack_free,gpio_get_level(GPIO_NUM_15),gpio_get_level(GPIO_NUM_6));
        
        if(size<=0 || size>=static_cast<int>(sizeof(observation_json)))failed=true;
        else {
          auto length=aa::encode_frame(reinterpret_cast<const std::uint8_t*>(observation_json),size,observation_frame,sizeof(observation_frame));
          if(!length)failed=true;else tx.observation(observation_frame,length);
        }
        report_due=now+20000;
      }
      const auto sink=[](const std::uint8_t* p,std::size_t size)->std::size_t{
        int n=uart_tx_chars(UART_NUM_0,reinterpret_cast<const char*>(p),size);
        return n<0?size+1:static_cast<std::size_t>(n);
      };
      if(!tx.pump_bounded(esp_timer_get_time(),endpoint,sink))failed=true;
    }
    if(failed){
      endpoint.disconnect(esp_timer_get_time());arena_imu_stop();servo.cancel();
      ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0));
      vTaskSuspend(nullptr);
    }
    if(xTaskDelayUntil(&last,1)==pdFALSE){last=xTaskGetTickCount();vTaskDelay(1);}
  }
}
