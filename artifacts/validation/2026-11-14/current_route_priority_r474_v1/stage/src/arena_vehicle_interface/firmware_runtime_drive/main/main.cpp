// 운영 연결 빌드 후보. 기본 프로필은 무효이며 현재 보드에 설치하지 않는다.
#include "board_profile.hpp"
#include "runtime_tx.hpp"
#include "runtime_peripherals.hpp"
#include "fixed_tick.hpp"
#include "stop_switch.hpp"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <cstdio>

namespace aa=arena_actuation;
namespace as=arena_servo;
static RuntimeTx tx;
static std::uint8_t host_rx[256],servo_rx[64],observation_frame[1024];
static char observation_json[850];
extern "C" void arena_imu_init(std::uint32_t boot);
extern "C" void arena_imu_step();
extern "C" void arena_imu_stop();
extern "C" bool arena_imu_emit(const void* data,std::size_t size){return tx.imu(data,size);}

static bool motor_zero(){
  return ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0)==ESP_OK;
}
static bool apply_motor(const aa::Md20aForwardCommand& command){
  if(command.status!=aa::Md20aMapStatus::kActive || command.duty_ratio==0)return motor_zero();
  // DIR은 실장 프로필의 고정 전진 극성이다. 음수 속도 피드백으로 DIR을 뒤집지 않는다.
  const auto duty=static_cast<std::uint32_t>(std::floor(command.duty_ratio*1023.0));
  return gpio_set_level(GPIO_NUM_6,command.dir_high?1:0)==ESP_OK &&
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty)==ESP_OK &&
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0)==ESP_OK;
}
static as::RuntimeServoSnapshot snapshot(const aa::ActuationMotorEndpoint& endpoint){
  const auto& r=endpoint.receiver();
  return {r.armed(),r.generation(),r.drive_seen(),r.steering()};
}

extern "C" void app_main(){
  pwm_zero();
  gpio_config_t stop_pin{};stop_pin.pin_bit_mask=1ULL<<8;stop_pin.mode=GPIO_MODE_INPUT;
  stop_pin.pull_up_en=GPIO_PULLUP_ENABLE;stop_pin.pull_down_en=GPIO_PULLDOWN_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&stop_pin));
  StopSwitchInput stop;
  auto host_events=serial(UART_NUM_0,921600,43,44);
  auto servo_events=serial(UART_NUM_1,1000000,17,18);
  auto unit=encoder();
  const std::uint32_t boot=esp_random();arena_imu_init(boot);
  std::uint32_t identity[8];const char hex[]="0123456789abcdef";
  for(int i=0;i<8;++i)identity[i]=hex[(boot>>((7-i)*4))&15];
  static const arena_steering::CadMapping map(arena_steering::kCadSamples,
    sizeof(arena_steering::kCadSamples)/sizeof(arena_steering::kCadSamples[0]));
  static const auto profile=installed_board_profile();
  const bool configured=profile.valid(map);
  aa::TxConfig tc;tc.offer_status_v1=true;tc.offer_motor_feedback_v1=true;
  tc.offer_period_us=20000;tc.diag_period_us=1000000;tc.max_consecutive_events=2;tc.line_ns_per_byte=10851;
  static aa::ActuationMotorEndpoint endpoint(profile.endpoint_receiver_config(map),identity,8,30000,
    aa::PayloadLayout::kEnvelope,tc,profile.kp,profile.ki,.05,0.0,profile.maximum_duty);
  static as::RuntimeServo servo(map,profile.servo);
  endpoint.connect(esp_timer_get_time());
  const auto servo_sink=[](const std::uint8_t* data,std::size_t size)->std::size_t{
    int n=uart_tx_chars(UART_NUM_1,reinterpret_cast<const char*>(data),size);
    return n<0?size+1:static_cast<std::size_t>(n);
  };
  const auto host_sink=[](const std::uint8_t* data,std::size_t size)->std::size_t{
    int n=uart_tx_chars(UART_NUM_0,reinterpret_cast<const char*>(data),size);
    return n<0?size+1:static_cast<std::size_t>(n);
  };
  auto start=esp_timer_get_time(),feedback_due=start,report_due=start;
  FixedTick control_clock(start);TickType_t last=xTaskGetTickCount();
  std::int64_t feedback_seq=0,fault_since=-1,last_servo_rx=start;
  int raw_count=0;bool failed=false;
  std::uint64_t observation_seq=0,servo_recoveries=0;
  double applied_duty=0;
  for(;;){
    auto now=esp_timer_get_time();
    stop.sample(gpio_get_level(GPIO_NUM_8),now,endpoint);
    if(stop.held() && !motor_zero())failed=true;
    if(!events_ok(host_events) || !events_ok(servo_events))failed=true;
    if(now>=feedback_due){
      if(pcnt_unit_get_count(unit,&raw_count)!=ESP_OK || raw_count>1000000000 || raw_count< -1000000000)
        failed=true;
      const auto captured=esp_timer_get_time();
      // 최초 count는 차분 기준점이다. 미교정 raw0은 유효 속도인 것처럼 입력하지 않는다.
      if(configured && !failed){
        endpoint.motor_feedback(captured,aa::IntInput::of(feedback_seq++),aa::IntInput::of(captured),
          aa::IntInput::of(static_cast<std::int64_t>(raw_count)*profile.encoder_direction));
      }
      feedback_due=captured+profile.feedback_period_us;
    }
    now=esp_timer_get_time();
    bool ready=configured && !failed && servo.ready(now);
    endpoint.check_deadlines(now,ready); // heartbeat 갱신 없이 만료/준비 소실을 ACK 전에 반영한다.
    endpoint.poll(now,ready);
    if(!failed){
      int n=uart_read_bytes(UART_NUM_0,host_rx,sizeof(host_rx),0);
      if(n<0)failed=true;
      else if(n)endpoint.feed(esp_timer_get_time(),host_rx,n,configured && servo.ready(esp_timer_get_time()));
      if(!failed) n=uart_read_bytes(UART_NUM_1,servo_rx,sizeof(servo_rx),0);
      if(n<0)failed=true;
      if(!failed) {
        now=esp_timer_get_time();if(n)last_servo_rx=now;
        endpoint.check_deadlines(now,configured && servo.ready(now));
        endpoint.poll(now,configured && servo.ready(now));
        servo.step(now,snapshot(endpoint),servo_rx,static_cast<std::size_t>(n),servo_sink);
      }
    }
    now=esp_timer_get_time();ready=configured && !failed && servo.ready(now);
    if(control_clock.poll(now)){
      const auto cycle=endpoint.control_tick(now,ready,!servo.engaged());
      const auto command=aa::md20a_forward_command(cycle,endpoint.receiver(),profile.motor_profile(),
        ready && servo.engaged());
      if(command.status==aa::Md20aMapStatus::kInvalidIntent)failed=true;
      if(!apply_motor(command))failed=true;
      applied_duty=command.status==aa::Md20aMapStatus::kActive?command.duty_ratio:0.0;
    }
    // 제어 tick 사이의 명시 STOP/만료/서보 소실도 다음5ms까지 출력하지 않는다.
    if(!ready || !endpoint.receiver().armed() || !servo.engaged() || endpoint.receiver().target_speed()==0){
      if(!motor_zero())failed=true;
      applied_duty=0;
    }
    if(configured && !failed && servo.state()==as::RuntimeServoState::Fault){
      if(fault_since<0)fault_since=now;
      // TX 완료와 RX 무응답30ms를 둘 다 확인한다. 이 시간은 늦은 ACK의 물리 보증이 아니다.
      if(!endpoint.receiver().armed() && now-fault_since>=as::Transaction::timeout_us &&
          now-last_servo_rx>=as::Transaction::timeout_us && uart_wait_tx_done(UART_NUM_1,0)==ESP_OK){
        if(uart_flush_input(UART_NUM_1)!=ESP_OK)failed=true;
        else if(servo.reset_after_drain(now)){++servo_recoveries;fault_since=-1;}
      }
    }else fault_since=-1;
    if(!failed){
      arena_imu_step();now=esp_timer_get_time();
      if(now>=report_due){
        int size=std::snprintf(observation_json,sizeof(observation_json),
          "{\"type\":\"runtime_drive_observation_v1\",\"boot\":%lu,\"seq\":%llu,\"mcu_us\":%lld,"
          "\"profile_valid\":%s,\"output_ready\":%s,\"servo_engaged\":%s,\"servo_state\":%u,"
          "\"servo_fault\":%u,\"servo_recoveries\":%llu,\"pcnt_count\":%d,\"duty_request\":%.6f,"
          "\"control_missed\":%llu,\"control_late_us\":%lld,\"imu_dropped\":%llu}",
          static_cast<unsigned long>(boot),static_cast<unsigned long long>(observation_seq++),
          static_cast<long long>(now),configured?"true":"false",ready?"true":"false",
          servo.engaged()?"true":"false",static_cast<unsigned>(servo.state()),static_cast<unsigned>(servo.fault()),
          static_cast<unsigned long long>(servo_recoveries),raw_count,applied_duty,
          static_cast<unsigned long long>(control_clock.missed()),static_cast<long long>(control_clock.maximum_lateness()),
          static_cast<unsigned long long>(tx.imu_dropped()));
        if(size<=0 || size>=static_cast<int>(sizeof(observation_json)))failed=true;
        else {
          auto length=aa::encode_frame(reinterpret_cast<const std::uint8_t*>(observation_json),size,
            observation_frame,sizeof(observation_frame));
          if(!length)failed=true;else tx.observation(observation_frame,length);
        }
        report_due=now+20000;
      }
      if(!tx.pump_bounded(esp_timer_get_time(),endpoint,host_sink,
          configured && servo.ready(esp_timer_get_time())))failed=true;
      if(!endpoint.receiver().armed() && !motor_zero())failed=true;
    }
    if(failed){
      motor_zero();endpoint.disconnect(esp_timer_get_time());arena_imu_stop();
      // UART/HAL 치명 오류에서 자동 재허가는 하지 않는다. 외부 독립 차단 검증은 별도다.
      vTaskSuspend(nullptr);
    }
    if(xTaskDelayUntil(&last,1)==pdFALSE){last=xTaskGetTickCount();vTaskDelay(1);}
  }
}
