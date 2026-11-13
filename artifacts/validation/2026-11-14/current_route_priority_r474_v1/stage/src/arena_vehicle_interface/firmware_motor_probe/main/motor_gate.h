// 단독 벤치용 단발 구동. 운영 제어/하드웨어 독립 차단을 대신하지 않음.
#include <stdlib.h>
#include "driver/ledc.h"
#include "freertos/semphr.h"

#define MOTOR_MAX_PERMILLE 100
#define MOTOR_MAX_MS 500
#define MOTOR_ARM_US 2000000LL
#define MOTOR_COOLDOWN_US 3000000LL
static SemaphoreHandle_t motor_lock;
static esp_timer_handle_t motor_timer;
static bool motor_active, motor_armed;
static uint32_t motor_token, motor_last_id, motor_pulses, motor_per_mille;
static int64_t motor_arm_deadline, motor_deadline, motor_started, motor_ended;
static const char *motor_reason="boot";

static void motor_off_locked(const char *reason) {
  ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0));
  if(motor_active)motor_ended=esp_timer_get_time();
  motor_active=false;motor_armed=false;motor_token=0;motor_per_mille=0;motor_reason=reason;
}
static void motor_off(const char *reason) {
  xSemaphoreTake(motor_lock,portMAX_DELAY);motor_off_locked(reason);xSemaphoreGive(motor_lock);
}
static void motor_tick(void *arg) {
  (void)arg;
  xSemaphoreTake(motor_lock,portMAX_DELAY);
  int64_t now=esp_timer_get_time();
  if(motor_active && now>=motor_deadline)motor_off_locked("deadline");
  else if(motor_armed && now>=motor_arm_deadline)motor_off_locked("arm_expired");
  xSemaphoreGive(motor_lock);
}
static void motor_setup(void) {
  motor_lock=xSemaphoreCreateMutex();configASSERT(motor_lock);
  ledc_timer_config_t t={.speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_0,
    .duty_resolution=LEDC_TIMER_10_BIT,.freq_hz=20000,.clk_cfg=LEDC_USE_XTAL_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&t));
  ledc_channel_config_t c={.gpio_num=15,.speed_mode=LEDC_LOW_SPEED_MODE,
    .channel=LEDC_CHANNEL_0,.intr_type=LEDC_INTR_DISABLE,.timer_sel=LEDC_TIMER_0,.duty=0,.hpoint=0};
  ESP_ERROR_CHECK(ledc_channel_config(&c));
  ESP_ERROR_CHECK(gpio_input_enable(15));
  motor_off("boot");
  esp_timer_create_args_t args={.callback=motor_tick,.name="motor_deadline"};
  ESP_ERROR_CHECK(esp_timer_create(&args,&motor_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(motor_timer,1000));
}
static void motor_status(void) {
  char b[700];xSemaphoreTake(motor_lock,portMAX_DELAY);
  snprintf(b,sizeof(b),"{\"event\":\"motor\",\"boot\":%lu,\"seq\":%lu,\"us\":%lld,\"active\":%s,\"armed\":%s,\"last_id\":%lu,\"pulses\":%lu,\"duty_permille\":%lu,\"started_us\":%lld,\"deadline_us\":%lld,\"ended_us\":%lld,\"reason\":\"%s\",\"pwm_pin\":%d,\"dir_pin\":%d}",
    (unsigned long)boot,(unsigned long)sequence++,(long long)esp_timer_get_time(),
    motor_active?"true":"false",motor_armed?"true":"false",(unsigned long)motor_last_id,
    (unsigned long)motor_pulses,(unsigned long)motor_per_mille,(long long)motor_started,
    (long long)motor_deadline,(long long)motor_ended,motor_reason,gpio_get_level(15),gpio_get_level(6));
  xSemaphoreGive(motor_lock);emit(b);
}
static void motor_reply(uint32_t id,const char *result,uint32_t token) {
  char b[250];snprintf(b,sizeof(b),"{\"event\":\"reply\",\"boot\":%lu,\"seq\":%lu,\"id\":%lu,\"result\":\"%s\",\"token\":%lu}",
    (unsigned long)boot,(unsigned long)sequence++,(unsigned long)id,result,(unsigned long)token);emit(b);
}
static bool motor_u32(const char *text,uint32_t *out) {
  if(!text || !*text)return false;
  uint64_t n=0;for(const char *p=text;*p;++p){if(*p<'0'||*p>'9')return false;n=n*10+(*p-'0');if(n>UINT32_MAX)return false;}
  *out=(uint32_t)n;return true;
}
static void motor_command(char *line) {
  // ARM/PULSE 모두 CRC와 현재 boot, 증가하는 요청 ID가 필요하다.
  size_t len=strlen(line);uint32_t crc=0;
  if(len<10 || line[8]!=' '){motor_off("bad_frame");motor_reply(0,"bad_frame",0);return;}
  for(int i=0;i<8;++i){char c=line[i];unsigned v;
    if(c>='0'&&c<='9')v=c-'0';else if(c>='a'&&c<='f')v=c-'a'+10;
    else if(c>='A'&&c<='F')v=c-'A'+10;else {motor_off("bad_crc");motor_reply(0,"bad_crc",0);return;}
    crc=(crc<<4)|v;
  }
  if(crc32(line+9,len-9)!=crc){motor_off("bad_crc");motor_reply(0,"bad_crc",0);return;}
  char *parts[7],*save=NULL;unsigned n=0;
  for(char *p=strtok_r(line+9," ",&save);p && n<7;p=strtok_r(NULL," ",&save))parts[n++]=p;
  uint32_t request_boot=0,id=0,token=0,duty=0,ms=0;bool arm=n==3 && !strcmp(parts[0],"ARM");
  bool pulse=n==6 && !strcmp(parts[0],"PULSE");
  if((!arm&&!pulse)||!motor_u32(parts[1],&request_boot)||!motor_u32(parts[2],&id)||
     (pulse&&(!motor_u32(parts[3],&token)||!motor_u32(parts[4],&duty)||!motor_u32(parts[5],&ms)))) {
    motor_off("bad_args");motor_reply(0,"bad_args",0);return;
  }
  const char *result="rejected";uint32_t issued=0;
  xSemaphoreTake(motor_lock,portMAX_DELAY);int64_t now=esp_timer_get_time();
  if(request_boot!=boot){result="wrong_boot";motor_off_locked(result);}
  else if(id==0 || id<=motor_last_id){result="old_id";motor_off_locked(result);}
  else {
    motor_last_id=id;
    if(motor_active){result="busy";motor_off_locked(result);}
    else if(arm) {
      if(motor_ended && now-motor_ended<MOTOR_COOLDOWN_US){result="cooldown";motor_off_locked(result);}
      else {motor_armed=true;motor_token=esp_random();if(!motor_token)motor_token=1;
        motor_arm_deadline=now+MOTOR_ARM_US;issued=motor_token;result="armed";}
    } else if(!motor_armed || !token || token!=motor_token || now>=motor_arm_deadline) {
      result="not_armed";motor_off_locked(result);
    } else if(duty<1 || duty>MOTOR_MAX_PERMILLE || ms<1 || ms>MOTOR_MAX_MS) {
      result="out_of_range";motor_off_locked(result);
    } else {
      motor_armed=false;motor_token=0;motor_started=esp_timer_get_time();
      motor_deadline=motor_started+(int64_t)ms*1000;motor_per_mille=duty;
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,(duty*1024+500)/1000));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0));
      motor_active=true;++motor_pulses;motor_reason="pulse";result="started";
    }
  }
  xSemaphoreGive(motor_lock);motor_reply(id,result,issued);motor_status();
}
