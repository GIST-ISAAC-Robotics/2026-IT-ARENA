// 단발 모터 벤치와 엔코더 관측. 속도 폐루프/운영 제어 앱이 아님.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RING_SIZE 256
typedef struct { uint32_t number; int64_t us; uint8_t before,after; int8_t delta; } edge_t;
static const int8_t steps[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static edge_t ring[RING_SIZE];
static volatile unsigned head,tail;
static volatile uint32_t edges,ar,af,br,bf,invalid,same,irqs,dropped;
static volatile int32_t software_count;
static volatile uint8_t state;
static volatile int64_t last_edge_us;
static pcnt_unit_handle_t unit;
static uint32_t boot,sequence,tx_errors;
static bool streaming;
static int64_t stream_start;

static uint32_t crc32(const char *s,size_t n) {
  uint32_t c=0xffffffff;
  while(n--) {c^=(uint8_t)*s++;for(int i=0;i<8;i++)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));}
  return ~c;
}
static void emit(const char *json) {
  char line[800];size_t len=strlen(json);
  int n=snprintf(line,sizeof(line),"%08lx %s\n",(unsigned long)crc32(json,len),json);
  if(n<=0 || n>=(int)sizeof(line)) {++tx_errors;return;}
  if(uart_write_bytes(UART_NUM_0,line,n)!=n)++tx_errors;
}
#include "motor_gate.h"

static uint8_t pins(void) {return (gpio_get_level(4)<<1)|gpio_get_level(5);}
static void IRAM_ATTR on_edge(void *arg) {
  (void)arg;
  portENTER_CRITICAL_ISR(&lock);
  uint8_t next=pins(),previous=state,changed=next^previous;
  ++irqs;
  if(!changed) {++same;portEXIT_CRITICAL_ISR(&lock);return;}
  int64_t now=esp_timer_get_time();
  if(changed&2) {if(next&2)++ar;else ++af;}
  if(changed&1) {if(next&1)++br;else ++bf;}
  if(changed==3)++invalid;
  int8_t delta=steps[(previous<<2)|next];
  software_count+=delta;state=next;last_edge_us=now;++edges;
  unsigned following=(head+1)%RING_SIZE;
  if(following==tail)++dropped;
  else {ring[head]=(edge_t){.number=edges,.us=now,.before=previous,.after=next,.delta=delta};head=following;}
  portEXIT_CRITICAL_ISR(&lock);
}
static void info(void) {
  char b[600];
  snprintf(b,sizeof(b),"{\"event\":\"info\",\"firmware\":\"motor_probe_v1\",\"boot\":%lu,\"seq\":%lu,\"gpio_a\":4,\"gpio_b\":5,\"pulls\":\"disabled\",\"pcnt_filter_ns\":1000,\"decode\":\"quadrature_x4\",\"positive_sequence\":\"00,10,11,01,00\",\"motor_pwm\":%d,\"motor_dir\":%d}",
    (unsigned long)boot,(unsigned long)sequence++,gpio_get_level(15),gpio_get_level(6));emit(b);
}
static void status(void) {
  int64_t begin=esp_timer_get_time();int count=0;
  ESP_ERROR_CHECK(pcnt_unit_get_count(unit,&count));
  portENTER_CRITICAL(&lock);
  int32_t soft=software_count;uint32_t e=edges,a=ar,b=af,c=br,d=bf,i=invalid,s=same,q=irqs,l=dropped;
  uint8_t st=state;int64_t last=last_edge_us;
  portEXIT_CRITICAL(&lock);
  uint8_t raw_ab=pins();
  int64_t end=esp_timer_get_time();char text[700];
  snprintf(text,sizeof(text),"{\"event\":\"status\",\"boot\":%lu,\"seq\":%lu,\"begin_us\":%lld,\"end_us\":%lld,\"pcnt_count\":%d,\"software_count\":%ld,\"edges\":%lu,\"a_rise\":%lu,\"a_fall\":%lu,\"b_rise\":%lu,\"b_fall\":%lu,\"invalid\":%lu,\"same_state_irq\":%lu,\"irqs\":%lu,\"ring_dropped\":%lu,\"ab\":%u,\"raw_ab\":%u,\"last_edge_us\":%lld,\"streaming\":%s,\"tx_errors\":%lu}",
    (unsigned long)boot,(unsigned long)sequence++,(long long)begin,(long long)end,count,(long)soft,
    (unsigned long)e,(unsigned long)a,(unsigned long)b,(unsigned long)c,(unsigned long)d,
    (unsigned long)i,(unsigned long)s,(unsigned long)q,(unsigned long)l,st,raw_ab,(long long)last,
    streaming?"true":"false",(unsigned long)tx_errors);emit(text);
}
static void drain(void) {
  for(int n=0;n<RING_SIZE;++n) {
    edge_t e;bool available=false;
    portENTER_CRITICAL(&lock);
    if(tail!=head) {e=ring[tail];tail=(tail+1)%RING_SIZE;available=true;}
    portEXIT_CRITICAL(&lock);
    if(!available)break;
    char b[300];snprintf(b,sizeof(b),"{\"event\":\"edge\",\"boot\":%lu,\"seq\":%lu,\"edge_number\":%lu,\"edge_us\":%lld,\"before\":%u,\"after\":%u,\"delta\":%d}",
      (unsigned long)boot,(unsigned long)sequence++,(unsigned long)e.number,(long long)e.us,e.before,e.after,e.delta);emit(b);
  }
}
void app_main(void) {
  gpio_set_level(15,0);gpio_set_level(6,0);
  gpio_config_t out={.pin_bit_mask=(1ULL<<15)|(1ULL<<6),.mode=GPIO_MODE_INPUT_OUTPUT};
  ESP_ERROR_CHECK(gpio_config(&out));
  motor_setup();
  uart_config_t serial={.baud_rate=921600,.data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,1024,8192,0,NULL,0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&serial));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,43,44,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
  gpio_config_t input={.pin_bit_mask=(1ULL<<4)|(1ULL<<5),.mode=GPIO_MODE_INPUT,
    .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&input));
  pcnt_unit_config_t config={.high_limit=30000,.low_limit=-30000,.flags.accum_count=true};
  ESP_ERROR_CHECK(pcnt_new_unit(&config,&unit));
  pcnt_glitch_filter_config_t filter={.max_glitch_ns=1000};
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit,&filter));
  pcnt_channel_handle_t a,b;
  pcnt_chan_config_t ca={.edge_gpio_num=4,.level_gpio_num=5};
  pcnt_chan_config_t cb={.edge_gpio_num=5,.level_gpio_num=4};
  ESP_ERROR_CHECK(pcnt_new_channel(unit,&ca,&a));
  ESP_ERROR_CHECK(pcnt_new_channel(unit,&cb,&b));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a,PCNT_CHANNEL_EDGE_ACTION_DECREASE,PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(a,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b,PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(b,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit,30000));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit,-30000));
  ESP_ERROR_CHECK(gpio_set_pull_mode(4,GPIO_FLOATING));
  ESP_ERROR_CHECK(gpio_set_pull_mode(5,GPIO_FLOATING));
  state=pins();
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_set_intr_type(4,GPIO_INTR_ANYEDGE));
  ESP_ERROR_CHECK(gpio_set_intr_type(5,GPIO_INTR_ANYEDGE));
  ESP_ERROR_CHECK(gpio_isr_handler_add(4,on_edge,NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(5,on_edge,NULL));
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  boot=esp_random();info();status();motor_status();int64_t last=esp_timer_get_time();
  char line[160];size_t used=0;bool overflow=false;int64_t first_byte=0;
  for(;;) {
    uint8_t cmd;int received=0;
    while(received++<256 && uart_read_bytes(UART_NUM_0,&cmd,1,0)==1) {
      if(cmd=='\n') {
        if(overflow){motor_off("overflow");motor_reply(0,"overflow",0);}
        else {
          line[used]=0;
          if(!strcmp(line,"?")){info();status();motor_status();}
          else if(!strcmp(line,"S")){streaming=true;stream_start=esp_timer_get_time();info();status();motor_status();}
          else if(!strcmp(line,"X")){motor_off("host_stop");streaming=false;info();status();motor_status();}
          else motor_command(line);
        }
        used=0;overflow=false;first_byte=0;
      } else {
        if(!first_byte)first_byte=esp_timer_get_time();
        if(cmd<32 || cmd>126 || used>=sizeof(line)-1)overflow=true;
        else if(!overflow)line[used++]=(char)cmd;
      }
    }
    int64_t now=esp_timer_get_time();
    if(first_byte && now-first_byte>250000){motor_off("partial_timeout");used=0;overflow=false;first_byte=0;motor_reply(0,"partial_timeout",0);}
    if(streaming && now-stream_start>600000000){motor_off("session_expired");streaming=false;info();}
    drain();
    if(now-last>=(streaming?20000:1000000)){status();motor_status();last=now;}
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
