// 격리된 ST 버스 서보 시험. EEPROM/ID/baud/영점 쓰기와 구동모터 PWM 금지.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool armed;
static int servo_id = -1, centre, speed_limit;
static int64_t lease_end, hard_end;
static uint32_t boot_id, sequence;

static void host(const char *s) { uart_write_bytes(UART_NUM_0, s, strlen(s)); }
static int xfer(const uint8_t *tx, int n, uint8_t *rx, int cap) {
  uart_flush_input(UART_NUM_1);
  if (uart_write_bytes(UART_NUM_1, tx, n) != n) return -1;
  if (uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(30)) != ESP_OK) return -1;
  int used = 0;
  int64_t end = esp_timer_get_time() + 30000;
  while (esp_timer_get_time() < end && used < cap) {
    int k = uart_read_bytes(UART_NUM_1, rx + used, cap - used, pdMS_TO_TICKS(1));
    if (k > 0) used += k;
  }
  return used;
}
static bool read_reg(int id, int address, int count, uint8_t *out) {
  uint8_t tx[8] = {255,255,id,4,2,address,count,0}, rx[128];
  tx[7] = (uint8_t)~(id+4+2+address+count);
  int n = xfer(tx,8,rx,sizeof(rx));
  for (int i=0; i+count+6<=n; ++i) {
    if (rx[i]!=255 || rx[i+1]!=255 || rx[i+2]!=id ||
        rx[i+3]!=count+2 || rx[i+4]!=0) continue;
    unsigned sum=0; for (int j=2;j<count+6;++j) sum+=rx[i+j];
    if ((sum&255)!=255) continue;
    memcpy(out,rx+i+5,count); return true;
  }
  return false;
}
static void disarm(void) {
  if (armed && servo_id>=0) {
    uint8_t tx[8]={255,255,servo_id,4,3,40,0,0},rx[32];
    tx[7]=(uint8_t)~(servo_id+4+3+40);
    xfer(tx,8,rx,sizeof(rx));
  }
  armed=false;
}
static bool allowed(const uint8_t *p, int n) {
  if (n<6 || p[0]!=255 || p[1]!=255 || p[2]>253 || n!=p[3]+4) return false;
  unsigned sum=0; for(int i=2;i<n;++i)sum+=p[i];
  if ((sum&255)!=255) return false;
  if (p[4]==1) return n==6;
  if (p[4]==2) return n==8 && p[6]>0 && p[6]<=64 && p[5]+p[6]<=73;
  if (p[4]!=3) return false;
  if (n==8 && p[5]==40 && p[6]==0) return true;
  if (!armed || p[2]!=servo_id || esp_timer_get_time()>lease_end ||
      esp_timer_get_time()>hard_end) return false;
  if (n==8 && p[5]==40 && p[6]==1) return true;
  if (n==9 && p[5]==48) return (p[6]|(p[7]<<8))<=300;
  if (n==14 && p[5]==41) {
    int pos=p[7]|(p[8]<<8),speed=p[11]|(p[12]<<8);
    return p[6]>=5 && p[6]<=20 && pos<=4095 && abs(pos-centre)<=114 &&
           p[9]==0 && p[10]==0 && speed>0 && speed<=speed_limit;
  }
  return false;
}
static void command(char *line) {
  char msg[600]; unsigned baud,id,c,s; char extra;
  if (!strcmp(line,"INFO")) {
    snprintf(msg,sizeof(msg),"{\"event\":\"info\",\"firmware\":\"servo_probe_v1\",\"boot\":%lu,\"seq\":%lu,\"tx_gpio\":17,\"rx_gpio\":18,\"motor_pwm\":%d,\"motor_dir\":%d,\"armed\":%s}\n",
      (unsigned long)boot_id,(unsigned long)sequence++,gpio_get_level(15),gpio_get_level(6),armed?"true":"false");
    host(msg); return;
  }
  if (!strcmp(line,"X")) { disarm(); host("{\"event\":\"disarmed\"}\n");return; }
  if (sscanf(line,"B %u %c",&baud,&extra)==1) {
    const unsigned rates[]={1000000,500000,250000,128000,115200,76800,57600,38400};
    bool valid=false; for(unsigned i=0;i<sizeof(rates)/sizeof(rates[0]);++i)valid|=baud==rates[i];
    if (valid && !armed && uart_set_baudrate(UART_NUM_1,baud)==ESP_OK) {
      snprintf(msg,sizeof(msg),"{\"event\":\"baud\",\"baud\":%u}\n",baud);host(msg);return;
    }
  }
  if (sscanf(line,"ARM %u %u %u %c",&id,&c,&s,&extra)==3 &&
      !armed && id<=253 && c<=4095 && s>0 && s<=400) {
    uint8_t pos[2],mode=255;
    if (read_reg(id,56,2,pos) && read_reg(id,33,1,&mode) && mode==0 &&
        abs((pos[0]|(pos[1]<<8))-(int)c)<=3) {
      servo_id=id;centre=c;speed_limit=s;armed=true;
      lease_end=esp_timer_get_time()+1000000;hard_end=esp_timer_get_time()+60000000;
      host("{\"event\":\"armed\",\"lease_ms\":1000,\"max_span_counts\":114,\"hard_seconds\":60}\n");return;
    }
  }
  if (line[0]=='Q' && line[1]==' ') {
    uint8_t tx[80],rx[160];int n=0;char *h=line+2;bool valid=true;
    size_t len=strlen(h);
    if (len%2 || len<12 || len>sizeof(tx)*2) valid=false;
    while(valid && *h) {
      char b[3]={h[0],h[1],0},*end;unsigned v=strtoul(b,&end,16);
      if(*end || v>255){valid=false;break;} tx[n++]=v;h+=2;
    }
    if(valid && allowed(tx,n)) {
      int64_t begin=esp_timer_get_time();int k=xfer(tx,n,rx,sizeof(rx));
      int64_t end=esp_timer_get_time();
      if(armed)lease_end=end+1000000;
      char hex[321];for(int i=0;i<k;++i)snprintf(hex+2*i,3,"%02x",rx[i]);
      hex[k>0?2*k:0]=0;
      snprintf(msg,sizeof(msg),"{\"event\":\"reply\",\"boot\":%lu,\"seq\":%lu,\"begin_us\":%lld,\"end_us\":%lld,\"rx\":\"%s\",\"io_error\":%s}\n",
        (unsigned long)boot_id,(unsigned long)sequence++,(long long)begin,(long long)end,hex,k<0?"true":"false");
      host(msg);return;
    }
  }
  host("{\"event\":\"rejected\"}\n");
}
void app_main(void) {
  // LOW를 먼저 래치한 뒤 출력화. ROM/리셋 구간의 실제 파형 보장은 별도다.
  gpio_set_level(15,0);gpio_set_level(6,0);
  gpio_config_t outputs={.pin_bit_mask=(1ULL<<15)|(1ULL<<6),.mode=GPIO_MODE_INPUT_OUTPUT};
  ESP_ERROR_CHECK(gpio_config(&outputs));
  uart_config_t host_cfg={.baud_rate=921600,.data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,2048,4096,0,NULL,0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&host_cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,43,44,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
  host_cfg.baud_rate=1000000;
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1,2048,2048,0,NULL,0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1,&host_cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,17,18,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
  boot_id=esp_random();
  char line[200];int used=0;bool discard=false;
  for(;;) {
    if(armed && (esp_timer_get_time()>lease_end || esp_timer_get_time()>hard_end)) {
      disarm();host("{\"event\":\"lease_expired\"}\n");
    }
    uint8_t b;int n=uart_read_bytes(UART_NUM_0,&b,1,pdMS_TO_TICKS(10));
    if(n!=1)continue;
    if(b=='\n') { if(!discard){line[used]=0;command(line);}used=0;discard=false; }
    else if(b!='\r') {if(used<(int)sizeof(line)-1 && !discard)line[used++]=b;else discard=true;}
  }
}
