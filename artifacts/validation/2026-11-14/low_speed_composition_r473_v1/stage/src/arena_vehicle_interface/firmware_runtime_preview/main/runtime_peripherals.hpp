#pragma once
// 확정 GPIO의 기존 preview 초기화/비차단 UART 보조. 운영 후보와 동일하게 사용한다.
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
static QueueHandle_t serial(uart_port_t port,int baud,int txpin,int rxpin){
  QueueHandle_t events=nullptr;
  uart_config_t cfg{};cfg.baud_rate=baud;cfg.data_bits=UART_DATA_8_BITS;
  cfg.parity=UART_PARITY_DISABLE;cfg.stop_bits=UART_STOP_BITS_1;
  cfg.flow_ctrl=UART_HW_FLOWCTRL_DISABLE;cfg.source_clk=UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(port,4096,0,32,&events,0));
  ESP_ERROR_CHECK(uart_param_config(port,&cfg));
  ESP_ERROR_CHECK(uart_set_pin(port,txpin,rxpin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
  return events;
}
static bool events_ok(QueueHandle_t queue){
  uart_event_t event{};
  for(int i=0;i<32 && xQueueReceive(queue,&event,0)==pdTRUE;++i)
    if(event.type!=UART_DATA)return false;
  return uxQueueMessagesWaiting(queue)<32;
}

static pcnt_unit_handle_t encoder(){
  gpio_config_t gpio{};gpio.pin_bit_mask=(1ULL<<4)|(1ULL<<5);gpio.mode=GPIO_MODE_INPUT;
  gpio.pull_up_en=GPIO_PULLUP_DISABLE;gpio.pull_down_en=GPIO_PULLDOWN_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&gpio));
  pcnt_unit_handle_t unit=nullptr;pcnt_unit_config_t cfg{};
  cfg.high_limit=30000;cfg.low_limit=-30000;cfg.flags.accum_count=true;
  ESP_ERROR_CHECK(pcnt_new_unit(&cfg,&unit));
  pcnt_glitch_filter_config_t filter{};filter.max_glitch_ns=1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit,&filter));
  pcnt_channel_handle_t a=nullptr,b=nullptr;
  pcnt_chan_config_t ca{};ca.edge_gpio_num=4;ca.level_gpio_num=5;
  pcnt_chan_config_t cb{};cb.edge_gpio_num=5;cb.level_gpio_num=4;
  ESP_ERROR_CHECK(pcnt_new_channel(unit,&ca,&a));
  ESP_ERROR_CHECK(pcnt_new_channel(unit,&cb,&b));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a,PCNT_CHANNEL_EDGE_ACTION_DECREASE,PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(a,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b,PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(b,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit,30000));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit,-30000));
  ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_4,GPIO_FLOATING));
  ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_5,GPIO_FLOATING));
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));return unit;
}

static void pwm_zero(){
  // MD20A PWM LOW 단락 제동 요청. ROM/리셋 구간 또는 실제 제동 성능 보증 아님.
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_15,0));ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_6,0));
  gpio_config_t pins{};pins.pin_bit_mask=(1ULL<<15)|(1ULL<<6);pins.mode=GPIO_MODE_INPUT_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&pins));
  ledc_timer_config_t timer{};timer.speed_mode=LEDC_LOW_SPEED_MODE;timer.timer_num=LEDC_TIMER_0;
  timer.duty_resolution=LEDC_TIMER_10_BIT;timer.freq_hz=20000;timer.clk_cfg=LEDC_USE_XTAL_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
  ledc_channel_config_t ch{};ch.gpio_num=15;ch.speed_mode=LEDC_LOW_SPEED_MODE;
  ch.channel=LEDC_CHANNEL_0;ch.timer_sel=LEDC_TIMER_0;ch.duty=0;
  ESP_ERROR_CHECK(ledc_channel_config(&ch));
  ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0));
}

