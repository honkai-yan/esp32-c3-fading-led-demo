/**
 * ESP32-C3 入门案例 - 板载LED渐变闪烁程序，基于 Arduino ESP32-C3。
 * 使用状态机模式实现LED渐变闪烁，具有高优先级的中断响应。
 *
 * Author: Peter Adams
 * Date: 2025-07-30
 * Board: ESP32-C3-DevKitM-1/ESP32-C3 Super Mini
 * IDE: VSCode + PlatformIO
 */

#include <Arduino.h>
#include <driver/ledc.h>

// 板载LED引脚号
#define led_pin 8
// boot 按钮引脚号
#define boot_btn_pin 9
// 渐变时间
#define fade_in_time 2000
#define fade_out_time 2000
// 渐变间隔
#define fade_interval 500
// ledc 通道 esp32-c3 共有 6 个通道，默认为 0
#define ledc_channel LEDC_CHANNEL_0
// ledc 的 pwm 时钟频率
#define ledc_pwm_freq 5000
// ledc 的 pwm 时钟分辨率
#define ledc_pwm_resolution LEDC_TIMER_12_BIT

// ledc 起始 duty
#define ledc_start_duty 0
// ledc 目标 duty
#define ledc_target_duty 4095

// ledc 状态
enum class LED_STATE
{
  OFF,
  START,
  FADE_IN,
  FADE_OUT,
  FADE_INTERVAL,
  INTERRUPTED,
};

/*
led渐变函数
案例使用的 Arduino Core 版本为 2.x，只能调用 esp32 idf 的底层 api 来操作渐变
3.x 的 Arduino Core Api 可以直接使用 ledcFade 等高级函数
案例既使用了 Arduino 封装的一些高级函数，也使用了 esp32 idf 的底层 api
相应文档：https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
*/
void ledc_fade(uint8_t channel, uint32_t target_duty, uint32_t duration)
{
  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, target_duty, duration);
  ledc_fade_start(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, LEDC_FADE_NO_WAIT);
}

// 缓存中断前的状态
volatile LED_STATE last_state = LED_STATE::START;
// 当前状态
volatile LED_STATE led_state = LED_STATE::START;

// 中断处理函数
void IRAM_ATTR interrupt_handler()
{
  last_state = led_state;
  led_state = LED_STATE::INTERRUPTED;
}

void setup()
{
  Serial.begin(9600);
  // 配置 ledc，Arduino 会自动配置时钟和计时器
  ledcSetup(ledc_channel, ledc_pwm_freq, ledc_pwm_resolution);
  // 将 ledc 通道绑定到指定引脚，Arduino 会自动配置通道
  ledcAttachPin(led_pin, ledc_channel);
  // 装载渐变函数
  ledc_fade_func_install(0);
  // led 初始状态（熄灭）
  ledcWrite(ledc_channel, ledc_target_duty);

  // 配置 boot 引脚为 input，内部上拉
  pinMode(boot_btn_pin, INPUT_PULLUP);
  // 绑定中断处理函数
  attachInterrupt(digitalPinToInterrupt(boot_btn_pin), interrupt_handler, FALLING);

  delay(1000);
  Serial.println("System ready.");
}

void loop()
{
  // 上次执行某操作的时间
  static uint32_t last_time = 0;

  // 通过中断来控制 led 开关状态
  if (led_state == LED_STATE::INTERRUPTED)
  {
    Serial.println("detected interrupt");
    // 如果中断前 led 渐变正在运行，则将其关闭，否则将其打开
    if (last_state == LED_STATE::OFF)
    {
      Serial.println("led start");
      led_state = LED_STATE::START;
    }
    else
    {
      Serial.println("led off");
      led_state = LED_STATE::OFF;
    }
  }

  // 状态机
  switch (led_state)
  {
  // 关闭状态，确保 led 渐变被关闭且保持熄灭
  case LED_STATE::OFF:
    ledc_stop(LEDC_LOW_SPEED_MODE, ledc_channel, 1);
    ledcWrite(ledc_channel, ledc_target_duty);
    break;
  // 中断状态，由于中断优先级最高，故处理逻辑放在了上面
  case LED_STATE::INTERRUPTED:
    break;
  // 开启状态，初始化 led 渐变，并跳转到 fade_in 状态
  case LED_STATE::START:
    ledc_fade(ledc_channel, ledc_start_duty, fade_in_time);
    led_state = LED_STATE::FADE_IN;
    last_time = millis();
    Serial.println("fade in");
    break;
  // fade_in 状态，间隔时间后跳转到 fade_out 状态
  case LED_STATE::FADE_IN:
    if (millis() - last_time >= fade_in_time)
    {
      // to fade out
      Serial.println("fade out");
      led_state = LED_STATE::FADE_OUT;
      ledc_fade(ledc_channel, ledc_target_duty, fade_out_time);
      last_time = millis();
    }
    break;
  // fade_out 状态，间隔时间后跳转到 fade_interval 状态
  case LED_STATE::FADE_OUT:
    if (millis() - last_time >= fade_out_time)
    {
      // to fade interval
      Serial.println("fade intterval");
      led_state = LED_STATE::FADE_INTERVAL;
      last_time = millis();
    }
    break;
  // fade_interval 状态，间隔时间后跳转到 fade_in 状态
  case LED_STATE::FADE_INTERVAL:
    ledcWrite(ledc_channel, ledc_target_duty);
    if (millis() - last_time >= fade_interval)
    {
      // to fade in
      Serial.println("fade in");
      led_state = LED_STATE::FADE_IN;
      ledc_fade(ledc_channel, ledc_start_duty, fade_in_time);
      last_time = millis();
    }
    break;
  // 其它状态为非法状态，保持 led 关闭
  default:
    led_state = LED_STATE::OFF;
    break;
  }

  // 延时 33ms，减小占用率（约等于 30帧/s）
  delay(33);
}