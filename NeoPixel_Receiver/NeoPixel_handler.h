#ifndef NEOPIXEL_HANDLER_H
#define NEOPIXEL_HANDLER_H

#include <Adafruit_NeoPixel.h>

#define LED_PIN     6     // 数据线连接的Arduino引脚
#define LED_COUNT   60    // 灯珠数量

class NeoPixelHandler {
public:
  NeoPixelHandler();
  void init();
  void update();
  void setSpeedFactor(float factor);
  float getSpeedFactor() const;
  void setDirection(int dir); // 设置方向
  
private:
  Adafruit_NeoPixel strip;
  float speedFactor;
  int direction; // 流动方向 (1: 正向, -1: 反向)
  unsigned long lastUpdateTime;
  float waveOffset;
  
  // 绿色系颜色
  const uint32_t LIGHT_GREEN = strip.Color(150, 255, 150);
  const uint32_t GREEN = strip.Color(0, 255, 0);
  const uint32_t DARK_GREEN = strip.Color(0, 50, 0);
  
  // 可配置参数 - 反向流动速度降低因子
  const float REVERSE_SLOW_FACTOR = 0.7; // 反向流动时速度降低因子
  
  void greenWaveEffect();
  uint32_t blendColors(uint32_t color1, uint32_t color2, float ratio);
};

#endif