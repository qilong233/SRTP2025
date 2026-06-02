#ifndef NEOPIXEL_HANDLER_H
#define NEOPIXEL_HANDLER_H

#include <Adafruit_NeoPixel.h>

#define LED_PIN      6      // 数据线连接的Arduino引脚
#define LED_COUNT    60     // 灯珠数量

class NeoPixelHandler {
public:
  NeoPixelHandler();
  void init();
  void update();
  void setSpeedFactor(float factor);
  float getSpeedFactor() const;
  void setDirection(int dir); // 设置方向
  void setRedFactor(float factor); // 新增：设置变红比例
  
private:
  Adafruit_NeoPixel strip;
  float speedFactor;
  int direction; // 流动方向 (1: 正向, -1: 反向)
  float currentRedFactor; // 0.0 = 纯绿, 1.0 = 纯红
  
  unsigned long lastUpdateTime;
  float waveOffset;
  
  // 绿色系颜色
  const uint32_t LIGHT_GREEN = strip.Color(150, 255, 150);
  const uint32_t GREEN = strip.Color(0, 255, 0);
  const uint32_t DARK_GREEN = strip.Color(0, 50, 0);

  // 新增：红色系颜色 (对应绿色系的亮度层级)
  const uint32_t LIGHT_RED = strip.Color(255, 50, 50);
  const uint32_t RED = strip.Color(255, 0, 0);
  const uint32_t DARK_RED = strip.Color(50, 0, 0);
  
  // 可配置参数 - 反向流动速度降低因子
  const float REVERSE_SLOW_FACTOR = 0.7; // 反向流动时速度降低因子
  
  void waveEffect(); // 重命名原 greenWaveEffect
  uint32_t blendColors(uint32_t color1, uint32_t color2, float ratio);
  uint32_t getWaveColor(float position, bool useRedPalette); // 辅助函数：根据位置获取波浪颜色
};

#endif