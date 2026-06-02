#include "NeoPixel_handler.h"
#include <Arduino.h>

NeoPixelHandler::NeoPixelHandler() 
  : strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800), 
    speedFactor(1.0), 
    direction(1), // 默认正向
    lastUpdateTime(0),
    waveOffset(0.0) {
}

void NeoPixelHandler::init() {
  strip.begin();
  strip.setBrightness(50);  // 建议亮度50-100
  strip.show();             // 初始化灯条
}

void NeoPixelHandler::update() {
  // 根据速度因子计算更新间隔
  unsigned long updateInterval = 30 / speedFactor; // 基础30ms延迟除以速度因子
  
  if (millis() - lastUpdateTime >= updateInterval) {
    greenWaveEffect();
    lastUpdateTime = millis();
  }
}

void NeoPixelHandler::setSpeedFactor(float factor) {
  // 限制速度因子在合理范围内
  speedFactor = constrain(factor, 1.0, 3.0);
}

float NeoPixelHandler::getSpeedFactor() const {
  return speedFactor;
}

// 设置灯条流动方向
void NeoPixelHandler::setDirection(int dir) {
  direction = (dir >= 0) ? 1 : -1; // 确保方向为1或-1
}

void NeoPixelHandler::greenWaveEffect() {
  for (int i = 0; i < LED_COUNT; i++) {
    // 计算每个灯珠的位置比例 (0.0~1.0)
    float position = fmod((float)i / LED_COUNT + waveOffset, 1.0);
    if (position < 0) position += 1.0; // 确保位置在0-1范围内
    
    // 根据位置选择颜色
    uint32_t color;
    if (position < 0.142) {          // 0-1/7 浅绿→绿
      color = blendColors(LIGHT_GREEN, GREEN, position * 7);
    } else if (position < 0.285) {   // 1/7-2/7 保持绿
      color = GREEN;
    } else if (position < 0.428) {   // 2/7-3/7 绿→墨绿
      color = blendColors(GREEN, DARK_GREEN, (position - 0.285) * 7);
    } else if (position < 0.571) {   // 3/7-4/7 墨绿→绿
      color = blendColors(DARK_GREEN, GREEN, (position - 0.428) * 7);
    } else if (position < 0.714) {   // 4/7-5/7 保持绿
      color = GREEN;
    } else {                         // 5/7-1.0 绿→浅绿
      color = blendColors(GREEN, LIGHT_GREEN, (position - 0.714) * 7);
    }
    
    strip.setPixelColor(i, color);
  }
  
  strip.show();
  
  // 根据方向调整波形偏移量
  // 减速时使用较小的速度因子，使反向流动速度变慢
  float effectiveSpeed = speedFactor;
  if (direction < 0) {
    effectiveSpeed = speedFactor * REVERSE_SLOW_FACTOR;
  }
  
  waveOffset += 0.02 * effectiveSpeed * direction;  // 方向影响波形移动方向
  
  // 确保偏移量在0-1范围内
  if (waveOffset > 1.0) waveOffset -= 1.0;
  if (waveOffset < 0.0) waveOffset += 1.0;
}

uint32_t NeoPixelHandler::blendColors(uint32_t color1, uint32_t color2, float ratio) {
  if (ratio > 1.0) ratio = 1.0;
  
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;
  
  uint8_t r = r1 + (r2 - r1) * ratio;
  uint8_t g = g1 + (g2 - g1) * ratio;
  uint8_t b = b1 + (b2 - b1) * ratio;
  
  return strip.Color(r, g, b);
}