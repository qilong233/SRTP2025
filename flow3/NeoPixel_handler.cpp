#include "NeoPixel_handler.h"
#include <Arduino.h>

NeoPixelHandler::NeoPixelHandler() 
  : strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800), 
    speedFactor(1.0), 
    direction(1), 
    currentRedFactor(0.0), // 初始化为全绿
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
    waveEffect();
    lastUpdateTime = millis();
  }
}

void NeoPixelHandler::setSpeedFactor(float factor) {
  speedFactor = constrain(factor, 1.0, 3.0);
}

float NeoPixelHandler::getSpeedFactor() const {
  return speedFactor;
}

void NeoPixelHandler::setDirection(int dir) {
  direction = (dir >= 0) ? 1 : -1;
}

// 新增：设置红色混合比例
void NeoPixelHandler::setRedFactor(float factor) {
  currentRedFactor = constrain(factor, 0.0, 1.0);
}

// 辅助函数：获取单色系的波浪颜色
uint32_t NeoPixelHandler::getWaveColor(float position, bool useRedPalette) {
    uint32_t c_light = useRedPalette ? LIGHT_RED : LIGHT_GREEN;
    uint32_t c_mid   = useRedPalette ? RED : GREEN;
    uint32_t c_dark  = useRedPalette ? DARK_RED : DARK_GREEN;

    if (position < 0.142) {          // 0-1/7 浅→中
      return blendColors(c_light, c_mid, position * 7);
    } else if (position < 0.285) {   // 1/7-2/7 保持中
      return c_mid;
    } else if (position < 0.428) {   // 2/7-3/7 中→深
      return blendColors(c_mid, c_dark, (position - 0.285) * 7);
    } else if (position < 0.571) {   // 3/7-4/7 深→中
      return blendColors(c_dark, c_mid, (position - 0.428) * 7);
    } else if (position < 0.714) {   // 4/7-5/7 保持中
      return c_mid;
    } else {                         // 5/7-1.0 中→浅
      return blendColors(c_mid, c_light, (position - 0.714) * 7);
    }
}

void NeoPixelHandler::waveEffect() {
  for (int i = 0; i < LED_COUNT; i++) {
    // 计算每个灯珠的位置比例 (0.0~1.0)
    float position = fmod((float)i / LED_COUNT + waveOffset, 1.0);
    if (position < 0) position += 1.0; 
    
    // 1. 计算原本的绿色流光颜色
    uint32_t greenBase = getWaveColor(position, false);
    
    // 2. 如果需要变红，计算目标红色流光颜色
    uint32_t finalColor = greenBase;
    
    if (currentRedFactor > 0.01) {
        uint32_t redBase = getWaveColor(position, true);
        // 3. 将绿色和红色根据比例混合
        finalColor = blendColors(greenBase, redBase, currentRedFactor);
    }
    
    strip.setPixelColor(i, finalColor);
  }
  
  strip.show();
  
  // 根据方向调整波形偏移量
  float effectiveSpeed = speedFactor;
  if (direction < 0) {
    effectiveSpeed = speedFactor * REVERSE_SLOW_FACTOR;
  }
  
  waveOffset += 0.02 * effectiveSpeed * direction;
  
  if (waveOffset > 1.0) waveOffset -= 1.0;
  if (waveOffset < 0.0) waveOffset += 1.0;
}

uint32_t NeoPixelHandler::blendColors(uint32_t color1, uint32_t color2, float ratio) {
  if (ratio > 1.0) ratio = 1.0;
  if (ratio < 0.0) ratio = 0.0;
  
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;
  
  uint8_t r = r1 + (uint8_t)((float)(r2 - r1) * ratio);
  uint8_t g = g1 + (uint8_t)((float)(g2 - g1) * ratio);
  uint8_t b = b1 + (uint8_t)((float)(b2 - b1) * ratio);
  
  return strip.Color(r, g, b);
}