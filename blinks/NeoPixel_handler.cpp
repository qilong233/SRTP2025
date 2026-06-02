#include "NeoPixel_handler.h"
#include <Arduino.h>

NeoPixelHandler::NeoPixelHandler() 
  : strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800), 
    isBlinkMode(false),
    blinkInterval(500),
    lastUpdateTime(0),
    lastBlinkTime(0),
    blinkState(false) {
}

void NeoPixelHandler::init() {
  strip.begin();
  strip.setBrightness(50);  // 整体亮度设置
  normalEffect();           // 初始化为常态效果
}

void NeoPixelHandler::update() {
  if (isBlinkMode) {
    // 闪烁模式
    if (millis() - lastBlinkTime >= blinkInterval) {
      blinkEffect();
      lastBlinkTime = millis();
    }
  } else {
    // 常态模式：不需要频繁更新，只需要设置一次
    // 但为了确保状态正确，每隔一段时间检查一次
    if (millis() - lastUpdateTime >= 1000) { // 每秒检查一次
      normalEffect();
      lastUpdateTime = millis();
    }
  }
}

void NeoPixelHandler::setMode(bool blinkMode) {
  if (isBlinkMode != blinkMode) {
    isBlinkMode = blinkMode;
    
    // 模式切换时立即更新显示
    if (isBlinkMode) {
      // 切换到闪烁模式，重置闪烁状态
      blinkState = false;
      lastBlinkTime = millis();
    } else {
      // 切换到常态模式
      normalEffect();
    }
  }
}

void NeoPixelHandler::setBlinkInterval(int interval) {
  // 确保闪烁间隔在合理范围内
  blinkInterval = constrain(interval, 50, 1000);
}

void NeoPixelHandler::normalEffect() {
  // 常态效果：所有灯珠显示单一颜色，低亮度
  uint32_t dimGreen = strip.Color(0, 80, 0); // 低亮度绿色
  
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, dimGreen);
  }
  
  strip.show();
}

void NeoPixelHandler::blinkEffect() {
  // 切换闪烁状态
  blinkState = !blinkState;
  
  if (blinkState) {
    // 亮起状态：所有灯珠亮绿色（较高亮度）
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(0, 255, 0)); // 绿色
    }
  } else {
    // 熄灭状态：所有灯珠熄灭
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 0)); // 熄灭
    }
  }
  
  strip.show();
}