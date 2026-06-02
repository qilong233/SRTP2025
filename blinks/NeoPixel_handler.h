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
  void setMode(bool blinkMode);
  void setBlinkInterval(int interval);
  
private:
  Adafruit_NeoPixel strip;
  bool isBlinkMode;
  int blinkInterval;
  unsigned long lastUpdateTime;
  unsigned long lastBlinkTime;
  bool blinkState;
  
  void normalEffect();
  void blinkEffect();
};

#endif