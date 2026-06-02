#include <Adafruit_NeoPixel.h>

#define LED_PIN     6     // 数据线连接的Arduino引脚
#define LED_COUNT   60    // 灯珠数量

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// 定义绿色系颜色
const uint32_t LIGHT_GREEN = strip.Color(150, 255, 150);  // 浅绿
const uint32_t GREEN = strip.Color(0, 255, 0);           // 标准绿
const uint32_t DARK_GREEN = strip.Color(0, 50, 0);       // 墨绿

void setup() {
  strip.begin();
  strip.setBrightness(50);  // 建议亮度50-100
  strip.show();             // 初始化灯条
}

void loop() {
  greenWaveEffect(30);      // 调用绿色波形效果（30ms延迟）
}

// 绿色波形流动效果
void greenWaveEffect(int delayTime) {
  static float offset = 0.0;  // 动态偏移量
  
  for (int i = 0; i < LED_COUNT; i++) {
    // 计算每个灯珠的位置比例 (0.0~1.0)
    float position = fmod((float)i / LED_COUNT + offset, 1.0);
    
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
  offset += 0.02;  // 调整速度（值越大越快）
  if (offset > 1.0) offset = 0;
  delay(delayTime);
}

// 颜色混合函数（线性插值）
uint32_t blendColors(uint32_t color1, uint32_t color2, float ratio) {
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