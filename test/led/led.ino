#include <Adafruit_NeoPixel.h>

#define LED_PIN     6     // 数据线连接的Arduino引脚
#define LED_COUNT   60    // 灯珠数量

// 创建NeoPixel对象
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();          // 初始化灯条
  strip.setBrightness(50); // 设置亮度（0-255，建议不超过100以免过流）
  strip.show();           // 关闭所有灯珠
}

void loop() {
  waveFlowEffect(20);     // 调用波形流动函数（参数为延迟毫秒数）
}

// RGB波形流动效果
void waveFlowEffect(int delayTime) {
  static float offset = 0.0; // 动态偏移量

  for (int i = 0; i < LED_COUNT; i++) {
    // 计算每个灯珠的位置比例 (0.0~1.0)
    float position = (float)i / LED_COUNT;
    
    // 使用正弦函数生成RGB波形（相位差120度）
    float r = sin(2 * PI * (position + offset)) * 127 + 128;
    float g = sin(2 * PI * (position + offset) + 2.0 * PI / 3.0) * 127 + 128;
    float b = sin(2 * PI * (position + offset) + 4.0 * PI / 3.0) * 127 + 128;
    
    // 设置灯珠颜色
    strip.setPixelColor(i, (int)r, (int)g, (int)b);
  }
  
  strip.show();            // 更新灯条显示
  offset += 0.05;          // 调整偏移量控制速度（值越大越快）
  if (offset > 1.0) offset = 0;
  delay(delayTime);        // 控制动画速度
}