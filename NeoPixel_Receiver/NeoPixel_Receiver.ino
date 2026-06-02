#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

// 创建对象
RF24 radio(9, 10); // CE, CSN
Adafruit_NeoPixel strip(60, 6, NEO_GRB + NEO_KHZ800);

// 无线通信地址
const byte address[6] = "00001";

// 数据结构（必须与发射端相同）
struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temperature;
  unsigned long timestamp;
};

SensorData receivedData;

// 状态变量
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT = 1000; // 连接超时(ms)
bool dataReceived = false;
int receiveCount = 0;
int lostPackets = 0;
unsigned long lastPacketTime = 0;

// ===== 可配置参数 =====
const float ACCEL_THRESHOLD_LOW = 1.5;
const float ACCEL_THRESHOLD_HIGH = 5;
const float DECEL_THRESHOLD_LOW = -1.5;
const float DECEL_THRESHOLD_HIGH = -5;
const float MIN_SPEED_FACTOR = 1.0;
const float MAX_SPEED_FACTOR = 3.0;
const float REVERSE_SLOW_FACTOR = 0.7;

// 灯条控制变量
float speedFactor = 1.0;
int direction = 1;
unsigned long lastUpdateTime = 0;
float waveOffset = 0.0;

// 绿色系颜色
const uint32_t LIGHT_GREEN = strip.Color(150, 255, 150);
const uint32_t GREEN = strip.Color(0, 255, 0);
const uint32_t DARK_GREEN = strip.Color(0, 50, 0);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // 初始化无线模块
  if (!radio.begin()) {
    Serial.println("nRF24L01+ initialization failed!");
    while (1);
  }
  
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  
  // 初始化灯条
  strip.begin();
  strip.setBrightness(50);
  strip.show();
  
  Serial.println("=== Wireless LED Controller Receiver ===");
  Serial.println("Acceleration (m/s²)\t\tGyroscope (deg/s)");
  Serial.println("X\t\tY\t\tZ\t\tX\t\tY\t\tZ\tTemp(°C)\tSpeed\tDirection\tPackets");
  
  // 显示配置参数
  Serial.println("=== 当前配置参数 ===");
  Serial.print("加速度阈值: "); Serial.print(ACCEL_THRESHOLD_LOW); 
  Serial.print(" 到 "); Serial.println(ACCEL_THRESHOLD_HIGH);
  Serial.print("减速度阈值: "); Serial.print(DECEL_THRESHOLD_LOW); 
  Serial.print(" 到 "); Serial.println(DECEL_THRESHOLD_HIGH);
  Serial.print("速度因子范围: "); Serial.print(MIN_SPEED_FACTOR); 
  Serial.print(" 到 "); Serial.println(MAX_SPEED_FACTOR);
  Serial.println("====================");
}

void loop() {
  // 接收无线数据
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    lastReceiveTime = millis();
    
    if (!dataReceived) {
      dataReceived = true;
      Serial.println(">>> Wireless connection established! <<<");
    }
    
    receiveCount++;
    
    // 计算数据包间隔
    unsigned long currentTime = millis();
    if (lastPacketTime > 0) {
      unsigned long interval = currentTime - lastPacketTime;
      if (interval > 200) { // 如果数据包间隔超过200ms，认为有丢包
        lostPackets++;
      }
    }
    lastPacketTime = currentTime;
    
    // 处理接收到的传感器数据
    processSensorData();
    
    // 串口输出
    printSensorData();
  }
  
  // 检查连接状态
  if (millis() - lastReceiveTime > TIMEOUT && dataReceived) {
    Serial.println(">>> Wireless connection lost! <<<");
    dataReceived = false;
    // 连接丢失时重置为默认状态
    speedFactor = MIN_SPEED_FACTOR;
    direction = 1;
  }
  
  // 更新灯条效果
  updateLEDStrip();
}

void processSensorData() {
  // 根据X轴加速度调整灯条速度和方向
  speedFactor = MIN_SPEED_FACTOR;
  direction = 1;
  
  if (receivedData.accelX > ACCEL_THRESHOLD_LOW) {
    // 加速状态：正向流动，速度增加
    float accelRange = ACCEL_THRESHOLD_HIGH - ACCEL_THRESHOLD_LOW;
    float speedRange = MAX_SPEED_FACTOR - MIN_SPEED_FACTOR;
    float normalizedAccel = min(receivedData.accelX - ACCEL_THRESHOLD_LOW, accelRange);
    speedFactor = MIN_SPEED_FACTOR + (normalizedAccel / accelRange) * speedRange;
    direction = 1;
  } else if (receivedData.accelX < DECEL_THRESHOLD_LOW) {
    // 减速状态：反向流动，速度增加
    float decelRange = abs(DECEL_THRESHOLD_HIGH - DECEL_THRESHOLD_LOW);
    float speedRange = MAX_SPEED_FACTOR - MIN_SPEED_FACTOR;
    float normalizedDecel = min(abs(receivedData.accelX) - abs(DECEL_THRESHOLD_LOW), decelRange);
    speedFactor = MIN_SPEED_FACTOR + (normalizedDecel / decelRange) * speedRange;
    direction = -1;
  }
}

void printSensorData() {
  Serial.print(receivedData.accelX, 2); Serial.print("\t\t");
  Serial.print(receivedData.accelY, 2); Serial.print("\t\t");
  Serial.print(receivedData.accelZ, 2); Serial.print("\t\t");
  
  Serial.print(receivedData.gyroX, 1); Serial.print("\t\t");
  Serial.print(receivedData.gyroY, 1); Serial.print("\t\t");
  Serial.print(receivedData.gyroZ, 1); Serial.print("\t\t");
  
  Serial.print(receivedData.temperature, 1); Serial.print("\t\t");
  
  Serial.print(speedFactor, 2); Serial.print("\t");
  
  Serial.print(direction == 1 ? "Forward" : "Reverse"); Serial.print("\t\t");
  
  Serial.print(receiveCount);
  
  // 每100个数据包显示一次统计信息
  if (receiveCount % 100 == 0) {
    Serial.print(" (Lost: ");
    Serial.print(lostPackets);
    Serial.print(")");
  }
  
  Serial.println();
}

void updateLEDStrip() {
  // 根据速度因子计算更新间隔
  unsigned long updateInterval = 30 / speedFactor;
  
  if (millis() - lastUpdateTime >= updateInterval) {
    greenWaveEffect();
    lastUpdateTime = millis();
  }
}

void greenWaveEffect() {
  for (int i = 0; i < strip.numPixels(); i++) {
    // 计算每个灯珠的位置比例
    float position = fmod((float)i / strip.numPixels() + waveOffset, 1.0);
    if (position < 0) position += 1.0;
    
    // 根据位置选择颜色
    uint32_t color;
    if (position < 0.142) {
      color = blendColors(LIGHT_GREEN, GREEN, position * 7);
    } else if (position < 0.285) {
      color = GREEN;
    } else if (position < 0.428) {
      color = blendColors(GREEN, DARK_GREEN, (position - 0.285) * 7);
    } else if (position < 0.571) {
      color = blendColors(DARK_GREEN, GREEN, (position - 0.428) * 7);
    } else if (position < 0.714) {
      color = GREEN;
    } else {
      color = blendColors(GREEN, LIGHT_GREEN, (position - 0.714) * 7);
    }
    
    strip.setPixelColor(i, color);
  }
  
  strip.show();
  
  // 根据方向调整波形偏移量
  float effectiveSpeed = speedFactor;
  if (direction < 0) {
    effectiveSpeed = speedFactor * REVERSE_SLOW_FACTOR;
  }
  
  waveOffset += 0.02 * effectiveSpeed * direction;
  
  // 确保偏移量在0-1范围内
  if (waveOffset > 1.0) waveOffset -= 1.0;
  if (waveOffset < 0.0) waveOffset += 1.0;
}

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