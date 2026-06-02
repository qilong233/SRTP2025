#include "MPU6050_handler.h"
#include "NeoPixel_handler.h"

// 创建全局对象
MPU6050Handler mpuHandler;
NeoPixelHandler pixelHandler;

// 状态跟踪变量
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 150; // 读取间隔(ms)
int resetCount = 0;
const int MAX_RESETS = 5; // 最大复位次数

// ===== 可配置参数 - 加速度阈值和速度因子 =====
// 加速度阈值配置
const float ACCEL_THRESHOLD_LOW = 1.5;    // 开始加速的阈值 (m/s²)
const float ACCEL_THRESHOLD_HIGH = 5;  // 最大加速度阈值 (m/s²)

// 减速度阈值配置
const float DECEL_THRESHOLD_LOW = -1.5;   // 开始减速的阈值 (m/s²)
const float DECEL_THRESHOLD_HIGH = -5; // 最大减速度阈值 (m/s²)

// 速度因子配置
const float MIN_SPEED_FACTOR = 1.0;       // 最小速度因子
const float MAX_SPEED_FACTOR = 3.0;       // 最大速度因子
const float REVERSE_SLOW_FACTOR = 0.7;    // 反向流动速度降低因子

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // 初始化传感器和灯条
  if (!mpuHandler.init()) {
    Serial.println("MPU6050 initialization failed!");
    while(1); // 停止执行
  }
  pixelHandler.init();
  
  Serial.println("System Initialized with Red Fade Logic");
}

void loop() {
  // 检查是否到达读取间隔
  if (millis() - lastReadTime < READ_INTERVAL) {
    pixelHandler.update(); // 继续更新灯条
    return;
  }
  lastReadTime = millis();
  
  // 读取传感器数据
  SensorData data;
  bool readSuccess = mpuHandler.readData(data);
  
  // 检查数据有效性
  if (!readSuccess || 
      (data.accelX == 0 && data.accelY == 0 && data.accelZ == 0) ||
      (data.gyroX == 0 && data.gyroY == 0 && data.gyroZ == 0)) {
    
    Serial.print("Invalid data! Attempting recovery... ");
    resetCount++;
    
    if (resetCount > MAX_RESETS) {
      Serial.println("Critical failure! Please reset Arduino.");
      while(1); // 永久停止
    }
    mpuHandler.recoverSensor();
    return;
  }
  
  // 重置错误计数器
  resetCount = 0;
  
  // 变量初始化
  float speedFactor = MIN_SPEED_FACTOR;
  int direction = 1; // 1表示正向，-1表示反向
  float redFactor = 0.0; // 0.0=全绿, 1.0=全红
  
  if (data.accelX > ACCEL_THRESHOLD_LOW) {
    // === 加速状态 (绿色，正向) ===
    float accelRange = ACCEL_THRESHOLD_HIGH - ACCEL_THRESHOLD_LOW;
    float speedRange = MAX_SPEED_FACTOR - MIN_SPEED_FACTOR;
    float normalizedAccel = min(data.accelX - ACCEL_THRESHOLD_LOW, accelRange);
    speedFactor = MIN_SPEED_FACTOR + (normalizedAccel / accelRange) * speedRange;
    direction = 1;
    redFactor = 0.0; // 保持绿色
    
  } else if (data.accelX < DECEL_THRESHOLD_LOW) {
    // === 减速状态 (反向，渐变红) ===
    
    // 1. 计算速度因子 (逻辑不变)
    float decelRange = abs(DECEL_THRESHOLD_HIGH - DECEL_THRESHOLD_LOW);
    float speedRange = MAX_SPEED_FACTOR - MIN_SPEED_FACTOR;
    float normalizedDecel = min(abs(data.accelX) - abs(DECEL_THRESHOLD_LOW), decelRange);
    speedFactor = MIN_SPEED_FACTOR + (normalizedDecel / decelRange) * speedRange;
    direction = -1;

    // 2. 计算变红程度 (新增逻辑)
    // 计算渐变区间的中点 (例如: -1.5 和 -5 的中点是 -3.25)
    float midDecel = (DECEL_THRESHOLD_LOW + DECEL_THRESHOLD_HIGH) / 2.0;
    
    if (data.accelX <= midDecel) {
        // 如果加速度超过中点（更负），直接全红
        redFactor = 1.0;
    } else {
        // 在开始阈值和中点之间进行线性插值
        // 范围是从 DECEL_THRESHOLD_LOW 到 midDecel
        float fadeRange = abs(midDecel - DECEL_THRESHOLD_LOW);
        float currentVal = abs(data.accelX - DECEL_THRESHOLD_LOW);
        redFactor = constrain(currentVal / fadeRange, 0.0, 1.0);
    }

  } else {
    // === 正常状态 ===
    speedFactor = MIN_SPEED_FACTOR;
    direction = 1;
    redFactor = 0.0;
  }
  
  // 更新灯条参数
  pixelHandler.setSpeedFactor(speedFactor);
  pixelHandler.setDirection(direction*-1);
  pixelHandler.setRedFactor(redFactor); // 设置红色混合比例
  
  // 输出调试信息
  Serial.print("AccX: "); Serial.print(data.accelX, 2);
  Serial.print("\tSpeed: "); Serial.print(speedFactor, 2);
  Serial.print("\tRedRatio: "); Serial.print(redFactor, 2);
  Serial.println();
  
  // 更新灯条效果
  pixelHandler.update();
}