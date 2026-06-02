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

// 配置参数
const float ACCEL_THRESHOLD = 2.5;    // 加速度阈值
const float DECEL_THRESHOLD = -2.0;   // 减速度阈值
const int BLINK_INTERVAL_MIN = 50;    // 最小闪烁间隔(ms)
const int BLINK_INTERVAL_MAX = 500;   // 最大闪烁间隔(ms)

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // 初始化传感器和灯条
  if (!mpuHandler.init()) {
    Serial.println("MPU6050 initialization failed!");
    while(1); // 停止执行
  }
  pixelHandler.init();
  
  // 保持与方法1相同的输出格式
  Serial.println("MPU6050 with Kalman Filter and Zero Drift Correction");
  Serial.println("Acceleration (m/s²)\t\tGyroscope (deg/s)");
  Serial.println("X\t\tY\t\tZ\t\tX\t\tY\t\tZ\tTemp(°C)");
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
    
    // 尝试软复位
    Serial.print("Reset ");
    Serial.println(resetCount);
    mpuHandler.recoverSensor();
    return;
  }
  
  // 重置错误计数器
  resetCount = 0;
  
  // 根据X轴加速度调整灯条模式
  int blinkInterval = 0;
  bool isBlinkMode = false;
  
  if (data.accelX > ACCEL_THRESHOLD || data.accelX < DECEL_THRESHOLD) {
    // 加速或减速状态：快速闪烁
    isBlinkMode = true;
    float absAccel = abs(data.accelX);
    float threshold = (data.accelX > 0) ? ACCEL_THRESHOLD : abs(DECEL_THRESHOLD);
    
    // 计算闪烁间隔 (加速度越大，间隔越小，闪烁越快)
    blinkInterval = BLINK_INTERVAL_MAX - 
                   (absAccel - threshold) * 
                   (BLINK_INTERVAL_MAX - BLINK_INTERVAL_MIN) / 
                   (10.0 - threshold); // 假设最大加速度为10m/s²
    
    // 确保间隔在合理范围内
    blinkInterval = constrain(blinkInterval, BLINK_INTERVAL_MIN, BLINK_INTERVAL_MAX);
  }
  
  // 设置灯条模式和闪烁间隔
  pixelHandler.setMode(isBlinkMode);
  pixelHandler.setBlinkInterval(blinkInterval);
  
  // 输出处理后的数据 - 保持与方法1相同的格式
  Serial.print(data.accelX, 2); Serial.print("\t\t");
  Serial.print(data.accelY, 2); Serial.print("\t\t");
  Serial.print(data.accelZ, 2); Serial.print("\t\t");
  
  Serial.print(data.gyroX, 1); Serial.print("\t\t");
  Serial.print(data.gyroY, 1); Serial.print("\t\t");
  Serial.print(data.gyroZ, 1); Serial.print("\t\t");
  
  Serial.print(data.temperature, 1);
  
  // 添加额外信息（闪烁间隔），但不影响原有格式
  if (isBlinkMode) {
    Serial.print("\tBlink:");
    Serial.print(blinkInterval);
  }
  
  Serial.println();  // 换行
  
  // 更新灯条效果
  pixelHandler.update();
}