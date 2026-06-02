#include "MPU6050_handler.h"
#include "NeoPixel_handler.h"

// 创建全局对象
MPU6050Handler mpuHandler;
NeoPixelHandler pixelHandler;

// 状态跟踪变量（从MPU6050_handler移出，因为需要在主循环中访问）
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 150; // 读取间隔(ms)
int resetCount = 0;
const int MAX_RESETS = 5; // 最大复位次数

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // 初始化传感器和灯条
  if (!mpuHandler.init()) {
    Serial.println("MPU6050 initialization failed!");
    while(1); // 停止执行
  }
  pixelHandler.init();
  
  Serial.println("MPU6050 with Kalman Filter and Zero Drift Correction");
  Serial.println("Acceleration (m/s²)\t\tGyroscope (deg/s)");
  Serial.println("X\t\tY\t\tZ\t\tX\t\tY\t\tZ\tTemp(°C)\tSpeed\tDirection");
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
  
  // 根据X轴加速度调整灯条速度和方向
  float speedFactor = 1.0;
  int direction = 1; // 1表示正向，-1表示反向
  
  if (data.accelX > 3.0) {
    // 加速状态：正向流动，速度增加
    speedFactor = 1.0 + min((data.accelX - 3.0) / 3.5, 2.0);
    direction = 1;
  } else if (data.accelX < -2.0) {
    // 减速状态：反向流动，速度增加
    speedFactor = 1.0 + min(abs(data.accelX) / 3.5, 2.0);
    direction = -1;
  } else {
    // 正常状态：正向流动，默认速度
    speedFactor = 1.0;
    direction = 1;
  }
  
  pixelHandler.setSpeedFactor(speedFactor);
  pixelHandler.setDirection(direction);
  
  // 输出处理后的数据（与初始代码格式相同）
  Serial.print(data.accelX, 2); Serial.print("\t\t");
  Serial.print(data.accelY, 2); Serial.print("\t\t");
  Serial.print(data.accelZ, 2); Serial.print("\t\t");
  
  Serial.print(data.gyroX, 1); Serial.print("\t\t");
  Serial.print(data.gyroY, 1); Serial.print("\t\t");
  Serial.print(data.gyroZ, 1); Serial.print("\t\t");
  
  Serial.print(data.temperature, 1); Serial.print("\t\t");
  
  // 添加灯条速度显示
  Serial.print(speedFactor, 2); Serial.print("\t");
  
  // 添加灯条方向显示
  Serial.print(direction == 1 ? "Forward" : "Reverse");
  
  Serial.println();  // 换行
  
  // 更新灯条效果
  pixelHandler.update();
}