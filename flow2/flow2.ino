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
// 这些参数可以方便地调整以适应不同的需求

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

// ===== 参数配置结束 =====

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // 初始化传感器和灯条
  if (!mpuHandler.init()) {
    Serial.println("MPU6050 initialization failed!");
    while(1); // 停止执行
  }
  pixelHandler.init();
  
  // 打印当前配置参数
  Serial.println("MPU6050 with Kalman Filter and Zero Drift Correction");
  Serial.println("=== 当前配置参数 ===");
  Serial.print("加速度阈值: "); Serial.print(ACCEL_THRESHOLD_LOW); 
  Serial.print(" 到 "); Serial.println(ACCEL_THRESHOLD_HIGH);
  Serial.print("减速度阈值: "); Serial.print(DECEL_THRESHOLD_LOW); 
  Serial.print(" 到 "); Serial.println(DECEL_THRESHOLD_HIGH);
  Serial.print("速度因子范围: "); Serial.print(MIN_SPEED_FACTOR); 
  Serial.print(" 到 "); Serial.println(MAX_SPEED_FACTOR);
  Serial.print("反向减速因子: "); Serial.println(REVERSE_SLOW_FACTOR);
  Serial.println("====================");
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
  float speedFactor = MIN_SPEED_FACTOR;
  int direction = 1; // 1表示正向，-1表示反向
  
  if (data.accelX > ACCEL_THRESHOLD_LOW) {
    // 加速状态：正向流动，速度增加
    // 计算速度系数 (从阈值到最大阈值映射到最小到最大速度因子)
    float accelRange = ACCEL_THRESHOLD_HIGH - ACCEL_THRESHOLD_LOW;
    float speedRange = MAX_SPEED_FACTOR - MIN_SPEED_FACTOR;
    float normalizedAccel = min(data.accelX - ACCEL_THRESHOLD_LOW, accelRange);
    speedFactor = MIN_SPEED_FACTOR + (normalizedAccel / accelRange) * speedRange;
    direction = 1;
  } else if (data.accelX < DECEL_THRESHOLD_LOW) {
    // 减速状态：反向流动，速度增加
    // 使用绝对值计算速度因子
    float decelRange = abs(DECEL_THRESHOLD_HIGH - DECEL_THRESHOLD_LOW);
    float speedRange = MAX_SPEED_FACTOR - MIN_SPEED_FACTOR;
    float normalizedDecel = min(abs(data.accelX) - abs(DECEL_THRESHOLD_LOW), decelRange);
    speedFactor = MIN_SPEED_FACTOR + (normalizedDecel / decelRange) * speedRange;
    direction = -1;
  } else {
    // 正常状态：正向流动，默认速度
    speedFactor = MIN_SPEED_FACTOR;
    direction = 1;
  }
  
  pixelHandler.setSpeedFactor(speedFactor);
  pixelHandler.setDirection(direction);
  
  // 输出处理后的数据
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