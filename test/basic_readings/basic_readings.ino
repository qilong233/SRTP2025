#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// 状态跟踪变量
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 150; // 读取间隔(ms)
int resetCount = 0;
const int MAX_RESETS = 5; // 最大复位次数

void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // 等待串口连接
  
  initSensor(); // 初始化传感器
  Serial.println("Corrected MPU6050 Reader - Acceleration and Gyro Data Properly Assigned");
}

void initSensor() {
  Serial.println("Initializing MPU6050...");
  
  // 尝试初始化MPU6050
  bool initialized = false;
  for (int i = 0; i < 3; i++) {
    Wire.begin(); // 重置I2C总线
    delay(50);
    
    if (mpu.begin()) {
      initialized = true;
      break;
    }
    Serial.print("Init failed, retrying... (");
    Serial.print(i+1);
    Serial.println("/3)");
    delay(300);
  }
  
  if (!initialized) {
    Serial.println("MPU6050 initialization failed!");
    return;
  }
  
  // 配置传感器设置
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 Ready!");
  
  // 正确的表头（加速度在前，角速度在后）
  Serial.println("Acceleration (m/s²)\t\tGyroscope (deg/s)");
  Serial.println("X\t\tY\t\tZ\t\tX\t\tY\t\tZ\tTemp(°C)");
  
  lastReadTime = millis(); // 重置计时器
}

void loop() {
  // 检查是否到达读取间隔
  if (millis() - lastReadTime < READ_INTERVAL) {
    return;
  }
  lastReadTime = millis();
  
  // 创建传感器事件对象
  sensors_event_t accel, gyro, temp;
  
  // 尝试读取传感器数据
  bool readSuccess = false;
  for (int attempt = 0; attempt < 2; attempt++) {
    if (mpu.getEvent(&accel, &gyro, &temp)) {
      readSuccess = true;
      break;
    }
    delay(10); // 短暂延迟后重试
  }
  
  // 检查数据有效性
  if (!readSuccess || 
      (accel.acceleration.x == 0 && accel.acceleration.y == 0 && accel.acceleration.z == 0) ||
      (gyro.gyro.x == 0 && gyro.gyro.y == 0 && gyro.gyro.z == 0)) {
    
    Serial.print("Invalid data! Attempting recovery... ");
    resetCount++;
    
    if (resetCount > MAX_RESETS) {
      Serial.println("Critical failure! Please reset Arduino.");
      while(1); // 永久停止
    }
    
    // 尝试软复位
    Serial.print("Reset ");
    Serial.println(resetCount);
    recoverSensor();
    return;
  }
  
  // 重置错误计数器
  resetCount = 0;
  
  // === 正确输出加速度数据 ===
  Serial.print(accel.acceleration.x, 2); Serial.print("\t\t");
  Serial.print(accel.acceleration.y, 2); Serial.print("\t\t");
  Serial.print(accel.acceleration.z, 2); Serial.print("\t\t");
  
  // === 正确输出角速度数据 ===
  // 注意：Adafruit库返回的是弧度/秒，转换为度/秒
  Serial.print(gyro.gyro.x * 57.2958, 1); Serial.print("\t\t"); // 弧度转度 (1 rad/s ≈ 57.3 deg/s)
  Serial.print(gyro.gyro.y * 57.2958, 1); Serial.print("\t\t");
  Serial.print(gyro.gyro.z * 57.2958, 1); Serial.print("\t\t");
  
  // 显示温度数据（°C）
  Serial.print(temp.temperature, 1);
  
  Serial.println();  // 换行
}

void recoverSensor() {
  // 1. 完全重置I2C总线
  Wire.end();
  delay(50);
  Wire.begin();
  delay(100);
  
  // 2. 重置传感器
  mpu.begin(); // 尝试重新初始化
  
  // 3. 重置配置
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // 4. 清空可能存在的错误状态
  for (int i = 0; i < 3; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); // 读取并丢弃
    delay(20);
  }
  
  Serial.println("Recovery complete. Resuming data collection...");
  lastReadTime = millis(); // 重置计时器
}