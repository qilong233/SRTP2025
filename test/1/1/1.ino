#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

Adafruit_MPU6050 mpu;
#define LED_PIN     6     // 灯带数据引脚
#define LED_COUNT   60    // 灯珠数量
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// 灯带控制变量
float offset = 0.0;                // 动态偏移量
float baseOffsetIncrement = 0.02;  // 基础偏移增量
float currentOffsetIncrement;      // 当前实际偏移增量
unsigned long lastLedUpdate = 0;   // 上次灯带更新时间
const int LED_UPDATE_INTERVAL = 30; // 灯带更新间隔(ms)

// 加速度控制参数
const float ACCEL_THRESHOLD = 3.0;  // 加速度阈值(m/s²)
const float MAX_SPEED_BOOST = 0.04; // 最大速度增量

// 状态跟踪变量
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 150; // 读取间隔(ms)
int resetCount = 0;
const int MAX_RESETS = 5; // 最大复位次数

// 零漂校正相关变量
bool isCalibrated = false;
float accel_offset[3] = {0, 0, 0}; // 加速度偏移量
float gyro_offset[3] = {0, 0, 0};  // 陀螺仪偏移量
const int CALIBRATION_SAMPLES = 200; // 校准样本数

// 卡尔曼滤波器结构体
struct KalmanFilter {
  float Q; // 过程噪声协方差
  float R; // 测量噪声协方差
  float P; // 估计误差协方差
  float K; // 卡尔曼增益
  float X; // 状态估计值
};

// 为每个轴创建卡尔曼滤波器实例
KalmanFilter accel_filter[3];
KalmanFilter gyro_filter[3];

// 定义绿色系颜色
const uint32_t LIGHT_GREEN = strip.Color(150, 255, 150);  // 浅绿
const uint32_t GREEN = strip.Color(0, 255, 0);           // 标准绿
const uint32_t DARK_GREEN = strip.Color(0, 50, 0);       // 墨绿

void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // 等待串口连接
  
  // 初始化灯带
  strip.begin();
  strip.setBrightness(50);  // 建议亮度50-100
  strip.show();             // 初始化灯条
  
  initSensor(); // 初始化传感器
  initKalmanFilters(); // 初始化卡尔曼滤波器
  calibrateSensor(); // 执行零漂校正
  
  currentOffsetIncrement = baseOffsetIncrement; // 初始化速度
  
  Serial.println("MPU6050 with Light Control");
  Serial.println("Acceleration (m/s²)\t\tGyroscope (deg/s)");
  Serial.println("X\t\tY\t\tZ\t\tX\t\tY\t\tZ\tTemp(°C)");
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
  lastReadTime = millis(); // 重置计时器
}

void initKalmanFilters() {
  // 初始化卡尔曼滤波器参数
  for (int i = 0; i < 3; i++) {
    // 加速度计滤波器参数
    accel_filter[i].Q = 0.01;  // 过程噪声 (较小，信任模型)
    accel_filter[i].R = 0.1;   // 测量噪声
    accel_filter[i].P = 1.0;   // 初始估计误差
    accel_filter[i].K = 0.0;   // 卡尔曼增益
    accel_filter[i].X = 0.0;   // 初始状态估计
    
    // 陀螺仪滤波器参数
    gyro_filter[i].Q = 0.005; // 过程噪声 (更小，陀螺仪更稳定)
    gyro_filter[i].R = 0.05;  // 测量噪声
    gyro_filter[i].P = 1.0;   // 初始估计误差
    gyro_filter[i].K = 0.0;   // 卡尔曼增益
    gyro_filter[i].X = 0.0;   // 初始状态估计
  }
}

void calibrateSensor() {
  Serial.println("Calibrating sensor... Please keep the sensor stationary!");
  
  float accel_sum[3] = {0, 0, 0};
  float gyro_sum[3] = {0, 0, 0};
  int valid_samples = 0;
  
  // 收集校准数据
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t accel, gyro, temp;
    
    if (mpu.getEvent(&accel, &gyro, &temp)) {
      // 累加陀螺仪数据用于零点校正
      gyro_sum[0] += gyro.gyro.x;
      gyro_sum[1] += gyro.gyro.y;
      gyro_sum[2] += gyro.gyro.z;
      
      // 累加加速度数据，但保留重力分量
      accel_sum[0] += accel.acceleration.x;
      accel_sum[1] += accel.acceleration.y;
      accel_sum[2] += accel.acceleration.z;
      
      valid_samples++;
      
      if (i % 50 == 0) {
        Serial.print("Calibration progress: ");
        Serial.print((i * 100) / CALIBRATION_SAMPLES);
        Serial.println("%");
      }
    }
    delay(10);
  }
  
  if (valid_samples > 0) {
    // 计算陀螺仪偏移量 (应该为0)
    gyro_offset[0] = gyro_sum[0] / valid_samples;
    gyro_offset[1] = gyro_sum[1] / valid_samples;
    gyro_offset[2] = gyro_sum[2] / valid_samples;
    
    // 计算加速度偏移量，但保持重力分量
    // 假设校准时Z轴向上，应该有约9.8m/s²的重力
    accel_offset[0] = accel_sum[0] / valid_samples; // X轴偏移
    accel_offset[1] = accel_sum[1] / valid_samples; // Y轴偏移
    // Z轴：移除偏移但保留重力 (期望值为9.8，实际测量值减去9.8就是偏移)
    accel_offset[2] = (accel_sum[2] / valid_samples) - 9.8;
    
    isCalibrated = true;
    
    Serial.println("Calibration complete!");
    Serial.print("Gyro offsets: ");
    Serial.print(gyro_offset[0] * 57.2958, 3); Serial.print(", ");
    Serial.print(gyro_offset[1] * 57.2958, 3); Serial.print(", ");
    Serial.print(gyro_offset[2] * 57.2958, 3); Serial.println(" deg/s");
    
    Serial.print("Accel offsets: ");
    Serial.print(accel_offset[0], 3); Serial.print(", ");
    Serial.print(accel_offset[1], 3); Serial.print(", ");
    Serial.print(accel_offset[2], 3); Serial.println(" m/s²");
  } else {
    Serial.println("Calibration failed!");
  }
}

float kalmanFilter(KalmanFilter* filter, float measurement) {
  // 预测步骤
  filter->P = filter->P + filter->Q;
  
  // 更新步骤
  filter->K = filter->P / (filter->P + filter->R);
  filter->X = filter->X + filter->K * (measurement - filter->X);
  filter->P = (1 - filter->K) * filter->P;
  
  return filter->X;
}

void updateLightEffect() {
  // 非阻塞方式更新灯带
  unsigned long currentMillis = millis();
  if (currentMillis - lastLedUpdate >= LED_UPDATE_INTERVAL) {
    lastLedUpdate = currentMillis;
    
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
    offset += currentOffsetIncrement;  // 应用当前速度
    if (offset > 1.0) offset = 0;
  }
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

void adjustLightSpeed(float accelX) {
  // 当X轴加速度超过阈值时增加速度
  if (abs(accelX) > ACCEL_THRESHOLD) {
    // 加速度越大，速度增加越多（最大到基础值+0.04）
    float boost = min(MAX_SPEED_BOOST, (abs(accelX) - ACCEL_THRESHOLD) * 0.01);
    currentOffsetIncrement = baseOffsetIncrement + boost;
  } else {
    // 低于阈值时恢复基础速度
    currentOffsetIncrement = baseOffsetIncrement;
  }
  
  // 可选：输出调试信息
  // Serial.print("AccelX: "); Serial.print(accelX);
  // Serial.print(" Speed: "); Serial.println(currentOffsetIncrement, 4);
}

void loop() {
  updateLightEffect(); // 持续更新灯带效果
  
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
  
  // 应用零漂校正和卡尔曼滤波
  float corrected_accel[3], corrected_gyro[3];
  
  if (isCalibrated) {
    // 零漂校正 - 加速度保留重力分量
    corrected_accel[0] = accel.acceleration.x - accel_offset[0];
    corrected_accel[1] = accel.acceleration.y - accel_offset[1];
    corrected_accel[2] = accel.acceleration.z - accel_offset[2]; // 包含重力
    
    // 零漂校正 - 陀螺仪
    corrected_gyro[0] = (gyro.gyro.x - gyro_offset[0]) * 57.2958; // 转换为度/秒
    corrected_gyro[1] = (gyro.gyro.y - gyro_offset[1]) * 57.2958;
    corrected_gyro[2] = (gyro.gyro.z - gyro_offset[2]) * 57.2958;
    
    // 应用卡尔曼滤波
    for (int i = 0; i < 3; i++) {
      corrected_accel[i] = kalmanFilter(&accel_filter[i], corrected_accel[i]);
      corrected_gyro[i] = kalmanFilter(&gyro_filter[i], corrected_gyro[i]);
    }
  } else {
    // 如果未校准，使用原始数据但应用卡尔曼滤波
    corrected_accel[0] = kalmanFilter(&accel_filter[0], accel.acceleration.x);
    corrected_accel[1] = kalmanFilter(&accel_filter[1], accel.acceleration.y);
    corrected_accel[2] = kalmanFilter(&accel_filter[2], accel.acceleration.z);
    
    corrected_gyro[0] = kalmanFilter(&gyro_filter[0], gyro.gyro.x * 57.2958);
    corrected_gyro[1] = kalmanFilter(&gyro_filter[1], gyro.gyro.y * 57.2958);
    corrected_gyro[2] = kalmanFilter(&gyro_filter[2], gyro.gyro.z * 57.2958);
  }
  
  // 根据X轴加速度调整灯带速度
  adjustLightSpeed(corrected_accel[0]);
  
  // === 输出处理后的加速度数据 (包含重力) ===
  Serial.print(corrected_accel[0], 2); Serial.print("\t\t");
  Serial.print(corrected_accel[1], 2); Serial.print("\t\t");
  Serial.print(corrected_accel[2], 2); Serial.print("\t\t");
  
  // === 输出处理后的角速度数据 (度/秒) ===
  Serial.print(corrected_gyro[0], 1); Serial.print("\t\t");
  Serial.print(corrected_gyro[1], 1); Serial.print("\t\t");
  Serial.print(corrected_gyro[2], 1); Serial.print("\t\t");
  
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