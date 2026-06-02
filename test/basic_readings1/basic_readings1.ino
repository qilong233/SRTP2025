#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// 状态跟踪变量
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 150; // 读取间隔(ms)
int resetCount = 0;
const int MAX_RESETS = 5; // 最大复位次数

// 零漂校正相关变量
bool isCalibrated = false;
float accel_offset[3] = {0, 0, 0}; // 加速度偏移量 (保留重力分量)
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

void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // 等待串口连接
  
  initSensor(); // 初始化传感器
  initKalmanFilters(); // 初始化卡尔曼滤波器
  calibrateSensor(); // 执行零漂校正
  
  Serial.println("MPU6050 with Kalman Filter and Zero Drift Correction");
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