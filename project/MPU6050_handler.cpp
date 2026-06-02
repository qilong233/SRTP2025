#include "MPU6050_handler.h"

MPU6050Handler::MPU6050Handler() : isCalibrated(false) {
  // 初始化偏移量为0
  for (int i = 0; i < 3; i++) {
    accel_offset[i] = 0;
    gyro_offset[i] = 0;
  }
}

bool MPU6050Handler::init() {
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
    return false;
  }
  
  // 配置传感器设置
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  initKalmanFilters();
  calibrate();
  
  Serial.println("MPU6050 Ready!");
  return true;
}

void MPU6050Handler::initKalmanFilters() {
  // 初始化卡尔曼滤波器参数
  for (int i = 0; i < 3; i++) {
    // 加速度计滤波器参数
    accel_filter[i].Q = 0.01;
    accel_filter[i].R = 0.1;
    accel_filter[i].P = 1.0;
    accel_filter[i].K = 0.0;
    accel_filter[i].X = 0.0;
    
    // 陀螺仪滤波器参数
    gyro_filter[i].Q = 0.005;
    gyro_filter[i].R = 0.05;
    gyro_filter[i].P = 1.0;
    gyro_filter[i].K = 0.0;
    gyro_filter[i].X = 0.0;
  }
}

void MPU6050Handler::calibrate() {
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

bool MPU6050Handler::readData(SensorData &data) {
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
  
  if (!readSuccess) {
    return false;
  }
  
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
  
  // 填充结果结构
  data.accelX = corrected_accel[0];
  data.accelY = corrected_accel[1];
  data.accelZ = corrected_accel[2];
  data.gyroX = corrected_gyro[0];
  data.gyroY = corrected_gyro[1];
  data.gyroZ = corrected_gyro[2];
  data.temperature = temp.temperature;
  
  return true;
}

float MPU6050Handler::kalmanFilter(KalmanFilter* filter, float measurement) {
  // 预测步骤
  filter->P = filter->P + filter->Q;
  
  // 更新步骤
  filter->K = filter->P / (filter->P + filter->R);
  filter->X = filter->X + filter->K * (measurement - filter->X);
  filter->P = (1 - filter->K) * filter->P;
  
  return filter->X;
}

void MPU6050Handler::recoverSensor() {
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
}