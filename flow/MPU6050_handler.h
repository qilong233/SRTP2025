#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// 传感器数据结构
struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temperature;
};

class MPU6050Handler {
public:
  MPU6050Handler();
  bool init();
  bool readData(SensorData &data);
  void recoverSensor();
  void calibrate();
  
private:
  Adafruit_MPU6050 mpu;
  
  // 零漂校正相关变量
  bool isCalibrated;
  float accel_offset[3];
  float gyro_offset[3];
  const int CALIBRATION_SAMPLES = 200;
  
  // 卡尔曼滤波器结构体
  struct KalmanFilter {
    float Q; // 过程噪声协方差
    float R; // 测量噪声协方差
    float P; // 估计误差协方差
    float K; // 卡尔曼增益
    float X; // 状态估计值
  };
  
  KalmanFilter accel_filter[3];
  KalmanFilter gyro_filter[3];
  
  void initKalmanFilters();
  float kalmanFilter(KalmanFilter* filter, float measurement);
};

#endif