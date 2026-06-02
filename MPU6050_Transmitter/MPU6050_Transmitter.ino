#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "MPU6050_handler.h"

// 创建对象
RF24 radio(9, 10); // CE, CSN
MPU6050Handler mpuHandler;

// 无线通信地址
const byte address[6] = "00001";

// 数据结构
struct WirelessSensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temperature;
  unsigned long timestamp;
};

WirelessSensorData wirelessData;

// 状态变量
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // 发送间隔(ms)
bool radioInitialized = false;
int sendCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // 初始化无线模块
  if (!radio.begin()) {
    Serial.println("nRF24L01+ initialization failed!");
    while (1);
  }
  
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.stopListening();
  
  radioInitialized = true;
  Serial.println("nRF24L01+ Transmitter Ready");
  
  // 初始化MPU6050
  if (!mpuHandler.init()) {
    Serial.println("MPU6050 initialization failed!");
    while(1);
  }
  
  Serial.println("=== Wireless Sensor Transmitter ===");
  Serial.println("Acceleration (m/s²)\t\tGyroscope (deg/s)");
  Serial.println("X\t\tY\t\tZ\t\tX\t\tY\t\tZ\tTemp(°C)\tPackets");
}

void loop() {
  // 检查发送间隔
  if (millis() - lastSendTime < SEND_INTERVAL) {
    return;
  }
  lastSendTime = millis();
  
  // 读取传感器数据
  SensorData sensorData;
  bool readSuccess = mpuHandler.readData(sensorData);
  
  if (!readSuccess) {
    Serial.println("Failed to read sensor data!");
    return;
  }
  
  // 填充无线数据结构
  wirelessData.accelX = sensorData.accelX;
  wirelessData.accelY = sensorData.accelY;
  wirelessData.accelZ = sensorData.accelZ;
  wirelessData.gyroX = sensorData.gyroX;
  wirelessData.gyroY = sensorData.gyroY;
  wirelessData.gyroZ = sensorData.gyroZ;
  wirelessData.temperature = sensorData.temperature;
  wirelessData.timestamp = millis();
  
  // 发送数据
  bool success = false;
  if (radioInitialized) {
    success = radio.write(&wirelessData, sizeof(wirelessData));
  }
  
  // 串口输出
  Serial.print(wirelessData.accelX, 2); Serial.print("\t\t");
  Serial.print(wirelessData.accelY, 2); Serial.print("\t\t");
  Serial.print(wirelessData.accelZ, 2); Serial.print("\t\t");
  
  Serial.print(wirelessData.gyroX, 1); Serial.print("\t\t");
  Serial.print(wirelessData.gyroY, 1); Serial.print("\t\t");
  Serial.print(wirelessData.gyroZ, 1); Serial.print("\t\t");
  
  Serial.print(wirelessData.temperature, 1); Serial.print("\t\t");
  
  if (success) {
    sendCount++;
    Serial.print(sendCount);
    Serial.println(" ✓");
  } else {
    Serial.println(" ✗ FAILED");
  }
  
  // 每100次发送显示一次信号状态
  if (sendCount % 100 == 0) {
    Serial.print("Signal strength: ");
    Serial.println(radio.getPALevel() == RF24_PA_MAX ? "MAX" : 
                   radio.getPALevel() == RF24_PA_HIGH ? "HIGH" :
                   radio.getPALevel() == RF24_PA_LOW ? "LOW" : "MIN");
  }
}