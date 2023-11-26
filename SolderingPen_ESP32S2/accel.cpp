#include "accel.h"

SPARKFUN_LIS2DH12 accel;  // Create instance

// get LIS/MPU temperature 获取LIS/MPU的温度
float getMPUTemp() {
#if defined(MPU)
  mpu6050.update();
  int16_t Temp = mpu6050.getTemp();
#elif defined(LIS)
  int16_t Temp = accel.getTemperature();
#endif

  return Temp;
}

// 读取SENSOR内部温度
double getChipTemp() {
#if defined(MPU)
  mpu6050.update();
  int16_t Temp = mpu6050.getTemp();
#elif defined(LIS)
  int16_t Temp = accel.getTemperature();
#endif

  return Temp;
}

