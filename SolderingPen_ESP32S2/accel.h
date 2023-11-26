#ifndef __ACCEL_H
#define __ACCEL_H

// 选择加速度计芯片
// #define MPU
#define LIS

/*#if defined(MPU)
  #include <MPU6050_tockn.h> //https://github.com/tockn/MPU6050_tockn
  MPU6050 mpu6050(Wire);*/

// #if defined(LIS)
#include "SparkFun_LIS2DH12.h"  //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
extern SPARKFUN_LIS2DH12 accel;  // Create instance

// get LIS/MPU temperature 获取LIS/MPU的温度
float getMPUTemp();

// 读取SENSOR内部温度
double getChipTemp();

#endif
