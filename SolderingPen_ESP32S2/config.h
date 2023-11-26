#ifndef __CONFIG_H
#define __CONFIG_H

// Firmware version
#define VERSION "v4.5.0" //20230729
#define VERSION_NUM 422

// Pins
#define SENSOR_PIN        1     // tip temperature sense 烙铁头温感


// 默认温度控制值(推荐焊接温度:300~380°C)
#define TEMP_MIN          50    // 最小温度
#define TEMP_MAX          450   // 最大温度
#define TEMP_DEFAULT      260   // 默认温度
#define TEMP_SLEEP        150   // 休眠温度
#define TEMP_BOOST        50    // 升温步进
#define TEMP_STEP         10    // 旋转编码器温度变化步进
#define POWER_LIMIT_15    170   // 功率限制
#define POWER_LIMIT_20    255   // 功率限制
#define POWER_LIMIT_20_2  127   // 功率限制

// 默认的T12烙铁头温度校准值
#define TEMP200           200   // temperature at ADC = 200 
#define TEMP280           280   // temperature at ADC = 280
#define TEMP360           360   // temperature at ADC = 360 
#define TEMPCHP           35    // chip temperature while calibration 校准时芯片温度
#define CALNUM            4     // Calibration point number
#define TIPMAX            8     // max number of tips
#define TIPNAMELENGTH     6     // max length of tip names (including termination)
#define TIPNAME           "PTS  " // default tip name

// 默认的定时器值 (0 = 禁用)
#define TIME2SLEEP        60    // 几秒钟后进入睡眠模式
#define TIME2OFF          5     // 几分钟后就要关闭加热器了
#define TIMEOFBOOST       60    // 停留在加热模式多少秒
#define WAKEUP_THRESHOLD  10    // MPU 震动检测精度，数值越小，越灵敏

// Control values
#define TIME2SETTLE       5000  // 以微秒为单位的时间允许OpAmp输出稳定
#define TIME2SETTLE_20V   2000  // 以微秒为单位的时间允许OpAmp输出稳定
#define SMOOTHIE          0.05  // OpAmp输出平滑系数 (1=无平滑; 默认：0.05)
#define PID_ENABLE        false // enable PID control
#define BEEP_ENABLE       true  // enable/disable buzzer
#define VOLTAGE_VALUE     3     // 电压值
#define QC_ENABLE         false // enable/disable QC3.0
#define MAINSCREEN        1     // type of main screen (0: big numbers; 1: more infos)

// EEPROM identifier
#define EEPROM_SIZE       1024

//Language
#define DEFAULT_LANGUAGE  0

//Hand side
#define DEFAULT_HAND_SIDE 0

#endif
