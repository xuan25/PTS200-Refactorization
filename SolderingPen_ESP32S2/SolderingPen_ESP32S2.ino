#include <esp32-hal.h>

//
#include "config.h"

//
#include <Button2.h>



#include "USB.h"
#include "UtilsEEPROM.h"



// #include<analogWrite.h>
#include <ESP32AnalogRead.h>  //Click here to get the library: http://librarymanager/All#ESP32AnalogRead

#include "esp_adc_cal.h"

// #ifdef U8X8_HAVE_HW_SPI
// #include <SPI.h>
// #endif
// #ifdef U8X8_HAVE_HW_I2C
// #include <Wire.h>
// #endif

#include <Wire.h>

#include <PID_v1.h>  // https://github.com/wagiminator/ATmega-Soldering-Station/blob/master/software/libraries/Arduino-PID-Library.zip
// (old cpp version of
// https://github.com/mblythe86/C-PID-Library/tree/master/PID_v1)
#include <EEPROM.h>  // 用于将用户设置存储到EEPROM







#include "power.h"
#include "heater.h"
#include "utils.h"
#include "accel.h"
#include "menu.h"



/*#else
  #error Wrong SENSOR type!
  #endif*/

int16_t gx = 0, gy = 0, gz = 0;
uint16_t accels[32][3];
uint8_t accelIndex = 0;
#define ACCEL_SAMPLES 32

// 定义积极和保守的PID调整参数
double aggKp = 11, aggKi = 0.5, aggKd = 1;
double consKp = 11, consKi = 3, consKd = 5;

// 用户可以更改并存储在EEPROM中的默认值
uint16_t DefaultTemp = TEMP_DEFAULT;
uint16_t SleepTemp = TEMP_SLEEP;
uint8_t BoostTemp = TEMP_BOOST;
uint16_t time2sleep = TIME2SLEEP;
uint8_t time2off = TIME2OFF;
uint8_t timeOfBoost = TIMEOFBOOST;
uint8_t MainScrType = MAINSCREEN;
bool PIDenable = PID_ENABLE;
bool beepEnable = BEEP_ENABLE;
uint8_t VoltageValue = VOLTAGE_VALUE;
bool QCEnable = QC_ENABLE;
uint8_t WAKEUPthreshold = WAKEUP_THRESHOLD;
bool restore_default_config = false;

// T12的默认值
uint16_t CalTemp[TIPMAX][4] = {TEMP200, TEMP280, TEMP360, TEMPCHP};
char TipName[TIPMAX][TIPNAMELENGTH] = {TIPNAME};
uint8_t CurrentTip = 0;
uint8_t NumberOfTips = 1;

// Variables for pin change interrupt 引脚更改中断的变量
volatile uint8_t c0, d0;
volatile bool ab0;
volatile bool handleMoved;

// Variables for temperature control 温度控制变量
uint16_t SetTemp, ShowTemp, gap, Step;
double Input, Output, Setpoint, RawTemp, CurrentTemp;

// Variables for voltage readings 电压读数变量
uint16_t Vcc, Vin;

// State variables 状态变量
bool inLockMode = true;
bool inSleepMode = false;
bool inOffMode = false;
bool inBoostMode = false;
bool inCalibMode = false;
bool isWorky = true;
bool beepIfWorky = true;
bool TipIsPresent = true;
bool OledClear;

// Timing variables 时间变量
uint32_t sleepmillis;
uint32_t boostmillis;
uint32_t buttonmillis;
uint32_t goneMinutes;
uint32_t goneSeconds;
uint8_t SensorCounter = 0;

// Specify variable pointers and initial PID tuning parameters
// 指定变量指针和初始PID调优参数
PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, REVERSE);

// Buffer for drawUTF8
char F_Buffer[20];

float newSENSORTmp = 0;

// ADC Calibrate
uint16_t vref_adc0, vref_adc1;
ESP32AnalogRead adc_sensor;

// Language
uint8_t language = 0;

// Hand Side
uint8_t hand_side = 0;

// Button2 Obj
Button2 btn;

float limit = 0.0;

void setup() {
  Serial.begin(115200);

  adc_sensor.attach(SENSOR_PIN);
  Vin_Init();

  // set the pin modes 设置引脚模式
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  initBuzzer();
  initButton();

  init_EEPROM();
  if (readButtonP() == BUTTON_RELEASED && readButtonN() == BUTTON_PRESSED &&
      readButtonM() == BUTTON_RELEASED) {
    write_default_EEPROM();
  }
  getEEPROM();

  Power_Init(QCEnable, VoltageValue);

  // read supply voltages in mV 以mV为单位读取电源电压
  delay(100);
  Vin = getVIN();

  // read and set current iron temperature 读取和设置当前的烙铁头温度
  SetTemp = DefaultTemp;
  RawTemp = denoiseAnalog(SENSOR_PIN);

  calculateTemp();

  // turn on heater if iron temperature is well below setpoint
  // 如果烙铁头温度远低于设定值，则打开加热器
  limit = POWER_LIMIT_20;
  if (VoltageValue < 3) {
    limit = POWER_LIMIT_15;
  }
  if (((CurrentTemp + 20) < DefaultTemp) && !inLockMode)
    Heater_SetPower(constrain(HEATER_ON, 0, limit));

  // set PID output range and start the PID
  // 设置PID输出范围，启动PID
  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(AUTOMATIC);

  // set initial rotary encoder values 设置旋转编码器的初始值
  initRotary();
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);

  // reset sleep timer 睡眠定时器重置
  sleepmillis = millis();

  // long beep for setup completion 安装完成时长哔哔声
  beep();
  beep();
  Serial.println("Soldering Pen");
  Wire.begin();
  Wire.setClock(400000);
  if (accel.begin() == false) {
    delay(500);
    Serial.println("Accelerometer not detected.");
  }

  Menu_Init(hand_side);
}

int SENSORCheckTimes = 0;

void loop() {
  ROTARYCheck();  // check rotary encoder (temp/boost setting, enter setup menu)
                  // 检查旋转编码器(温度/升压设置，进入设置菜单)
  SLEEPCheck();  // check and activate/deactivate sleep modes
                 // 检查和激活/关闭睡眠模式

  if (SENSORCheckTimes > 1) {
    SENSORCheck();  // reads temperature and vibration switch of the iron
                    // 读取烙铁头的温度和振动开关
    SENSORCheckTimes = 0;
  }
  SENSORCheckTimes++;

  Thermostat();  // heater control 加热器控制
  MainScreen(language, Setpoint, ShowTemp, inOffMode, inLockMode, inSleepMode, inBoostMode, isWorky, Output, MainScrType, Vin, newSENSORTmp);  // updates the main page on the OLED 刷新OLED主界面
}

// check rotary encoder; set temperature, toggle boost mode, enter setup menu
// accordingly 检查旋转编码器;设置温度，切换升压模式，进入设置菜单相应
void ROTARYCheck() {
  // set working temperature according to rotary encoder value
  // 根据旋转编码器值设定工作温度
  SetTemp = getRotary();

  uint8_t c = readButtonM();
  if (!c && c0) {
    delay(10);
    if (readButtonM() == c) {
      beep();
      buttonmillis = millis();
      delay(10);
      while ((!readButtonM()) && ((millis() - buttonmillis) < 500))
        ;
      delay(10);
      if ((millis() - buttonmillis) >= 500) {
        Heater_Off();  // shut off heater
        beep();
        SetupScreen(&SetTemp, &MainScrType, &VoltageValue, &QCEnable, 
            &beepEnable, &restore_default_config, &language, &hand_side, 
            TipName, CalTemp);
        updateEEPROM();
        handleMoved = true;
        setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
      } else {
        if (inLockMode) {
          inLockMode = false;
        } else {
          buttonmillis = millis();
          while ((readButtonM()) && ((millis() - buttonmillis) < 200))
            delay(10);
          if ((millis() - buttonmillis) >= 200) {  // single click
            if (inOffMode) {
              inOffMode = false;
            } else {
              inBoostMode = !inBoostMode;
              if (inBoostMode) {
                boostmillis = millis();
              }
              handleMoved = true;
            }
          } else {  // double click
            inOffMode = true;
          }
        }
      }
    }
  }
  c0 = c;

  // check timer when in boost mode 在升温模式时检查计时器
  if (inBoostMode && timeOfBoost) {
    goneSeconds = (millis() - boostmillis) / 1000;
    if (goneSeconds >= timeOfBoost) {
      inBoostMode = false;  // stop boost mode 停止升温模式
      beep();  // beep if boost mode is over 如果升温模式结束，会发出蜂鸣声
      beepIfWorky = true;  // beep again when working temperature is reached
                           // 当达到工作温度，会发出蜂鸣声
    }
  }
}

// check and activate/deactivate sleep modes 检查和激活/关闭睡眠模式
void SLEEPCheck() {
  if (inLockMode) {
    ;
  } else {
    if (handleMoved) {  // if handle was moved 如果手柄被移动
      Display_SetPowerSave(0);
      if (inSleepMode) {  // in sleep or off mode? 在睡眠模式还是关机模式?
        limit = POWER_LIMIT_20;
        if (VoltageValue < 3) {
          limit = POWER_LIMIT_15;
        }
        if ((CurrentTemp + 20) < SetTemp)  // if temp is well below setpoint 如果温度远低于设定值
            Heater_SetPower(constrain(HEATER_ON, 0, limit)); // then start the heater right now
        beep();              // beep on wake-up
        beepIfWorky = true;  // beep again when working temperature is reached
                             // 当达到工作温度，会发出蜂鸣声
      }
      handleMoved = false;  // reset handleMoved flag
      inSleepMode = false;  // reset sleep flag
      sleepmillis = millis();  // reset sleep timer
    }

    // check time passed since the handle was moved 检查把手被移动后经过的时间
    goneSeconds = (millis() - sleepmillis) / 1000;
    if ((!inSleepMode) && (time2sleep > 0) && (goneSeconds >= time2sleep)) {
      inSleepMode = true;
      beep();
    } else if ((!inOffMode) && (time2off > 0) &&
               ((goneSeconds / 60) >= time2off)) {
      inOffMode = true;
      Display_SetPowerSave(1);
      beep();
    }
  }
}

// reads temperature, vibration switch and supply voltages
// 读取温度，振动开关和电源电压
void SENSORCheck() {
  if (accel.available()) {
    accels[accelIndex][0] = accel.getRawX() + 32768;
    accels[accelIndex][1] = accel.getRawY() + 32768;
    accels[accelIndex][2] = accel.getRawZ() + 32768;
    accelIndex++;

    if (accelIndex >= ACCEL_SAMPLES) {
      accelIndex = 0;
      // cal variance
      uint64_t avg[3] = {0, 0, 0};
      for (int i = 0; i < ACCEL_SAMPLES; i++) {
        avg[0] += accels[i][0];
        avg[1] += accels[i][1];
        avg[2] += accels[i][2];
      }
      avg[0] /= ACCEL_SAMPLES;
      avg[1] /= ACCEL_SAMPLES;
      avg[2] /= ACCEL_SAMPLES;
      uint64_t var[3] = {0, 0, 0};
      for (int i = 0; i < ACCEL_SAMPLES; i++) {
        var[0] += (accels[i][0] - avg[0]) * (accels[i][0] - avg[0]);
        var[1] += (accels[i][1] - avg[1]) * (accels[i][1] - avg[1]);
        var[2] += (accels[i][2] - avg[2]) * (accels[i][2] - avg[2]);
      }
      var[0] /= ACCEL_SAMPLES;
      var[1] /= ACCEL_SAMPLES;
      var[2] /= ACCEL_SAMPLES;

      int varThreshold = WAKEUPthreshold * 10000;

      if (var[0] > varThreshold || var[1] > varThreshold ||
          var[2] > varThreshold) {
        handleMoved = true;
      }
    }
  }

  // #endif

  Heater_Off(); // shut off heater in order to measure temperature 关闭加热器以测量温度

  if (VoltageValue == 3) {
    delayMicroseconds(TIME2SETTLE_20V);
  } else {
    delayMicroseconds(TIME2SETTLE);  // wait for voltage to settle 等待电压稳定
  }
  double temp = denoiseAnalog(SENSOR_PIN);  // 读取ADC值的温度

  if (SensorCounter++ > 10) {
    Vin = getVIN();  // get Vin every now and then 时不时去获取VIN电压
    SensorCounter = 0;
  }

  if (!inLockMode) {
    limit = POWER_LIMIT_20;
    if (VoltageValue < 3) {
      limit = POWER_LIMIT_15;
    }
    Heater_SetPower(constrain(HEATER_PWM, 0, limit)); // turn on again heater 再次打开加热器
  }

  RawTemp += (temp - RawTemp) *
             SMOOTHIE;  // stabilize ADC temperature reading 稳定ADC温度读数
  calculateTemp();  // calculate real temperature value 计算实际温度值

  // stabilize displayed temperature when around setpoint
  // 稳定显示温度时，周围的设定值
  if ((ShowTemp != Setpoint) || (abs(ShowTemp - CurrentTemp) > 5))
    ShowTemp = CurrentTemp;
  if (abs(ShowTemp - Setpoint) <= 1) ShowTemp = Setpoint;
  if (inLockMode) {
    ShowTemp = 0;
  }

  // set state variable if temperature is in working range; beep if working
  // temperature was just reached
  // 温度在工作范围内可设置状态变量;当工作温度刚刚达到时，会发出蜂鸣声
  gap = abs(SetTemp - CurrentTemp);
  if (gap < 5) {
    if (!isWorky && beepIfWorky) beep();
    isWorky = true;
    beepIfWorky = false;
  } else
    isWorky = false;

  // checks if tip is present or currently inserted
  // 检查烙铁头是否存在或当前已插入
  if (ShowTemp > 500) TipIsPresent = false;  // tip removed ? 烙铁头移除？
  if (!TipIsPresent &&
      (ShowTemp < 500)) {  // new tip inserted ? 新的烙铁头插入？
    Heater_Off(); // shut off heater 关闭加热器
    beep();                                  // beep for info
    TipIsPresent = true;  // tip is present now 烙铁头已经存在
    ChangeTipScreen(TipName, CalTemp);  // show tip selection screen 显示烙铁头选择屏幕
    updateEEPROM();     // update setting in EEPROM EEPROM的更新设置
    handleMoved = true;  // reset all timers 重置所有计时器
    RawTemp = denoiseAnalog(
        SENSOR_PIN);  // restart temp smooth algorithm 重启临时平滑算法
    c0 = LOW;         // switch must be released 必须松开开关
    setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP,
              SetTemp);  // reset rotary encoder 重置旋转编码器
  }
}

// calculates real temperature value according to ADC reading and calibration
// values 根据ADC读数和校准值，计算出真实的温度值
void calculateTemp() {
  if (RawTemp < 200)
    CurrentTemp = map(RawTemp, 0, 200, 15, CalTemp[CurrentTip][0]);
  else if (RawTemp < 280)
    CurrentTemp =
        map(RawTemp, 200, 280, CalTemp[CurrentTip][0], CalTemp[CurrentTip][1]);
  else
    CurrentTemp =
        map(RawTemp, 280, 360, CalTemp[CurrentTip][1], CalTemp[CurrentTip][2]);
}

// controls the heater 控制加热器
void Thermostat() {
  // define Setpoint acoording to current working mode
  // 根据当前工作模式定义设定值
  if (inOffMode || inLockMode)
    Setpoint = 0;
  else if (inSleepMode)
    Setpoint = SleepTemp;
  else if (inBoostMode) {
    Setpoint = constrain(SetTemp + BoostTemp, 0, 450);
  } else
    Setpoint = SetTemp;

  if (SetTemp != DefaultTemp) {
    DefaultTemp = SetTemp;  // 把设置里面的默认温度也修改了
    update_default_temp_EEPROM();
  }

  // control the heater (PID or direct) 控制加热器(PID或直接)
  gap = abs(Setpoint - CurrentTemp);
  if (PIDenable) {
    Input = CurrentTemp;
    if (gap < 30)
      ctrl.SetTunings(consKp, consKi, consKd);
    else
      ctrl.SetTunings(aggKp, aggKi, aggKd);
    ctrl.Compute();
  } else {
    // turn on heater if current temperature is below setpoint
    // 如果当前温度低于设定值，则打开加热器
    if ((CurrentTemp + 0.5) < Setpoint)
      Output = 0;
    else
      Output = 255;
  }
  limit = POWER_LIMIT_20;
  if (VoltageValue < 3) {
    limit = POWER_LIMIT_15;
  } else if(VoltageValue == 3){
    limit = POWER_LIMIT_20_2;
  }
  Heater_SetPower(constrain((HEATER_PWM), 0, limit));   // set heater PWM 设置加热器PWM
}



// reads user settings from EEPROM; if EEPROM values are invalid, write defaults
// 从EEPROM读取用户设置;如果EEPROM值无效，则写入默认值
void getEEPROM() { read_EEPROM(); }

// writes user settings to EEPROM using updade function to minimize write cycles
// 使用升级功能将用户设置写入EEPROM，以最小化写入周期
void updateEEPROM() { update_EEPROM(); }

// 对32个ADC读数进行平均以降噪
//  VP+_Ru = 100k, Rd_GND = 1K
uint16_t denoiseAnalog(byte port) {
  uint32_t result = 0;
  float maxValue, minValue;
  int resultArray[8];

  for (uint8_t i = 0; i < 8; i++) {
    // get 32 readings and sort them 获取32个读数并对其进行排序
    float value, raw_adc;

    raw_adc = adc_sensor.readMiliVolts();
    value = constrain(0.4432 * raw_adc + 29.665, 20, 1000);

    resultArray[i] = value;
  }

  // sort resultArray with low time complexity
  // 用低时间复杂度对resultArray进行排序
  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = i + 1; j < 8; j++) {
      if (resultArray[i] > resultArray[j]) {
        int temp = resultArray[i];
        resultArray[i] = resultArray[j];
        resultArray[j] = temp;
      }
    }
  }

  // get the average of the middle 4 readings 获取中间20个读数的平均值
  for (uint8_t i = 2; i < 6; i++) {
    result += resultArray[i];
  }

  return (result / 4);  // devide by 32 and return value 除以32并返回值
}

int32_t variance(int16_t a[]) {
  // Compute mean (average of elements)计算平均值(元素的平均值)
  int32_t sum = 0;

  for (int i = 0; i < 32; i++) sum += a[i];
  int16_t mean = (int32_t)sum / 32;
  // Compute sum squared differences with mean.计算和平方差的平均值
  int32_t sqDiff = 0;
  for (int i = 0; i < 32; i++) sqDiff += (a[i] - mean) * (a[i] - mean);
  return (int32_t)sqDiff / 32;
}



void turnOffHeater(Button2 &b) { inOffMode = true; }

void heatWithLimit() {

  limit = 0;
  if (VoltageValue < 3) {
    limit = POWER_LIMIT_15;
  } else if (VoltageValue == 3) {
    limit = POWER_LIMIT_20;
  }
  Heater_SetPower(constrain(HEATER_PWM, 0, limit));  // turn on again heater 再次打开加热器
}