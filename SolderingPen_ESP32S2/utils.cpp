#include "utils.h"

#include <esp32-hal.h>
#include <Arduino.h>

#define BUZZER_PIN        3     // buzzer 蜂鸣器
#define BUTTON_PIN        0     // switch 按键right
#define BUTTON_P_PIN      4     // 1 键位为“+”
#define BUTTON_N_PIN      2     // 2 键位为“-”

// 旋转编码器的类型
#define ROTARY_TYPE       0     // 0: 2 increments/step; 1: 4 increments/step (default)
#define BUTTON_DELAY      5



#define TEMP_MIN          50    // 最小温度
#define TEMP_MAX          450   // 最大温度


void Utils_Init() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_P_PIN, INPUT_PULLUP);
  pinMode(BUTTON_N_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}


volatile uint8_t a0, b0;

unsigned int Button_Time1 = 0, Button_Time2 = 0;

// creates a short beep on the buzzer 在蜂鸣器上创建一个短的哔哔声
void beep() {
  bool beepEnable = true;
  if (beepEnable) {
    for (uint8_t i = 0; i < 255; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(125);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(125);
    }
  }
}

volatile int count, countMin, countMax, countStep;

void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void initButton() {
  pinMode(BUTTON_P_PIN, INPUT_PULLUP);
  pinMode(BUTTON_N_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

int readButtonM() {
  return digitalRead(BUTTON_PIN);
}

int readButtonP() {
  return digitalRead(BUTTON_P_PIN);
}

int readButtonN() {
  return digitalRead(BUTTON_N_PIN);
}

void initRotary() {
  a0 = 0;
  b0 = 0;
}

// sets start values for rotary encoder 设置旋转编码器的起始值
void setRotary(int rmin, int rmax, int rstep, int rvalue) {
  countMin = rmin << ROTARY_TYPE;
  countMax = rmax << ROTARY_TYPE;
  countStep = rstep;
  count = rvalue << ROTARY_TYPE;
}

void Button_loop() {
  if (!digitalRead(BUTTON_N_PIN) && a0 == 1) {
    delay(BUTTON_DELAY);
    if (!digitalRead(BUTTON_N_PIN)) {
      int count0 = count;
      count = constrain(count + countStep, countMin, countMax);
      if (!(countMin == TEMP_MIN && countMax == TEMP_MAX)) {
        if (count0 + countStep > countMax) {
          count = countMin;
        }
      }
      a0 = 0;
    }
  } else if (!digitalRead(BUTTON_N_PIN) && a0 == 0) {
    delay(BUTTON_DELAY);
    if (Button_Time1 > 10)  // 这里的数越大，需要长按时间更长
      count = constrain(count + countStep, countMin, countMax);
    else
      Button_Time1++;
  } else if (digitalRead(BUTTON_N_PIN)) {
    Button_Time1 = 0;
    a0 = 1;
  }

  if (!digitalRead(BUTTON_P_PIN) && b0 == 1) {
    delay(BUTTON_DELAY);
    if (!digitalRead(BUTTON_P_PIN)) {
      int count0 = count;
      count = constrain(count - countStep, countMin, countMax);
      if (!(countMin == TEMP_MIN && countMax == TEMP_MAX)) {
        if (count0 - countStep < countMin) {
          count = countMax;
        }
      }
      b0 = 0;
    }
  } else if (!digitalRead(BUTTON_P_PIN) && b0 == 0) {
    delay(BUTTON_DELAY);
    if (Button_Time2 > 10)  // 这里的数越大，需要长按时间更长
      count = constrain(count - countStep, countMin, countMax);
    else
      Button_Time2++;
  } else if (digitalRead(BUTTON_P_PIN)) {
    Button_Time2 = 0;
    b0 = 1;
  }
}

// reads current rotary encoder value 读取当前旋转编码器值
int getRotary() {
  Button_loop();
  return (count >> ROTARY_TYPE);
}




#include <ESP32AnalogRead.h>  //Click here to get the library: http://librarymanager/All#ESP32AnalogRead

#define VIN_PIN           6     // input voltage sense 检测输入电压

ESP32AnalogRead adc_vin;

void Vin_Init() {
  adc_vin.attach(VIN_PIN);
}

// get supply voltage in mV 得到以mV为单位的电源电压
uint16_t getVIN() {
  long value;
  long voltage;
  long result = 0;

  for (uint8_t i = 0; i < 4; i++) {  // get 32 readings 得到32个读数
    //    long val = analogRead(VIN_PIN);
    long val = adc_vin.readMiliVolts();

    result += val;  // add them up 把它们加起来
  }

  value = (result / 4);

  voltage = value * 31.3;

  return voltage;
}