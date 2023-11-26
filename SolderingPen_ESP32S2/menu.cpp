#include "menu.h"

#include <U8g2lib.h>  // https://github.com/olikraus/u8g2
// font
#include "PTS200_16.h"

#include "Languages.h"

// Type of OLED Controller
// #define SSD1306
#define SH1107
//typedef u8g2_uint_t u8g_uint_t;
#define SCREEN_OFFSET     3

// Setup u8g object depending on OLED controller
// 根据OLED控制器设置u8g对象
#if defined(SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE,
                                         /* clock=*/22, /* data=*/21);
#elif defined(SH1107)
U8G2_SH1107_64X128_F_HW_I2C u8g2(U8G2_R1, 7);
#else
#error Wrong OLED controller type!
#endif



#include "utils.h"



#include "power.h"
//
#include "FirmwareMSC.h"
#include "UtilsEEPROM.h"
#include "accel.h"

#include "config.h"

// MSC Firmware
FirmwareMSC MSC_Update;
bool MSC_Updating_Flag = false;

uint8_t SENSORTmpTime = 0;
float lastSENSORTmp = 0;
double ChipTemp;



#define BUTTON_PIN        0     // switch 按键right




static void usbEventCallback(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(0, 10,
                     "USB PLUGGED");  // write something to the internal memory
        u8g2.sendBuffer();            // transfer internal memory to the display
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(
            0, 10, "USB UNPLUGGED");  // write something to the internal memory
        u8g2.sendBuffer();            // transfer internal memory to the display
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(
            0, 10, "USB SUSPENDED");  // write something to the internal memory
        u8g2.sendBuffer();            // transfer internal memory to the display
        break;
      case ARDUINO_USB_RESUME_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(0, 10,
                     "USB RESUMED");  // write something to the internal memory
        u8g2.sendBuffer();            // transfer internal memory to the display
        break;

      default:
        break;
    }
  } else if (event_base == ARDUINO_FIRMWARE_MSC_EVENTS) {
    switch (event_id) {
      case ARDUINO_FIRMWARE_MSC_START_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(
            0, 10,
            "MSC Update Start");  // write something to the internal memory
        u8g2.sendBuffer();        // transfer internal memory to the display
        break;
      case ARDUINO_FIRMWARE_MSC_WRITE_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(0, 10,
                     "MSC Updating");  // write something to the internal memory
        u8g2.sendBuffer();  // transfer internal memory to the display
        break;
      case ARDUINO_FIRMWARE_MSC_END_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(
            0, 10, "MSC Update End");  // write something to the internal memory
        u8g2.sendBuffer();  // transfer internal memory to the display
        break;
      case ARDUINO_FIRMWARE_MSC_ERROR_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(
            0, 10,
            "MSC Update ERROR!");  // write something to the internal memory
        u8g2.sendBuffer();         // transfer internal memory to the display
        break;
      case ARDUINO_FIRMWARE_MSC_POWER_EVENT:
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(
            0, 10,
            "MSC Update Power");  // write something to the internal memory
        u8g2.sendBuffer();        // transfer internal memory to the display
        break;

      default:
        break;
    }
  }
}


void Display_SetPowerSave(uint8_t isEnable){
  u8g2.setPowerSave(0);
}

void Display_Init(uint8_t hand_side) {
  u8g2.initDisplay();
  u8g2.begin();
  u8g2.enableUTF8Print();
  if(hand_side){
    u8g2.setDisplayRotation(U8G2_R3);
  }else{
    u8g2.setDisplayRotation(U8G2_R1);
  }
}

void Menu_Init(uint8_t hand_side) {
  ChipTemp = getChipTemp();
  lastSENSORTmp = getMPUTemp();
  Display_Init(hand_side);
}


// draws the main screen 绘制主屏幕
void MainScreen(uint8_t language, double Setpoint, uint16_t ShowTemp, 
      bool inOffMode, bool inLockMode, bool inSleepMode, bool inBoostMode, 
      bool isWorky, double Output, uint8_t MainScrType, uint16_t Vin, 
      float newSENSORTmp) {
  u8g2.firstPage();
  do {
    //  draw setpoint temperature
    u8g2.setFont(PTS200_16);
    u8g2.setFontPosTop();
    u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_set_temp[language]);
    u8g2.setCursor(40, 0 + SCREEN_OFFSET);
    u8g2.print(Setpoint, 0);

    // draw status of heater 绘制加热器状态
    u8g2.setCursor(96, 0 + SCREEN_OFFSET);
    if (ShowTemp > 500)
      u8g2.print(txt_error[language]);
    else if (inOffMode || inLockMode)
      u8g2.print(txt_off[language]);
    else if (inSleepMode)
      u8g2.print(txt_sleep[language]);
    else if (inBoostMode)
      u8g2.print(txt_boost[language]);
    else if (isWorky)
      u8g2.print(txt_worky[language]);
    else if (Output < 180)
      u8g2.print(txt_on[language]);
    else
      u8g2.print(txt_hold[language]);

    // rest depending on main screen type 休息取决于主屏幕类型
    if (MainScrType) {
      // draw current tip and input voltage 绘制当前烙铁头及输入电压
      float fVin = (float)Vin / 1000;  // convert mv in V
      newSENSORTmp = newSENSORTmp + 0.01 * getMPUTemp();
      SENSORTmpTime++;
      if (SENSORTmpTime >= 100) {
        lastSENSORTmp = newSENSORTmp;
        newSENSORTmp = 0;
        SENSORTmpTime = 0;
      }
      u8g2.setCursor(0, 50);
      u8g2.print(lastSENSORTmp, 1);
      u8g2.print(F("C"));
      u8g2.setCursor(83, 50);
      u8g2.print(fVin, 1);
      u8g2.print(F("V"));
      // draw current temperature 绘制当前温度
      u8g2.setFont(u8g2_font_freedoomr25_tn);
      u8g2.setFontPosTop();
      u8g2.setCursor(37, 18);
      if (ShowTemp > 500)
        u8g2.print(F("000"));
      else
        u8g2.printf("%03d", ShowTemp);
    } else {
      // draw current temperature in big figures 用大数字绘制当前温度
      u8g2.setFont(u8g2_font_fub42_tn);
      u8g2.setFontPosTop();
      u8g2.setCursor(15, 20);
      if (ShowTemp > 500)
        u8g2.print(F("000"));
      else
        u8g2.printf("%03d", ShowTemp);
    }
  } while (u8g2.nextPage());
}

void TipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void TempScreen();
void TimerScreen();
uint8_t MenuScreen(const char *Items[][language_types], uint8_t numberOfItems,
                   uint8_t selected);
uint8_t TipsManuScreen(const char *Items[][language_types], uint8_t numberOfItems,
                   uint8_t selected, char TipName[TIPMAX][TIPNAMELENGTH]);
void MessageScreen();
uint16_t InputScreen(const char *Items[][language_types]);
void InfoScreen();
void ChangeTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void CalibrationScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void InputNameScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void DeleteTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void AddTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);

// setup screen 设置屏幕
void SetupScreen(uint16_t *SetTemp, uint8_t *MainScrType, uint8_t *VoltageValue, 
      bool *QCEnable, bool *beepEnable, bool *restore_default_config, 
      uint8_t *language, uint8_t *hand_side, 
      char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {
  
  uint16_t SaveSetTemp = *SetTemp;
  uint8_t selection = 0;
  bool repeat = true;

  while (repeat) {
    selection = MenuScreen(SetupItems, sizeof(SetupItems), selection);
    switch (selection) {
      case 0: {
        TipScreen(TipName, CalTemp);
        repeat = false;
      } break;
      case 1: {
        TempScreen();
      } break;
      case 2: {
        TimerScreen();
      } break;
      case 3: {
        *MainScrType = MenuScreen(MainScreenItems, sizeof(MainScreenItems), *MainScrType);
      } break;
      case 4: {
        InfoScreen();
      } break;
      case 5:
        *VoltageValue = MenuScreen(VoltageItems, sizeof(VoltageItems), *VoltageValue);
        Power_PDConfig(*VoltageValue);
        break;
      case 6:
        *QCEnable = MenuScreen(QCItems, sizeof(QCItems), *QCEnable);
        break;
      case 7:
        *beepEnable = MenuScreen(BuzzerItems, sizeof(BuzzerItems), *beepEnable);
        break;
      case 8: {
        *restore_default_config = MenuScreen(DefaultItems, sizeof(DefaultItems), *restore_default_config);
        if (restore_default_config) {
          *restore_default_config = false;
          write_default_EEPROM();
          read_EEPROM();
        }
      } break;
      case 9: {
        bool lastbutton = (!digitalRead(BUTTON_PIN));
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(0, 10,
                     "MSC Update");  // write something to the internal memory
        u8g2.sendBuffer();           // transfer internal memory to the display
        delay(1000);
        do {
          MSC_Update.onEvent(usbEventCallback);
          MSC_Update.begin();
          if (lastbutton && digitalRead(BUTTON_PIN)) {
            delay(10);
            lastbutton = false;
          }
        } while (digitalRead(BUTTON_PIN) || lastbutton);

        MSC_Update.end();
      } break;
      case 10: {
        Serial.println(*language);
        *language = MenuScreen(LanguagesItems, sizeof(LanguagesItems), *language);
        Serial.println(*language);
        repeat = false;
      } break;
      case 11: {
        if(*hand_side == 0){
          u8g2.setDisplayRotation(U8G2_R3);
          *hand_side = 1;
        }else{
          u8g2.setDisplayRotation(U8G2_R1);
          *hand_side = 0;
        }
        repeat = false;
      } break;
      default:
        repeat = false;
        break;
    }
  }
  *SetTemp = SaveSetTemp;
}

// tip settings screen 烙铁头设置屏幕
void TipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat) {
    selection = TipsManuScreen(TipItems, sizeof(TipItems), selection, TipName);
    switch (selection) {
      case 0:
        ChangeTipScreen(TipName, CalTemp);
        break;
      case 1:
        CalibrationScreen(TipName, CalTemp);
        break;
      case 2:
        InputNameScreen(TipName, CalTemp);
        break;
      case 3:
        DeleteTipScreen(TipName, CalTemp);
        break;
      case 4:
        AddTipScreen(TipName, CalTemp);
        break;
      default:
        repeat = false;
        break;
    }
  }
}

// temperature settings screen 温度设置屏幕
void TempScreen() {
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat) {
    selection = MenuScreen(TempItems, sizeof(TempItems), selection);
    switch (selection) {
      case 0:
        setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
        DefaultTemp = InputScreen(DefaultTempItems);
        break;
      case 1:
        setRotary(50, TEMP_MAX, TEMP_STEP, SleepTemp);
        SleepTemp = InputScreen(SleepTempItems);
        break;
      case 2:
        setRotary(10, 100, TEMP_STEP, BoostTemp);
        BoostTemp = InputScreen(BoostTempItems);
        break;
      default:
        repeat = false;
        break;
    }
  }
}

// timer settings screen 定时器设置屏幕
void TimerScreen() {
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat) {
    selection = MenuScreen(TimerItems, sizeof(TimerItems), selection);
    switch (selection) {
      case 0:
        setRotary(0, 600, 10, time2sleep);
        time2sleep = InputScreen(SleepTimerItems);
        break;
      case 1:
        setRotary(0, 60, 1, time2off);
        time2off = InputScreen(OffTimerItems);
        break;
      case 2:
        setRotary(0, 180, 10, timeOfBoost);
        timeOfBoost = InputScreen(BoostTimerItems);
        break;
      case 3:
        setRotary(0, 50, 5, WAKEUPthreshold);
        WAKEUPthreshold = InputScreen(WAKEUPthresholdItems);
        break;
      default:
        repeat = false;
        break;
    }
  }
}

// menu screen 菜单屏幕
uint8_t MenuScreen(const char *Items[][language_types], uint8_t numberOfItems,
                   uint8_t selected) {
  // Serial.println(numberOfItems);
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected) arrow = 1;
  numberOfItems = numberOfItems / language_types;
  numberOfItems >>= 2;

  // 根据OLED控制器设置选择方向
#if defined(SSD1306)
  setRotary(0, numberOfItems + 3, 1, selected);
#elif defined(SH1107)
  setRotary(0, numberOfItems - 2, 1, selected);
#else
#error Wrong OLED controller type!
#endif

  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++) {
        uint8_t drawnumber = selected + i + 1 - arrow;
        if (drawnumber < numberOfItems)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET,
                        Items[selected + i + 1 - arrow][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return selected;
}

uint8_t TipsManuScreen(const char *Items[][language_types], uint8_t numberOfItems,
                   uint8_t selected, char TipName[TIPMAX][TIPNAMELENGTH]) {
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected) arrow = 1;
  numberOfItems = numberOfItems / language_types;
  numberOfItems >>= 2;

  // 根据OLED控制器设置选择方向
#if defined(SSD1306)
  setRotary(0, numberOfItems + 3, 1, selected);
#elif defined(SH1107)
  setRotary(0, numberOfItems - 2, 1, selected);
#else
#error Wrong OLED controller type!
#endif

  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      u8g2.drawUTF8(54, 0 + SCREEN_OFFSET, TipName[CurrentTip]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++) {
        uint8_t drawnumber = selected + i + 1 - arrow;
        if (drawnumber < numberOfItems)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET,
                        Items[selected + i + 1 - arrow][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return selected;
}

void MessageScreen(const char *Items[][language_types], uint8_t numberOfItems) {
  numberOfItems = numberOfItems / language_types;
  bool lastbutton = (!digitalRead(BUTTON_PIN));
  u8g2.firstPage();
  do {
    u8g2.setFont(PTS200_16);
    u8g2.setFontPosTop();
    for (uint8_t i = 0; i < numberOfItems; i++)
      u8g2.drawUTF8(0, i * 16, Items[i][language]);
  } while (u8g2.nextPage());
  do {
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);
  beep();
}

// input value screen 输入值屏幕
uint16_t InputScreen(const char *Items[][language_types]) {
  uint16_t value;
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    value = getRotary();
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      u8g2.setCursor(0, 32);
      u8g2.print(">");
      u8g2.setCursor(10, 32);
      if (value == 0)
        u8g2.print(txt_Deactivated[language]);
      else {
        u8g2.print(value);
        u8g2.print(" ");
        u8g2.print(Items[1][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return value;
}

// information display screen 信息显示屏幕
void InfoScreen() {
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    uint16_t Vin = getVIN();                  // read supply voltage
    float fVin = (float)Vin / 1000;  // convert mv in V
    float fTmp = getChipTemp();      // read cold junction temperature
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
      u8g2.setFontPosTop();
      u8g2.setCursor(0, 0 + SCREEN_OFFSET);
      u8g2.print(txt_temp[language]);
      u8g2.print(fTmp, 1);
      u8g2.print(F(" C"));
      u8g2.setCursor(0, 16 + SCREEN_OFFSET);
      u8g2.print(txt_voltage[language]);
      u8g2.print(fVin, 1);
      u8g2.print(F(" V"));
      u8g2.setCursor(0, 16 * 2 + SCREEN_OFFSET);
      u8g2.print(txt_Version[language]);
      u8g2.print(VERSION);
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
}

// change tip screen 改变烙铁头屏幕
void ChangeTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {
  uint8_t selected = CurrentTip;
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected) arrow = 1;
  setRotary(0, NumberOfTips - 1, 1, selected);
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  Serial.print("selected: ");
  Serial.println(selected);
  Serial.print("lastselected: \n");
  Serial.println(lastselected);
  Serial.print("NumberOfTips: \n");
  Serial.println(NumberOfTips);

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
      u8g2.setFontPosTop();
      //      strcpy_P(F_Buffer, PSTR("选择烙铁头"));
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_select_tip[language]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++) {
        uint8_t drawnumber = selected + i - arrow;
        if (drawnumber < NumberOfTips)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET,
                        TipName[selected + i - arrow]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  CurrentTip = selected;
}

// temperature calibration screen 温度校准屏幕
void CalibrationScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {

  // WIP

  // uint16_t CalTempNew[4];
  // uint16_t tempSetTemp = SetTemp;
  // for (uint8_t CalStep = 0; CalStep < 3; CalStep++) {
  //   SetTemp = CalTemp[CurrentTip][CalStep];
  //   Serial.print("SetTemp: ");
  //   Serial.println(SetTemp);
  //   setRotary(100, 500, 1, SetTemp);
  //   beepIfWorky = true;
  //   bool lastbutton = (!digitalRead(BUTTON_PIN));

  //   do {
  //     SENSORCheck();  // reads temperature and vibration switch of the iron
  //                     // 读取烙铁头的温度和振动开关
  //     Thermostat();   // heater control

  //     u8g2.firstPage();
  //     do {
  //       u8g2.setFont(PTS200_16);
  //       u8g2.setFontPosTop();
  //       //        strcpy_P(F_Buffer, PSTR("校准"));
  //       u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_calibrate[language]);
  //       u8g2.setCursor(0, 16 + SCREEN_OFFSET);
  //       u8g2.print(txt_step[language]);
  //       u8g2.print(CalStep + 1);
  //       u8g2.print(" of 3");
  //       if (isWorky) {
  //         u8g2.setCursor(0, 32 + SCREEN_OFFSET);
  //         u8g2.print(txt_set_measured[language]);
  //         u8g2.setCursor(0, 48 + SCREEN_OFFSET);
  //         u8g2.print(txt_s_temp[language]);
  //         u8g2.print(getRotary());
  //       } else {
  //         u8g2.setCursor(0, 32 + SCREEN_OFFSET);
  //         u8g2.print(txt_temp_2[language]);
  //         u8g2.print(uint16_t(RawTemp));
  //         u8g2.setCursor(0, 48 + SCREEN_OFFSET);
  //         u8g2.print(txt_wait_pls[language]);
  //       }
  //     } while (u8g2.nextPage());
  //     if (lastbutton && digitalRead(BUTTON_PIN)) {
  //       delay(10);
  //       lastbutton = false;
  //     }
  //   } while (digitalRead(BUTTON_PIN) || lastbutton);

  //   CalTempNew[CalStep] = getRotary();
  //   beep();
  //   delay(10);
  // }

  // Heater_Off();  // shut off heater 关闭加热器
  // if (VoltageValue == 3) {
  //   delayMicroseconds(TIME2SETTLE_20V);
  // } else {
  //   delayMicroseconds(TIME2SETTLE);  // wait for voltage to settle 等待电压稳定
  // }
  // CalTempNew[3] = getChipTemp();  // read chip temperature 读芯片温度
  // if ((CalTempNew[0] + 10 < CalTempNew[1]) &&
  //     (CalTempNew[1] + 10 < CalTempNew[2])) {
  //   if (MenuScreen(StoreItems, sizeof(StoreItems), 0)) {
  //     for (uint8_t i = 0; i < 4; i++) CalTemp[CurrentTip][i] = CalTempNew[i];
  //   }
  // }

  // SetTemp = tempSetTemp;
  // update_EEPROM();
}

// input tip name screen 输入烙铁头名字屏幕
void InputNameScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {
  uint8_t value;

  for (uint8_t digit = 0; digit < (TIPNAMELENGTH - 1); digit++) {
    bool lastbutton = (!digitalRead(BUTTON_PIN));
    setRotary(31, 96, 1, 65);
    do {
      value = getRotary();
      if (value == 31) {
        value = 95;
        setRotary(31, 96, 1, 95);
      }
      if (value == 96) {
        value = 32;
        setRotary(31, 96, 1, 32);
      }
      u8g2.firstPage();
      do {
        u8g2.setFont(PTS200_16);
        u8g2.setFontPosTop();
        u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_enter_tip_name[language]);
        u8g2.setCursor(12 * digit, 48 + SCREEN_OFFSET);
        u8g2.print(char(94));
        u8g2.setCursor(0, 32 + SCREEN_OFFSET);
        for (uint8_t i = 0; i < digit; i++) u8g2.print(TipName[CurrentTip][i]);
        u8g2.setCursor(12 * digit, 32 + SCREEN_OFFSET);
        u8g2.print(char(value));
      } while (u8g2.nextPage());
      if (lastbutton && digitalRead(BUTTON_PIN)) {
        delay(10);
        lastbutton = false;
      }
    } while (digitalRead(BUTTON_PIN) || lastbutton);
    TipName[CurrentTip][digit] = value;
    beep();
    delay(10);
  }
  TipName[CurrentTip][TIPNAMELENGTH - 1] = 0;
  return;
}

// delete tip screen 删除烙铁头屏幕
void DeleteTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {
  if (NumberOfTips == 1) {
    MessageScreen(DeleteMessage, sizeof(DeleteMessage));
  } else if (MenuScreen(SureItems, sizeof(SureItems), 0)) {
    if (CurrentTip == (NumberOfTips - 1)) {
      CurrentTip--;
    } else {
      for (uint8_t i = CurrentTip; i < (NumberOfTips - 1); i++) {
        for (uint8_t j = 0; j < TIPNAMELENGTH; j++)
          TipName[i][j] = TipName[i + 1][j];
        for (uint8_t j = 0; j < 4; j++) CalTemp[i][j] = CalTemp[i + 1][j];
      }
    }
    NumberOfTips--;
  }
}

// add new tip screen 添加新的烙铁头屏幕
void AddTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]) {
  if (NumberOfTips < TIPMAX) {
    CurrentTip = NumberOfTips++;
    InputNameScreen(TipName, CalTemp);
    CalTemp[CurrentTip][0] = TEMP200;
    CalTemp[CurrentTip][1] = TEMP280;
    CalTemp[CurrentTip][2] = TEMP360;
    CalTemp[CurrentTip][3] = TEMPCHP;
  } else
    MessageScreen(MaxTipMessage, sizeof(MaxTipMessage));
}



