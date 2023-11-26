#ifndef __MENU_H
#define __MENU_H

#include <esp32-hal.h>
#include "config.h"

void Menu_Init(uint8_t hand_side);
void MainScreen(uint8_t language, double Setpoint, uint16_t ShowTemp, 
      bool inOffMode, bool inLockMode, bool inSleepMode, bool inBoostMode, 
      bool isWorky, double Output, uint8_t MainScrType, uint16_t Vin, 
      float newSENSORTmp);
void SetupScreen(uint16_t *SetTemp, uint8_t *MainScrType, uint8_t *VoltageValue, 
      bool *QCEnable, bool *beepEnable, bool *restore_default_config, 
      uint8_t *language, uint8_t *hand_side, 
      char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void ChangeTipScreen(char TipName[TIPMAX][TIPNAMELENGTH], uint16_t CalTemp[TIPMAX][4]);
void Display_SetPowerSave(uint8_t isEnable);

#endif