#ifndef __POWER_H
#define __POWER_H

#include <esp32-hal.h>

void Power_Init(bool isQCEnable, uint8_t voltageValue);
void Power_PDConfig(uint8_t voltageValue);

#endif