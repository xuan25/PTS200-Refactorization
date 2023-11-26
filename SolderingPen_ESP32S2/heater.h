#ifndef __HEATER_H
#define __HEATER_H

#include <esp32-hal.h>

// Type of MOSFET
#define P_MOSFET // P_MOSFET or N_MOSFET

// MOSFET control definitions
#if defined(P_MOSFET)           // P-Channel MOSFET
#define HEATER_ON         255
#define HEATER_OFF        0
#define HEATER_PWM        255 - Output
#elif defined(N_MOSFET)         // N-Channel MOSFET
#define HEATER_ON         0
#define HEATER_OFF        255
#define HEATER_PWM        Output
#else
#error Invalid MOSFET type!
#endif

void Heater_Init(uint8_t voltageValue);
void Heater_SetPower(uint32_t power);
void Heater_Off();

#endif