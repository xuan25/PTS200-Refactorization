#ifndef __UTILS_H
#define __UTILS_H

#define BUTTON_PRESSED LOW
#define BUTTON_RELEASED HIGH

#include <esp32-hal.h>

int readButtonM();

int readButtonP();

int readButtonN();



void beep();

void initBuzzer();

void initButton();


void initRotary();
void setRotary(int rmin, int rmax, int rstep, int rvalue);
int getRotary();

void Vin_Init();
// get supply voltage in mV 得到以mV为单位的电源电压
uint16_t getVIN();

#endif