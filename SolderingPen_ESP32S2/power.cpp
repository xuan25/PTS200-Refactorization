#include "power.h"

#include <QC3Control.h>

#include "heater.h"

#define PD_CFG_0          16
#define PD_CFG_1          17
#define PD_CFG_2          18

QC3Control QC(14, 13);

void Power_PDConfig(uint8_t voltageValue) {

  switch (voltageValue) {
    case 0: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, LOW);
      digitalWrite(PD_CFG_2, LOW);
    } break;
    case 1: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, LOW);
      digitalWrite(PD_CFG_2, HIGH);
    } break;
    case 2: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, HIGH);
    } break;
    case 3: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, LOW);
    } break;
    case 4: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, LOW);
    } break;
    default:
      break;
  }

  Heater_Init(voltageValue);
}

void Power_QCConfig(uint8_t voltageValue) {
  QC.begin();
  delay(100);
  switch (voltageValue) {
    case 0: {
      QC.set9V();
    } break;
    case 1: {
      QC.set12V();
    } break;
    case 2: {
      QC.set12V();
    } break;
    case 3: {
      QC.set20V();
    } break;
    case 4: {
      QC.set20V();
    } break;
    default:
      break;
  }
}

void Power_Init(bool isQCEnable, uint8_t voltageValue) {

  pinMode(PD_CFG_0, OUTPUT);
  pinMode(PD_CFG_1, OUTPUT);
  pinMode(PD_CFG_2, OUTPUT);

  digitalWrite(PD_CFG_0, LOW);
  digitalWrite(PD_CFG_1, HIGH);
  digitalWrite(PD_CFG_2, LOW);

  if (isQCEnable) {
    Power_QCConfig(voltageValue);
  }

  Power_PDConfig(voltageValue);
}