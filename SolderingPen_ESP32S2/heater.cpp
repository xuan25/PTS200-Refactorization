#include "heater.h"

#define CONTROL_PIN       5     // heater MOSFET PWM control 加热器MOSFET PWM控制
#define CONTROL_CHANNEL   2     // PWM channel
#define CONTROL_FREQ      200   // PWM frequency
#define CONTROL_FREQ_20V  1000  // PWM frequency for 20V
#define CONTROL_RES       8     // PWM resolution

void Heater_Init(uint8_t voltageValue) {
  if (voltageValue == 3) {
    ledcSetup(CONTROL_CHANNEL, CONTROL_FREQ_20V, CONTROL_RES);
  } else {
    ledcSetup(CONTROL_CHANNEL, CONTROL_FREQ, CONTROL_RES);
  }

  ledcAttachPin(CONTROL_PIN, CONTROL_CHANNEL);
  
  // Init the heater with off state
  // TODO: verify system status has synced
  Heater_Off();
}

void Heater_SetPower(uint32_t power) {
  ledcWrite(CONTROL_CHANNEL, power);
}

void Heater_Off() {
  Heater_SetPower(HEATER_OFF);
}
