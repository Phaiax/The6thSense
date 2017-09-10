#pragma once

#include "Arduino.h"


//const uint8_t LED_BUILTIN = 13; // already defined by arduino
const uint8_t LED_FRONT = A4;
const uint8_t LED_RIGHT = A2;
const uint8_t LED_BACK = A3;
const uint8_t LED_LEFT = A5;
const uint8_t SWITCH = A1;

const uint8_t FRONT = 10;
const uint8_t RECHTS = 11;
const uint8_t BACK = 5;
const uint8_t LEFT = 6;

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define MONITOR_DELAY (250)

#define BNO055_I2C_ADDRESS (55)