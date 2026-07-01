#pragma once

#include <RadioLib.h>

extern SX1262 radio;
extern volatile bool loRaInterruptFired;

void IRAM_ATTR onLoRaInterrupt();
