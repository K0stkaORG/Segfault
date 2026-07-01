#include "LoRaRadio.h"
#include "AvionicsConfig.h"

SX1262 radio = new Module(AvionicsConfig::LoRaCsPin,
                          AvionicsConfig::LoRaDio1Pin,
                          AvionicsConfig::LoRaResetPin,
                          AvionicsConfig::LoRaBusyPin);

volatile bool loRaInterruptFired = false;

void IRAM_ATTR onLoRaInterrupt() {
  loRaInterruptFired = true;
}
