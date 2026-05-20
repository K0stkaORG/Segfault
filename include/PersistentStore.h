#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "FlightFsm.h"

class PersistentStore {
 public:
  bool begin();
  void end();

  uint32_t bootCount() const;
  FlightState loadFlightState();
  void saveFlightState(FlightState state);
  uint8_t loadPacketCounter();
  void savePacketCounter(uint8_t packetCounter);
  void saveInitStatus(bool bmp280Ok, bool bmi270Ok, bool gpsOk, bool loraOk);

 private:
  Preferences preferences_;
  bool ready_ = false;
  uint32_t bootCount_ = 0;
};
