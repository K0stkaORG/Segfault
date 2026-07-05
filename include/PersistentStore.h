#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "FlightFsm.h"

struct SensorBaseline {
  float groundAltitude_m = 0.0f;
  float accelZOffset_g = 0.0f;
};

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

  bool loadBaseline(SensorBaseline &baseline);
  void saveBaseline(const SensorBaseline &baseline);

  void saveElapsedFlightTime(uint32_t elapsedMs);
  uint32_t loadElapsedFlightTime();

 private:
  Preferences preferences_;
  bool ready_ = false;
  uint32_t bootCount_ = 0;
};
