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

  FlightState loadFlightState();
  void saveFlightState(FlightState state);

  bool loadBaseline(SensorBaseline &baseline);
  void saveBaseline(const SensorBaseline &baseline);

  void saveElapsedFlightTime(uint32_t elapsedMs);
  uint32_t loadElapsedFlightTime();

 private:
  Preferences preferences_;
  bool ready_ = false;
};
