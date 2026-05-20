#pragma once

#include <Arduino.h>
#include "FlightFsm.h"
#include "Measurements.h"
#include "Telemetry.h"

class StateLogic {
 public:
  void tick(uint32_t nowMs,
            FlightFsm &fsm,
            const MeasurementSnapshot &measurement,
            TelemetryService &telemetry);

 private:
  void tickBeforeLaunch(uint32_t nowMs,
                        FlightFsm &fsm,
                        const MeasurementSnapshot &measurement,
                        TelemetryService &telemetry);
};
