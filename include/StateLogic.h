#pragma once

#include <Arduino.h>
#include "FlightFsm.h"
#include "Measurements.h"
#include "Telemetry.h"
#include "PersistentStore.h"

class StateLogic {
 public:
  void begin(PersistentStore &store, FlightFsm &fsm, MeasurementService &measurements);
  void tick(uint32_t nowMs,
            FlightFsm &fsm,
            const MeasurementSnapshot &measurement,
            TelemetryService &telemetry);

 private:
  void tickBeforeLaunch(uint32_t nowMs,
                        FlightFsm &fsm,
                        const MeasurementSnapshot &measurement,
                        TelemetryService &telemetry);

  PersistentStore *store_ = nullptr;
  MeasurementService *measurements_ = nullptr;
  FlightState previousState_ = FlightState::BeforeLaunch;
};
