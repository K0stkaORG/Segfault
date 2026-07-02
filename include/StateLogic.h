#pragma once

#include <Arduino.h>
#include "FlightFsm.h"
#include "Measurements.h"
#include "Telemetry.h"
#include "PersistentStore.h"
#include "ParachuteServo.h"

class StateLogic {
 public:
  void begin(PersistentStore &store, FlightFsm &fsm, MeasurementService &measurements);
  void tick(uint32_t nowMs,
            FlightFsm &fsm,
            const MeasurementSnapshot &measurement,
            TelemetryService &telemetry,
            ParachuteServo &parachute);

 private:
  void tickBeforeLaunch(uint32_t nowMs,
                        FlightFsm &fsm,
                        const MeasurementSnapshot &measurement,
                        TelemetryService &telemetry);
  void tickArmed(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement);
  void tickFlight(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement);
  void tickApogee(uint32_t nowMs, FlightFsm &fsm, ParachuteServo &parachute);

  PersistentStore *store_ = nullptr;
  MeasurementService *measurements_ = nullptr;
  FlightState previousState_ = FlightState::BeforeLaunch;

  uint32_t highAccelStartMs_ = 0;
  float maxAltitude_m_ = 0.0f;
};
