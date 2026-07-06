#pragma once

#include <Arduino.h>
#include "FlightFsm.h"
#include "Measurements.h"
#include "Telemetry.h"
#include "PersistentStore.h"
#include "ParachuteServo.h"

class FlightLogger;
class RecoveryService;

class StateLogic : public FlightFsmListener {
 public:
  void begin(PersistentStore &store,
             FlightFsm &fsm,
             MeasurementService &measurements,
             FlightLogger &logger,
             RecoveryService &recovery);
  void onStateTransition(FlightState oldState, FlightState newState) override;
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
  void tickApogee(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement, ParachuteServo &parachute);
  void tickChuteDeployed(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement);
  void tryLog(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement);

  PersistentStore *store_ = nullptr;
  MeasurementService *measurements_ = nullptr;
  FlightLogger *logger_ = nullptr;
  RecoveryService *recovery_ = nullptr;

  uint32_t highAccelStartMs_ = 0;
  float maxAltitude_m_ = 0.0f;
  uint32_t flightStartTimeMs_ = 0;
  uint32_t lastNvsWriteMs_ = 0;
  uint32_t lastLogWriteMs_ = 0;
};
