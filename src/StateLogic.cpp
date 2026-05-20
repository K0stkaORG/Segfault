#include "StateLogic.h"

#include "AvionicsConfig.h"

void StateLogic::tick(uint32_t nowMs,
                      FlightFsm &fsm,
                      const MeasurementSnapshot &measurement,
                      TelemetryService &telemetry) {
  switch (fsm.currentState()) {
    case FlightState::BeforeLaunch:
      tickBeforeLaunch(nowMs, fsm, measurement, telemetry);
      break;
    case FlightState::RBFRemoved:
    case FlightState::Flight:
    case FlightState::ApogeeReached:
    case FlightState::ChuteDeployed:
      break;
  }
}

void StateLogic::tickBeforeLaunch(
    uint32_t nowMs,
    FlightFsm &fsm,
    const MeasurementSnapshot &measurement,
    TelemetryService &telemetry) {
  (void)nowMs;
  (void)fsm;
  (void)measurement;

  telemetry.setInterval(AvionicsConfig::BeforeLaunchHeartbeatIntervalMs);
}
