#include "StateLogic.h"

#include "AvionicsConfig.h"

void StateLogic::begin(PersistentStore &store, FlightFsm &fsm, MeasurementService &measurements) {
  store_ = &store;
  measurements_ = &measurements;
  previousState_ = fsm.currentState();
  
  if (previousState_ >= FlightState::Flight) {
    SensorBaseline savedBaseline;
    if (store_->loadBaseline(savedBaseline)) {
      measurements_->setBaseline(savedBaseline);
    }
    measurements_->setBaseliningEnabled(false);
  } else {
    measurements_->setBaseliningEnabled(true);
  }
}

void StateLogic::tick(uint32_t nowMs,
                      FlightFsm &fsm,
                      const MeasurementSnapshot &measurement,
                      TelemetryService &telemetry) {
  FlightState currentState = fsm.currentState();

  if (currentState >= FlightState::Flight && previousState_ == FlightState::BeforeLaunch) {
    if (measurements_ != nullptr) {
      measurements_->setBaseliningEnabled(false);
      if (store_ != nullptr) {
        store_->saveBaseline(measurements_->getBaseline());
      }
    }
  }
  
  previousState_ = currentState;

  switch (currentState) {
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
