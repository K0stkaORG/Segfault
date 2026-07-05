#include "StateLogic.h"

#include "AvionicsConfig.h"

void StateLogic::begin(PersistentStore &store, FlightFsm &fsm, MeasurementService &measurements) {
  store_ = &store;
  measurements_ = &measurements;
  fsm.registerListener(this);
  
  FlightState initial = fsm.currentState();
  if (initial >= FlightState::Flight) {
    SensorBaseline savedBaseline;
    if (store_->loadBaseline(savedBaseline)) {
      measurements_->setBaseline(savedBaseline);
    }
    measurements_->setBaseliningEnabled(false);
    
    uint32_t loadedElapsedMs = store_->loadElapsedFlightTime();
    flightStartTimeMs_ = millis() - loadedElapsedMs;
    lastNvsWriteMs_ = millis();
  } else {
    measurements_->setBaseliningEnabled(true);
    flightStartTimeMs_ = 0;
    lastNvsWriteMs_ = 0;
  }
}

void StateLogic::onStateTransition(FlightState oldState, FlightState newState) {
  if (newState >= FlightState::Flight && oldState < FlightState::Flight) {
    if (measurements_ != nullptr && store_ != nullptr) {
      measurements_->setBaseliningEnabled(false);
      
      SensorBaseline baseline = measurements_->getBaseline();
      measurements_->setBaseline(baseline);
      store_->saveBaseline(baseline);

      flightStartTimeMs_ = millis();
      lastNvsWriteMs_ = millis();
      store_->saveElapsedFlightTime(0);
    }
  } else if (newState < FlightState::Flight && oldState >= FlightState::Flight) {
    if (measurements_ != nullptr) {
      measurements_->setBaseliningEnabled(true);
      highAccelStartMs_ = 0;
      maxAltitude_m_ = 0.0f;
      flightStartTimeMs_ = 0;
      lastNvsWriteMs_ = 0;
      if (store_ != nullptr) {
        store_->saveElapsedFlightTime(0);
      }
    }
  }
}

void StateLogic::tick(uint32_t nowMs,
                      FlightFsm &fsm,
                      const MeasurementSnapshot &measurement,
                      TelemetryService &telemetry,
                      ParachuteServo &parachute) {
  FlightState currentState = fsm.currentState();

  switch (currentState) {
    case FlightState::BeforeLaunch:
      tickBeforeLaunch(nowMs, fsm, measurement, telemetry);
      break;
    case FlightState::Armed:
      telemetry.setInterval(AvionicsConfig::BeforeLaunchHeartbeatIntervalMs);
      tickArmed(nowMs, fsm, measurement);
      break;
    case FlightState::Flight:
      telemetry.setInterval(AvionicsConfig::TelemetryIntervalMs);
      tickFlight(nowMs, fsm, measurement);
      break;
    case FlightState::ApogeeReached:
      telemetry.setInterval(AvionicsConfig::TelemetryIntervalMs);
      tickApogee(nowMs, fsm, parachute);
      break;
    case FlightState::ChuteDeployed:
      telemetry.setInterval(AvionicsConfig::TelemetryIntervalMs);
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

void StateLogic::tickArmed(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement) {
  /*
  // KY-024 Breakaway Wire Logic (Disabled due to sensor issues)
  if (!measurement.ky024.digital) { 
     fsm.setState(FlightState::Flight);
     return;
  }
  */

  if (measurements_ == nullptr) return;

  float accelZ = (measurement.imu.accelZ_g - measurements_->getBaseline().accelZOffset_g) * 9.80665f;
  
  if (accelZ > AvionicsConfig::LaunchAccelThresholdMps2) {
    if (highAccelStartMs_ == 0) {
      highAccelStartMs_ = nowMs;
    } else if (nowMs - highAccelStartMs_ > AvionicsConfig::LaunchAccelDurationMs) {
      fsm.setState(FlightState::Flight);
    }
  } else {
    highAccelStartMs_ = 0;
  }
}

void StateLogic::tickFlight(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement) {
  if (measurements_ == nullptr) return;

  if (measurement.aglAltitude_m > maxAltitude_m_) {
    maxAltitude_m_ = measurement.aglAltitude_m;
  }

  bool primaryTrigger = (measurement.aglAltitude_m < (maxAltitude_m_ - AvionicsConfig::ApogeeAltitudeDropM)) &&
                        (measurement.verticalVelocity_mps <= AvionicsConfig::ApogeeVelocityThresholdUpMps);

  uint32_t elapsedMs = nowMs - flightStartTimeMs_;

  if (nowMs - lastNvsWriteMs_ >= 1000) {
    lastNvsWriteMs_ = nowMs;
    if (store_ != nullptr) {
      store_->saveElapsedFlightTime(elapsedMs);
    }
  }

  bool backupTrigger = (elapsedMs > AvionicsConfig::ApogeeFailsafeTimerMs);

  if (primaryTrigger || backupTrigger) {
    fsm.setState(FlightState::ApogeeReached);
  }
}

void StateLogic::tickApogee(uint32_t nowMs, FlightFsm &fsm, ParachuteServo &parachute) {
  (void)nowMs;
  parachute.deploy();
  fsm.setState(FlightState::ChuteDeployed);
}
