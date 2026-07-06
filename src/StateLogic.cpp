#include "StateLogic.h"
#include "AvionicsConfig.h"
#include "FlightLogger.h"

void StateLogic::begin(PersistentStore &store, FlightFsm &fsm, MeasurementService &measurements, FlightLogger &logger) {
  store_ = &store;
  measurements_ = &measurements;
  logger_ = &logger;
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
  lastLogWriteMs_ = 0;
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

  // Erase partition exactly once upon transitioning to Armed state
  if (newState == FlightState::Armed) {
    if (logger_ != nullptr) {
      logger_->erase();
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
      tickApogee(nowMs, fsm, measurement, parachute);
      break;
    case FlightState::ChuteDeployed:
      telemetry.setInterval(AvionicsConfig::TelemetryIntervalMs);
      tickChuteDeployed(nowMs, fsm, measurement);
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
  if (measurements_ == nullptr) return;

  // Logging condition 1: Armed mode AND accelZ > 1g above baseline
  float compensatedAccelZ = (measurement.imu.accelZ_g - measurements_->getBaseline().accelZOffset_g);
  if (compensatedAccelZ > AvionicsConfig::FlightLogArmedAccelThresholdG) {
    tryLog(nowMs, fsm, measurement);
  }

  float accelZ = compensatedAccelZ * 9.80665f;
  
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
  // Logging condition 2: Flight mode (unconditional)
  tryLog(nowMs, fsm, measurement);

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

void StateLogic::tickApogee(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement, ParachuteServo &parachute) {
  // Logging condition 3: Apogee mode (unconditional)
  tryLog(nowMs, fsm, measurement);

  parachute.deploy();
  fsm.setState(FlightState::ChuteDeployed);
}

void StateLogic::tickChuteDeployed(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement) {
  if (measurements_ == nullptr) return;

  // Logging condition 4: Chute mode AND (accelZ > 1g OR accelZ < -1g)
  float compensatedAccelZ = (measurement.imu.accelZ_g - measurements_->getBaseline().accelZOffset_g);
  if (compensatedAccelZ > AvionicsConfig::FlightLogChuteAccelThresholdG || 
      compensatedAccelZ < -AvionicsConfig::FlightLogChuteAccelThresholdG) {
    tryLog(nowMs, fsm, measurement);
  }
}

void StateLogic::tryLog(uint32_t nowMs, FlightFsm &fsm, const MeasurementSnapshot &measurement) {
  if (logger_ == nullptr) return;
  if (nowMs - lastLogWriteMs_ >= AvionicsConfig::FlightLogIntervalMs) {
    lastLogWriteMs_ = nowMs;
    logger_->log(nowMs, fsm, measurement);
  }
}
