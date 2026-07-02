#pragma once

#include <Arduino.h>

class PersistentStore;

enum class FlightState : uint8_t {
  BeforeLaunch = 0,
  Armed,
  Flight,
  ApogeeReached,
  ChuteDeployed,
};

class FlightFsm {
 public:
  explicit FlightFsm(FlightState initialState = FlightState::BeforeLaunch);

  void attachPersistentStore(PersistentStore &persistentStore);
  FlightState currentState() const;
  void setState(FlightState state);
  uint8_t stateFlags() const;
  const char *stateName() const;

 private:
  FlightState state_;
  PersistentStore *persistentStore_ = nullptr;
};

bool isValidFlightState(uint8_t rawState);
const char *flightStateName(FlightState state);
