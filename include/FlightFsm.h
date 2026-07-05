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

class FlightFsmListener {
 public:
  virtual ~FlightFsmListener() = default;
  virtual void onStateTransition(FlightState oldState, FlightState newState) = 0;
};

class FlightFsm {
 public:
  explicit FlightFsm(FlightState initialState = FlightState::BeforeLaunch);

  void attachPersistentStore(PersistentStore &persistentStore);
  void registerListener(FlightFsmListener *listener);
  FlightState currentState() const;
  void setState(FlightState state);
  uint8_t stateFlags() const;
  const char *stateName() const;

 private:
  FlightState state_;
  PersistentStore *persistentStore_ = nullptr;
  FlightFsmListener *listener_ = nullptr;
};

bool isValidFlightState(uint8_t rawState);
const char *flightStateName(FlightState state);
