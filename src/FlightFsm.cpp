#include "FlightFsm.h"

#include "PersistentStore.h"

FlightFsm::FlightFsm(FlightState initialState) : state_(initialState) {}

void FlightFsm::attachPersistentStore(PersistentStore &persistentStore) {
  persistentStore_ = &persistentStore;
}

FlightState FlightFsm::currentState() const {
  return state_;
}

void FlightFsm::setState(FlightState state) {
  state_ = state;

  if (persistentStore_ != nullptr) {
    persistentStore_->saveFlightState(state_);
  }
}

uint8_t FlightFsm::stateFlags() const {
  uint8_t flags = 0;

  if (state_ == FlightState::Flight ||
      state_ == FlightState::ApogeeReached ||
      state_ == FlightState::ChuteDeployed) {
    flags |= 1U << 0;
  }

  if (state_ == FlightState::ApogeeReached ||
      state_ == FlightState::ChuteDeployed) {
    flags |= 1U << 1;
  }

  if (state_ == FlightState::ChuteDeployed) {
    flags |= 1U << 2;
  }

  return flags;
}

const char *FlightFsm::stateName() const {
  return flightStateName(state_);
}

bool isValidFlightState(uint8_t rawState) {
  return rawState <= static_cast<uint8_t>(FlightState::ChuteDeployed);
}

const char *flightStateName(FlightState state) {
  switch (state) {
    case FlightState::BeforeLaunch:
      return "BeforeLaunch";
    case FlightState::RBFRemoved:
      return "RBFRemoved";
    case FlightState::Flight:
      return "Flight";
    case FlightState::ApogeeReached:
      return "ApogeeReached";
    case FlightState::ChuteDeployed:
      return "ChuteDeployed";
  }

  return "Unknown";
}
