#include "PersistentStore.h"

#include "AvionicsConfig.h"

namespace {
constexpr const char *KeyFlightState = "state";
}  // namespace

bool PersistentStore::begin() {
  ready_ = preferences_.begin(AvionicsConfig::NvsNamespace, false);
  return ready_;
}

void PersistentStore::end() {
  if (ready_) {
    preferences_.end();
    ready_ = false;
  }
}

FlightState PersistentStore::loadFlightState() {
  if (!ready_) {
    return FlightState::BeforeLaunch;
  }

  const uint8_t rawState = preferences_.getUChar(
      KeyFlightState, static_cast<uint8_t>(FlightState::BeforeLaunch));
  if (!isValidFlightState(rawState)) {
    return FlightState::BeforeLaunch;
  }

  return static_cast<FlightState>(rawState);
}

void PersistentStore::saveFlightState(FlightState state) {
  if (ready_) {
    preferences_.putUChar(KeyFlightState, static_cast<uint8_t>(state));
  }
}

bool PersistentStore::loadBaseline(SensorBaseline &baseline) {
  if (!ready_) {
    return false;
  }

  baseline.groundAltitude_m = preferences_.getFloat("base_alt", 0.0f);
  baseline.accelZOffset_g = preferences_.getFloat("base_accz", 0.0f);
  return preferences_.isKey("base_alt");
}

void PersistentStore::saveBaseline(const SensorBaseline &baseline) {
  if (!ready_) {
    return;
  }

  preferences_.putFloat("base_alt", baseline.groundAltitude_m);
  preferences_.putFloat("base_accz", baseline.accelZOffset_g);
}

void PersistentStore::saveElapsedFlightTime(uint32_t elapsedMs) {
  if (ready_) {
    if (!AvionicsConfig::DisableFlightLogWrites) {
      preferences_.putUInt("fl_elapsed", elapsedMs);
    }
  }
}

uint32_t PersistentStore::loadElapsedFlightTime() {
  if (ready_) {
    return preferences_.getUInt("fl_elapsed", 0);
  }
  return 0;
}
