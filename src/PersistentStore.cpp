#include "PersistentStore.h"

#include "AvionicsConfig.h"

namespace {
constexpr const char *KeyBootCount = "boot";
constexpr const char *KeyFlightState = "state";
constexpr const char *KeyPacketCounter = "pkt";
constexpr const char *KeyInitStatus = "init";
}  // namespace

bool PersistentStore::begin() {
  ready_ = preferences_.begin(AvionicsConfig::NvsNamespace, false);
  if (!ready_) {
    bootCount_ = 0;
    return false;
  }

  bootCount_ = preferences_.getUInt(KeyBootCount, 0) + 1;
  preferences_.putUInt(KeyBootCount, bootCount_);
  return true;
}

void PersistentStore::end() {
  if (ready_) {
    preferences_.end();
    ready_ = false;
  }
}

uint32_t PersistentStore::bootCount() const {
  return bootCount_;
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

uint8_t PersistentStore::loadPacketCounter() {
  if (!ready_) {
    return 0;
  }

  return preferences_.getUChar(KeyPacketCounter, 0);
}

void PersistentStore::savePacketCounter(uint8_t packetCounter) {
  if (ready_) {
    preferences_.putUChar(KeyPacketCounter, packetCounter);
  }
}

void PersistentStore::saveInitStatus(bool bmp280Ok,
                                     bool bmi270Ok,
                                     bool gpsOk,
                                     bool loraOk) {
  if (!ready_) {
    return;
  }

  uint8_t status = 0;
  if (bmp280Ok) {
    status |= 1U << 0;
  }
  if (bmi270Ok) {
    status |= 1U << 1;
  }
  if (gpsOk) {
    status |= 1U << 2;
  }
  if (loraOk) {
    status |= 1U << 3;
  }

  preferences_.putUChar(KeyInitStatus, status);
}
