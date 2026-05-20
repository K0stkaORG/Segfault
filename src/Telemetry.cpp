#include "Telemetry.h"

#include <LoRa.h>
#include <SPI.h>
#include "AvionicsConfig.h"

volatile bool TelemetryService::txInProgress_ = false;

bool TelemetryService::begin() {
  SPI.begin(AvionicsConfig::LoRaSckPin,
            AvionicsConfig::LoRaMisoPin,
            AvionicsConfig::LoRaMosiPin,
            AvionicsConfig::LoRaCsPin);

  LoRa.setPins(AvionicsConfig::LoRaCsPin,
               AvionicsConfig::LoRaResetPin,
               AvionicsConfig::LoRaDio0Pin);

  ready_ = LoRa.begin(AvionicsConfig::LoRaFrequencyHz);
  if (ready_) {
    LoRa.onTxDone(TelemetryService::onTxDone);
    LoRa.setSyncWord(AvionicsConfig::LoRaSyncWord);
    LoRa.idle();
  }

  txInProgress_ = false;
  enabled_ = false;
  intervalMs_ = 0;
  nextTelemetryAtMs_ = 0;
  return ready_;
}

void TelemetryService::setPacketCounter(uint8_t packetCounter) {
  packetCounter_ = packetCounter;
}

uint8_t TelemetryService::packetCounter() const {
  return packetCounter_;
}

void TelemetryService::setInterval(uint32_t intervalMs) {
  if (enabled_ && intervalMs_ == intervalMs) {
    return;
  }

  intervalMs_ = intervalMs;
  enabled_ = intervalMs_ > 0;
  nextTelemetryAtMs_ = 0;
}

void TelemetryService::disable() {
  enabled_ = false;
}

void TelemetryService::tick(uint32_t nowMs,
                            const FlightFsm &fsm,
                            const MeasurementSnapshot &measurement,
                            PersistentStore &persistentStore) {
  if (!timeoutFired(nowMs)) {
    return;
  }

  RocketTelemetry packet = TelemetryService::buildPacket(nowMs, fsm, measurement);
  packet.packetId = packetCounter_;

  const bool sent = send(packet);
  if (sent) {
    persistentStore.savePacketCounter(packetCounter_);
  }

  nextTelemetryAtMs_ = nowMs + intervalMs_;
}

bool TelemetryService::send(const RocketTelemetry &packet) {
  if (!ready_ || txInProgress_) {
    return false;
  }

  if (!LoRa.beginPacket()) {
    return false;
  }

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  const size_t bytesWritten = LoRa.write(bytes, sizeof(packet));
  if (bytesWritten != sizeof(packet)) {
    return false;
  }

  txInProgress_ = true;

  if (!LoRa.endPacket(true)) {
    txInProgress_ = false;
    return false;
  }

  printBytes(bytes, sizeof(packet));
  packetCounter_ = packet.packetId + 1;
  return true;
}

bool TelemetryService::isReady() const {
  return ready_;
}

RocketTelemetry TelemetryService::buildPacket(
    uint32_t nowMs,
    const FlightFsm &fsm,
    const MeasurementSnapshot &measurement) {
  RocketTelemetry packet{};

  packet.timestampMs = static_cast<uint16_t>(nowMs);
  packet.packetId = 0;
  packet.stateFlags = fsm.stateFlags();

  packet.accelX = measurement.imu.accelX;
  packet.accelY = measurement.imu.accelY;
  packet.accelZ = measurement.imu.accelZ;
  packet.gyroX = measurement.imu.gyroX;
  packet.gyroY = measurement.imu.gyroY;
  packet.gyroZ = measurement.imu.gyroZ;

  packet.pressureScaled = scalePressure(measurement.pressurePa);
  packet.triboVoltage = measurement.triboAdc;
  packet.batteryVoltage = scaleBattery(measurement.batteryAdc);

  if (measurement.gps.locationValid) {
    packet.gpsLatOffset = scaleGpsOffset(
        measurement.gps.latitudeDeg - AvionicsConfig::BaseLatitudeDeg);
    packet.gpsLonOffset = scaleGpsOffset(
        measurement.gps.longitudeDeg - AvionicsConfig::BaseLongitudeDeg);
  }

  if (measurement.gps.altitudeValid) {
    packet.gpsAltMeters = clampInt16(measurement.gps.altitudeMeters);
  }

  return packet;
}

void TelemetryService::onTxDone() {
  txInProgress_ = false;
}

bool TelemetryService::timeoutFired(uint32_t nowMs) const {
  return enabled_ && static_cast<int32_t>(nowMs - nextTelemetryAtMs_) >= 0;
}

void TelemetryService::printBytes(const uint8_t *bytes, size_t length) {
  Serial.print(F("TX "));
  Serial.print(length);
  Serial.print(F(" bytes:"));

  for (size_t i = 0; i < length; ++i) {
    Serial.print(' ');
    if (bytes[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(bytes[i], HEX);
  }

  Serial.println();
}

uint16_t TelemetryService::scalePressure(float pressurePa) {
  if (pressurePa <= 0.0f) {
    return 0;
  }

  const float scaled = pressurePa / 2.0f;
  if (scaled >= 65535.0f) {
    return 65535;
  }

  return static_cast<uint16_t>(scaled + 0.5f);
}

uint8_t TelemetryService::scaleBattery(uint16_t rawBatteryAdc) {
  return static_cast<uint8_t>(rawBatteryAdc >> 4);
}

int16_t TelemetryService::scaleGpsOffset(double offsetDeg) {
  return clampInt16(offsetDeg * 100000.0);
}

int16_t TelemetryService::clampInt16(double value) {
  if (value > 32767.0) {
    return 32767;
  }
  if (value < -32768.0) {
    return -32768;
  }

  return static_cast<int16_t>(value);
}
