#include "Telemetry.h"

#include <RadioLib.h>
#include <SPI.h>
#include <stdio.h>
#include "AvionicsConfig.h"
#include "GroundControl.h"

namespace {
SX1262 radio = new Module(AvionicsConfig::LoRaCsPin,
                          AvionicsConfig::LoRaDio1Pin,
                          AvionicsConfig::LoRaResetPin,
                          AvionicsConfig::LoRaBusyPin);

constexpr char kHexDigits[] = "0123456789ABCDEF";

void tryPrintStatus(const char *prefix, int16_t code) {
  if (!AvionicsConfig::EnableSerial) {
    return;
  }

  char line[96] = {};
  const int written = snprintf(line, sizeof(line), "%s %d\n", prefix, code);
  if (written <= 0) {
    return;
  }

  const size_t length = static_cast<size_t>(written);
  if (Serial.availableForWrite() < length) {
    return;
  }

  Serial.write(reinterpret_cast<const uint8_t *>(line), length);
}
}  // namespace

volatile bool TelemetryService::txInProgress_ = false;
volatile bool TelemetryService::txDone_ = false;
volatile bool TelemetryService::rxDone_ = false;

bool TelemetryService::begin() {
  SPI.begin(AvionicsConfig::LoRaSckPin,
            AvionicsConfig::LoRaMisoPin,
            AvionicsConfig::LoRaMosiPin,
            AvionicsConfig::LoRaCsPin);

  const int16_t state = radio.begin(AvionicsConfig::LoRaFrequencyMHz,
                                    AvionicsConfig::LoRaBandwidthKHz,
                                    AvionicsConfig::LoRaSpreadingFactor,
                                    AvionicsConfig::LoRaCodingRate,
                                    AvionicsConfig::LoRaSyncWord,
                                    AvionicsConfig::LoRaTxPowerDbm,
                                    AvionicsConfig::LoRaPreambleLength);
  ready_ = state == RADIOLIB_ERR_NONE;
  if (ready_) {
    if (AvionicsConfig::LoRaUseDio2RfSwitch) {
      radio.setDio2AsRfSwitch(true);
    }
  } else {
    tryPrintStatus("SX1262 init failed:", state);
  }

  txInProgress_ = false;
  txDone_ = false;
  rxDone_ = false;
  rxListening_ = false;
  enabled_ = false;
  intervalMs_ = 0;
  nextTelemetryAtMs_ = 0;
  startReceiveIfIdle();
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
  serviceRadio();

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
  serviceRadio();

  if (!ready_ || txInProgress_) {
    return false;
  }

  if (rxListening_) {
    rxDone_ = false;
    radio.standby();
    rxListening_ = false;
  }

  radio.setPacketSentAction(TelemetryService::onTxDone);
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t *mutableBytes = const_cast<uint8_t *>(bytes);
  const int16_t state = radio.startTransmit(mutableBytes, sizeof(packet));
  if (state != RADIOLIB_ERR_NONE) {
    tryPrintStatus("SX1262 TX start failed:", state);
    return false;
  }

  txDone_ = false;
  txInProgress_ = true;
  if (AvionicsConfig::EnableTelemetrySerialDump) {
    printBytes(bytes, sizeof(packet));
  }
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
  packet.triboVoltage = static_cast<uint16_t>(measurement.ina226.busVoltageV * 1000.0f); // Convert V to mV
  packet.batteryVoltage = scaleBatteryMilliVolts(measurement.batteryMilliVolts);

  if (measurement.gps.locationValid) {
    packet.gpsLatOffset = scaleGpsOffset(
        measurement.gps.latitudeDeg - AvionicsConfig::BaseLatitudeDeg);
    packet.gpsLonOffset = scaleGpsOffset(
        measurement.gps.longitudeDeg - AvionicsConfig::BaseLongitudeDeg);
  }

  if (measurement.gps.altitudeValid) {
    packet.gpsAltMeters = clampInt16(measurement.gps.altitudeMeters);
  }

  packet.ky024Analog = measurement.ky024.analog;

  return packet;
}

void IRAM_ATTR TelemetryService::onTxDone() {
  txDone_ = true;
}

void IRAM_ATTR TelemetryService::onRxDone() {
  rxDone_ = true;
}

void TelemetryService::serviceRadio() {
  if (!ready_) {
    return;
  }

  if (txInProgress_) {
    if (!txDone_) {
      return;
    }

    txDone_ = false;
    const int16_t state = radio.finishTransmit();
    if (state != RADIOLIB_ERR_NONE) {
      tryPrintStatus("SX1262 TX finish failed:", state);
    }

    txInProgress_ = false;
    rxListening_ = false;
    startReceiveIfIdle();
    return;
  }

  if (rxDone_) {
    rxDone_ = false;
    rxListening_ = false;

    uint8_t buffer[AvionicsConfig::LoRaMaxRxPacketBytes] = {};
    const size_t packetLength = radio.getPacketLength();
    const size_t readLength = packetLength <= sizeof(buffer) ? packetLength : sizeof(buffer);
    const int16_t state = radio.readData(buffer, readLength);

    if (state == RADIOLIB_ERR_NONE) {
      GroundControl::handlePacket(buffer, readLength, radio.getRSSI(), radio.getSNR());
    } else {
      tryPrintStatus("SX1262 RX failed:", state);
    }
  }

  startReceiveIfIdle();
}

void TelemetryService::startReceiveIfIdle() {
  if (!ready_ || txInProgress_ || rxListening_) {
    return;
  }

  radio.setPacketReceivedAction(TelemetryService::onRxDone);
  const int16_t state = radio.startReceive();
  rxListening_ = state == RADIOLIB_ERR_NONE;
}

bool TelemetryService::timeoutFired(uint32_t nowMs) const {
  return enabled_ && static_cast<int32_t>(nowMs - nextTelemetryAtMs_) >= 0;
}

void TelemetryService::printBytes(const uint8_t *bytes, size_t length) {
  if (!AvionicsConfig::EnableSerial) {
    return;
  }

  char line[192] = {};
  size_t pos = 0;
  const int written = snprintf(line + pos, sizeof(line) - pos,
                               "TX %u bytes:",
                               static_cast<unsigned>(length));
  if (written <= 0) {
    return;
  }

  pos += static_cast<size_t>(written);
  for (size_t i = 0; bytes != nullptr && i < length; ++i) {
    if (pos + 3 >= sizeof(line)) {
      break;
    }
    line[pos++] = ' ';
    line[pos++] = kHexDigits[(bytes[i] >> 4) & 0x0F];
    line[pos++] = kHexDigits[bytes[i] & 0x0F];
  }

  if (pos + 1 >= sizeof(line)) {
    pos = sizeof(line) - 2;
  }
  line[pos++] = '\n';

  if (Serial.availableForWrite() < pos) {
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), pos);
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

uint8_t TelemetryService::scaleBatteryMilliVolts(uint16_t batteryMilliVolts) {
  const uint16_t scaled = (batteryMilliVolts + 10U) / 20U;
  if (scaled > 255U) {
    return 255;
  }

  return static_cast<uint8_t>(scaled);
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
