#pragma once

#include <Arduino.h>
#include "FlightFsm.h"
#include "Measurements.h"
#include "PersistentStore.h"

#pragma pack(push, 1)
struct RocketTelemetry {
  uint16_t timestampMs;
  uint8_t packetId;
  uint8_t stateFlags;

  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;

  uint16_t pressureScaled;
  uint16_t triboVoltage;
  uint8_t batteryVoltage;

  int16_t gpsLatOffset;
  int16_t gpsLonOffset;
  int16_t gpsAltMeters;
  uint16_t ky024Analog;
};
#pragma pack(pop)

static_assert(sizeof(RocketTelemetry) == 29, "RocketTelemetry must be 29 bytes");

class TelemetryService {
 public:
  bool begin();
  void setPacketCounter(uint8_t packetCounter);
  uint8_t packetCounter() const;
  void setInterval(uint32_t intervalMs);
  void disable();
  void tick(uint32_t nowMs,
            const FlightFsm &fsm,
            const MeasurementSnapshot &measurement,
            PersistentStore &persistentStore);
  bool send(const RocketTelemetry &packet);
  bool isReady() const;

  static RocketTelemetry buildPacket(uint32_t nowMs,
                                     const FlightFsm &fsm,
                                     const MeasurementSnapshot &measurement);

 private:
  static void IRAM_ATTR onTxDone();
  static void IRAM_ATTR onRxDone();
  void serviceRadio(uint32_t nowMs);
  void startReceiveIfIdle(uint32_t nowMs);
  bool canInterruptRxForTx(uint32_t nowMs, uint32_t telemetryOverdueMs) const;
  void persistPacketCounterIfDue(uint32_t nowMs, PersistentStore &persistentStore);
  bool timeoutFired(uint32_t nowMs) const;
  static void printBytes(const uint8_t *bytes, size_t length);
  static uint16_t scalePressure(float pressurePa);
  static uint8_t scaleBatteryMilliVolts(uint16_t batteryMilliVolts);
  static int16_t scaleGpsOffset(double offsetDeg);
  static int16_t clampInt16(double value);

  bool ready_ = false;
  uint8_t packetCounter_ = 0;
  bool enabled_ = false;
  uint32_t intervalMs_ = 0;
  uint32_t nextTelemetryAtMs_ = 0;
  uint32_t rxStartedAtMs_ = 0;
  uint32_t txStartedAtMs_ = 0;
  uint32_t telemetryDueSinceMs_ = 0;
  uint32_t nextPacketCounterPersistAtMs_ = 0;
  uint8_t lastPersistedPacketCounter_ = 0;

  static volatile bool txInProgress_;
  static volatile bool txDone_;
  static volatile bool rxDone_;
  bool rxListening_ = false;
};
