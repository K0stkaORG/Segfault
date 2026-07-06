#pragma once

#include <Arduino.h>
#include "FlightFsm.h"
#include "Measurements.h"
#include "PersistentStore.h"

#pragma pack(push, 1)
struct RocketTelemetry {
  uint16_t syncWord;
  uint16_t timestampMs;
  uint8_t packetId;
  uint8_t stateFlags;

  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;

  int16_t kfAltitudeAgl;
  uint16_t rawPressure;
  uint16_t triboVoltage;
  uint8_t batteryVoltage;

  int16_t gpsLatOffset;
  int16_t gpsLonOffset;
  int16_t kfVerticalVelocity;
  uint16_t ky024Analog;
};
#pragma pack(pop)

static_assert(sizeof(RocketTelemetry) == 33, "RocketTelemetry must be 33 bytes");

class TelemetryService {
 public:
  bool begin();
  uint8_t packetCounter() const;
  void setInterval(uint32_t intervalMs);
  void tick(uint32_t nowMs,
            const FlightFsm &fsm,
            const MeasurementSnapshot &measurement,
            PersistentStore &persistentStore);
  bool send(const RocketTelemetry &packet);
  static bool isTxInProgress();

  static RocketTelemetry buildPacket(uint32_t nowMs,
                                     const FlightFsm &fsm,
                                     const MeasurementSnapshot &measurement);

 private:
  static void serviceRadio();
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

  static volatile bool txInProgress_;
};
