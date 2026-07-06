#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "esp_partition.h"
#include "FlightFsm.h"
#include "Measurements.h"

#pragma pack(push, 1)
struct LogPacket {
  uint32_t syncWord;
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
  uint8_t rawPressureExt; // Aligns gpsLatOffset on 2-byte boundary

  int16_t gpsLatOffset;
  int16_t gpsLonOffset;
  int16_t kfVerticalVelocity;
  uint16_t ky024Analog;

  uint8_t gpsQuality;
  int8_t temperature;
  uint16_t checksum; // Fletcher16 checksum
};
#pragma pack(pop)

static_assert(sizeof(LogPacket) == 40, "LogPacket must be exactly 40 bytes");

class FlightLogger {
 public:
  ~FlightLogger();

  bool begin();
  void erase();
  bool log(uint32_t nowMs, const FlightFsm &fsm, const MeasurementSnapshot &measurement);

 private:
  static void loggingTask(void* parameter);
  uint16_t calculateFletcher16(const uint8_t *data, size_t len);
  uint32_t findWriteOffset();

  const esp_partition_t* partition_ = nullptr;
  QueueHandle_t queue_ = nullptr;
  TaskHandle_t taskHandle_ = nullptr;
  
  volatile uint32_t writeOffset_ = 0;
  volatile bool ready_ = false;
  volatile bool eraseInProgress_ = false;
};
