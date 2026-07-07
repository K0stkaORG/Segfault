#include "FlightLogger.h"
#include "AvionicsConfig.h"
#include "Telemetry.h" // For scalePressure, etc. if needed, but we do them locally to stay decoupled

FlightLogger::~FlightLogger() {
  if (taskHandle_ != nullptr) {
    vTaskDelete(taskHandle_);
  }
  if (queue_ != nullptr) {
    vQueueDelete(queue_);
  }
}

bool FlightLogger::begin() {
  partition_ = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, static_cast<esp_partition_subtype_t>(0x99), "flightlog");
  
  if (partition_ == nullptr) {
    if (AvionicsConfig::EnableSerial) {
      Serial.println(F("Logger: flightlog partition not found!"));
    }
    return false;
  }

  writeOffset_ = findWriteOffset();

  if (AvionicsConfig::EnableSerial) {
    Serial.print(F("Logger: write offset initialized at "));
    Serial.println(writeOffset_);
  }

  queue_ = xQueueCreate(50, sizeof(LogPacket));
  if (queue_ == nullptr) {
    return false;
  }

  BaseType_t ret = xTaskCreate(loggingTask, "logging_task", 3072, this, 1, &taskHandle_);
  if (ret != pdPASS) {
    vQueueDelete(queue_);
    queue_ = nullptr;
    return false;
  }

  ready_ = true;
  return true;
}

void FlightLogger::erase() {
  if (partition_ == nullptr) return;

  eraseInProgress_ = true;
  ready_ = false;

  // Clear queue
  if (queue_ != nullptr) {
    xQueueReset(queue_);
  }

  if (AvionicsConfig::EnableSerial) {
    Serial.println(F("Logger: Erasing flightlog partition..."));
  }

  esp_err_t err = esp_partition_erase_range(partition_, 0, partition_->size);

  if (AvionicsConfig::EnableSerial) {
    if (err == ESP_OK) {
      Serial.println(F("Logger: Erased successfully."));
    } else {
      Serial.print(F("Logger: Erase failed: "));
      Serial.println(err);
    }
  }

  writeOffset_ = 0;
  eraseInProgress_ = false;
  ready_ = (err == ESP_OK);
}

bool FlightLogger::log(uint32_t nowMs, const FlightFsm &fsm, const MeasurementSnapshot &measurement) {
  if (!ready_ || eraseInProgress_) {
    return false;
  }

  if (writeOffset_ + sizeof(LogPacket) > partition_->size) {
    // Partition full
    return false;
  }

  LogPacket packet;
  packet.syncWord = AvionicsConfig::FlightLogSyncWord;
  packet.timestampMs = static_cast<uint16_t>(nowMs);
  
  // Re-use packet ID or track locally. Let's use 0 or simple tracking.
  packet.packetId = 0; 
  packet.stateFlags = fsm.stateFlags();

  // IMU
  packet.accelX = static_cast<int16_t>(measurement.imu.accelX_g * AvionicsConfig::Bmi270AccelGToLsbTelemetry);
  packet.accelY = static_cast<int16_t>(measurement.imu.accelY_g * AvionicsConfig::Bmi270AccelGToLsbTelemetry);
  packet.accelZ = static_cast<int16_t>(measurement.imu.accelZ_g * AvionicsConfig::Bmi270AccelGToLsbTelemetry);
  packet.gyroX = static_cast<int16_t>(measurement.imu.gyroX_dps * AvionicsConfig::Bmi270GyroDpsToLsbTelemetry);
  packet.gyroY = static_cast<int16_t>(measurement.imu.gyroY_dps * AvionicsConfig::Bmi270GyroDpsToLsbTelemetry);
  packet.gyroZ = static_cast<int16_t>(measurement.imu.gyroZ_dps * AvionicsConfig::Bmi270GyroDpsToLsbTelemetry);

  // Pressure and Altimeter
  // Scaling matches the updated decimeter-scaled outputs
  // (aglAltitude_m * 10.0f and verticalVelocity_mps * 10.0f clamped)
  auto clamp16 = [](double val) -> int16_t {
    if (val > 32767.0) return 32767;
    if (val < -32768.0) return -32768;
    return static_cast<int16_t>(val);
  };
  
  packet.kfAltitudeAgl = clamp16(measurement.aglAltitude_m * 10.0f);
  
  // 24-bit raw pressure (0.1 Pa resolution)
  uint32_t scaledPressure = static_cast<uint32_t>(measurement.pressurePa * 10.0f);
  packet.rawPressure = scaledPressure & 0xFFFF;
  packet.rawPressureExt = (scaledPressure >> 16) & 0xFF;

  packet.triboVoltage = static_cast<uint16_t>(measurement.ina226.busVoltageV * 1000.0f);
  
  // Scale battery: Millivolts offset by 2000, divided by 10 (matching scaleBatteryMilliVolts in Telemetry.cpp)
  int32_t batMv = static_cast<int32_t>(measurement.batteryMilliVolts) - 2000;
  if (batMv < 0) batMv = 0;
  batMv /= 10;
  if (batMv > 255) batMv = 255;
  packet.batteryVoltage = static_cast<uint8_t>(batMv);

  // GPS
  if (measurement.gps.locationValid) {
    auto scaleGps = [clamp16](double offset) -> int16_t {
      return clamp16(offset * 100000.0);
    };
    packet.gpsLatOffset = scaleGps(measurement.gps.latitudeDeg - AvionicsConfig::BaseLatitudeDeg);
    packet.gpsLonOffset = scaleGps(measurement.gps.longitudeDeg - AvionicsConfig::BaseLongitudeDeg);
  } else {
    packet.gpsLatOffset = 0;
    packet.gpsLonOffset = 0;
  }

  packet.kfVerticalVelocity = clamp16(measurement.verticalVelocity_mps * 10.0f);
  packet.ky024Analog = measurement.ky024.analog;

  // GPS Quality: sats (4 bits) | fix type (2 bits) | system flags (2 bits)
  uint8_t sats = measurement.gps.satellites > 15 ? 15 : measurement.gps.satellites;
  uint8_t fix = measurement.gps.fixValid ? 2 : (measurement.gps.locationValid ? 1 : 0);
  uint8_t sysFlags = (measurement.bmp280Ok ? 1 : 0) | ((measurement.bmi270Ok ? 1 : 0) << 1);
  packet.gpsQuality = (sats & 0x0F) | ((fix & 0x03) << 4) | ((sysFlags & 0x03) << 6);

  // BMP280 Temperature (8-bit signed int, 0.5 °C resolution, clamped to -64.0 to +63.5 °C)
  float clampedTemp = measurement.temperatureC;
  if (clampedTemp < -64.0f) clampedTemp = -64.0f;
  if (clampedTemp > 63.5f) clampedTemp = 63.5f;
  packet.temperature = static_cast<int8_t>(clampedTemp * 2.0f);

  packet.checksum = 0;

  // Compute Fletcher16 over the first 38 bytes of structural data
  packet.checksum = calculateFletcher16(reinterpret_cast<const uint8_t*>(&packet), 38);

  // Push to queue without blocking (timeout = 0)
  BaseType_t ret = xQueueSend(queue_, &packet, 0);
  return (ret == pdPASS);
}

void FlightLogger::loggingTask(void* parameter) {
  FlightLogger* logger = static_cast<FlightLogger*>(parameter);
  LogPacket packet;

  while (true) {
    if (xQueueReceive(logger->queue_, &packet, portMAX_DELAY) == pdPASS) {
      if (logger->partition_ != nullptr && logger->ready_) {
        if (logger->writeOffset_ + sizeof(LogPacket) <= logger->partition_->size) {
          if (!AvionicsConfig::DisableFlightLogWrites) {
            esp_partition_write(logger->partition_, logger->writeOffset_, &packet, sizeof(LogPacket));
          }
          logger->writeOffset_ += sizeof(LogPacket);
        } else {
          logger->ready_ = false; // Stop write triggers
          if (AvionicsConfig::EnableSerial) {
            Serial.println(F("Logger: Partition is completely full. Logging stopped."));
          }
        }
      }
    }
  }
}

uint16_t FlightLogger::calculateFletcher16(const uint8_t *data, size_t len) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  for (size_t i = 0; i < len; ++i) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

uint32_t FlightLogger::findWriteOffset() {
  if (partition_ == nullptr) return 0;

  uint32_t low = 0;
  uint32_t high = partition_->size / 4096; // 256 sectors
  
  while (low < high) {
    uint32_t mid = low + (high - low) / 2;
    uint32_t firstDword = 0xFFFFFFFF;
    esp_partition_read(partition_, mid * 4096, &firstDword, 4);

    if (firstDword == 0xFFFFFFFF) {
      high = mid;
    } else {
      low = mid + 1;
    }
  }

  uint32_t sectorStart = low * 4096;
  if (sectorStart >= partition_->size) {
    return partition_->size;
  }

  uint32_t offset = sectorStart;
  uint32_t limit = sectorStart + 4096;
  if (limit > partition_->size) limit = partition_->size;

  while (offset + sizeof(LogPacket) <= limit) {
    uint32_t syncWord = 0;
    esp_partition_read(partition_, offset, &syncWord, 4);
    if (syncWord == 0xFFFFFFFF) {
      break;
    }
    offset += sizeof(LogPacket);
  }

  return offset;
}
