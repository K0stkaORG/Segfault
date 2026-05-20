#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

extern "C" {
#include <bmi270_api/bmi270.h>
}

struct Bmi270RawSample {
  int16_t accelX = 0;
  int16_t accelY = 0;
  int16_t accelZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
};

struct GpsSample {
  bool locationValid = false;
  bool altitudeValid = false;
  bool fixValid = false;
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeMeters = 0.0;
  uint32_t locationAgeMs = 0;
  uint8_t satellites = 0;
  float hdop = 0.0f;
};

struct MeasurementSnapshot {
  uint32_t updatedAtMs = 0;
  bool bmp280Ok = false;
  bool bmi270Ok = false;
  bool gpsOk = false;
  bool bmp280ReadOk = false;
  bool bmi270ReadOk = false;
  bool gpsReadOk = false;
  float temperatureC = 0.0f;
  float pressurePa = 0.0f;
  Bmi270RawSample imu;
  GpsSample gps;
  uint16_t triboAdc = 0;
  uint16_t batteryAdc = 0;
};

class MeasurementService {
 public:
  MeasurementService();

  bool begin();
  void tick(uint32_t nowMs);
  const MeasurementSnapshot &latest() const;

 private:
  struct Bmi270I2cContext {
    uint8_t address = 0;
    TwoWire *wire = nullptr;
  };

  bool beginBmp280();
  bool beginBmi270();
  bool beginGps();
  bool readBmp280();
  bool readBmi270();
  bool readGps(uint32_t nowMs);

  static BMI2_INTF_RETURN_TYPE bmi270Read(uint8_t regAddr,
                                          uint8_t *regData,
                                          uint32_t length,
                                          void *context);
  static BMI2_INTF_RETURN_TYPE bmi270Write(uint8_t regAddr,
                                           const uint8_t *regData,
                                           uint32_t length,
                                           void *context);
  static void bmi270DelayUs(uint32_t periodUs, void *context);

  Adafruit_BMP280 bmp280_;
  HardwareSerial gpsSerial_;
  TinyGPSPlus gpsParser_;
  bmi2_dev bmi270Dev_{};
  Bmi270I2cContext bmi270Context_{};
  MeasurementSnapshot snapshot_{};
  uint32_t nextSampleAtMs_ = 0;
};
