#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <INA226_WE.h>
#include "PersistentStore.h"

#ifndef XPOWERS_CHIP_AXP2101
#define XPOWERS_CHIP_AXP2101
#endif
#include <XPowersLib.h>

extern "C" {
#include <bmi270_api/bmi270.h>
}

struct KalmanState {
  float altitude_m = 0.0f;
  float velocity_mps = 0.0f;
  float p00 = 1.0f;
  float p01 = 0.0f;
  float p10 = 0.0f;
  float p11 = 1.0f;
};

struct Bmi270RawSample {
  int16_t accelX = 0;
  int16_t accelY = 0;
  int16_t accelZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
};

struct ImuCalibratedSample {
  float accelX_g = 0.0f;
  float accelY_g = 0.0f;
  float accelZ_g = 0.0f;
  float gyroX_dps = 0.0f;
  float gyroY_dps = 0.0f;
  float gyroZ_dps = 0.0f;
};

struct Ky024Sample {
  uint16_t analog = 0;
  bool digital = false;
};

struct Ina226Sample {
  float shuntVoltageMv = 0.0f;
  float busVoltageV = 0.0f;
  float currentMa = 0.0f;
  float powerMw = 0.0f;
};

struct GpsSample {
  bool locationValid = false;
  bool altitudeValid = false;
  bool timeValid = false;
  bool fixValid = false;
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeMeters = 0.0;
  uint32_t timeOfDayMs = 0;
  uint32_t locationAgeMs = 0;
  uint8_t satellites = 0;
  float hdop = 0.0f;
};

struct MeasurementSnapshot {
  uint32_t updatedAtMs = 0;
  bool bmp280Ok = false;
  bool bmi270Ok = false;
  bool gpsOk = false;
  bool ina226Ok = false;
  bool pmuOk = false;
  bool bmp280ReadOk = false;
  bool bmi270ReadOk = false;
  bool gpsReadOk = false;
  bool ina226ReadOk = false;
  bool pmuReadOk = false;
  float temperatureC = 0.0f;
  float pressurePa = 0.0f;
  float aglAltitude_m = 0.0f;
  float verticalVelocity_mps = 0.0f;
  Bmi270RawSample imuRaw;
  ImuCalibratedSample imu;
  GpsSample gps;
  Ky024Sample ky024;
  Ina226Sample ina226;
  uint16_t batteryMilliVolts = 0;
};

class MeasurementService {
 public:
  MeasurementService();

  bool begin();
  void tick(uint32_t nowMs);
  const MeasurementSnapshot &latest() const;
  
  void setBaseliningEnabled(bool enabled);
  void setBaseline(const SensorBaseline& baseline);
  SensorBaseline getBaseline() const;
  void resetBaseline();

 private:
  struct Bmi270I2cContext {
    uint8_t address = 0;
    TwoWire *wire = nullptr;
  };

  bool beginBmp280();
  bool beginBmi270();
  bool beginGps();
  bool beginIna226();
  bool beginPmu();
  bool readBmp280();
  bool readBmi270();
  bool readGps(uint32_t nowMs);
  bool readIna226();
  bool readPmu();

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
  INA226_WE ina226_;
  XPowersPMU pmu_;
  bmi2_dev bmi270Dev_{};
  Bmi270I2cContext bmi270Context_{};
  MeasurementSnapshot snapshot_{};
  uint32_t nextSampleAtMs_ = 0;
  
  bool isBaseliningEnabled_ = true;
  SensorBaseline baseline_{};
  KalmanState kfState_{};
  uint32_t lastKfUpdateMs_ = 0;
  bool kfInitialized_ = false;
};
