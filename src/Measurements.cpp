#include "Measurements.h"

#include <string.h>
#include "AvionicsConfig.h"

MeasurementService::MeasurementService()
    : gpsSerial_(AvionicsConfig::GpsUartPort),
      ina226_(AvionicsConfig::Ina226I2cAddress) {
}

bool MeasurementService::begin() {
  Wire.begin(AvionicsConfig::I2cSdaPin, AvionicsConfig::I2cSclPin);
  Wire.setClock(AvionicsConfig::I2cClockHz);

  snapshot_.bmp280Ok = beginBmp280();
  snapshot_.bmi270Ok = beginBmi270();
  snapshot_.gpsOk = beginGps();
  snapshot_.ina226Ok = beginIna226();
  snapshot_.pmuOk = beginPmu();
  nextSampleAtMs_ = 0;

  pinMode(AvionicsConfig::Ky024AnalogPin, INPUT);
  pinMode(AvionicsConfig::Ky024DigitalPin, INPUT);

  return snapshot_.bmp280Ok && snapshot_.bmi270Ok && snapshot_.gpsOk &&
         snapshot_.ina226Ok && snapshot_.pmuOk;
}

void MeasurementService::tick(uint32_t nowMs) {
  if (static_cast<int32_t>(nowMs - nextSampleAtMs_) < 0) {
    return;
  }

  snapshot_.updatedAtMs = nowMs;

  snapshot_.ky024.analog = analogRead(AvionicsConfig::Ky024AnalogPin);
  snapshot_.ky024.digital = (digitalRead(AvionicsConfig::Ky024DigitalPin) == HIGH);

  snapshot_.bmp280ReadOk = snapshot_.bmp280Ok && readBmp280();
  snapshot_.bmi270ReadOk = snapshot_.bmi270Ok && readBmi270();
  snapshot_.gpsReadOk = snapshot_.gpsOk && readGps(nowMs);
  snapshot_.ina226ReadOk = snapshot_.ina226Ok && readIna226();
  snapshot_.pmuReadOk = snapshot_.pmuOk && readPmu();

  nextSampleAtMs_ = nowMs + AvionicsConfig::MeasurementIntervalMs;
}

const MeasurementSnapshot &MeasurementService::latest() const {
  return snapshot_;
}

bool MeasurementService::beginBmp280() {
  if (!bmp280_.begin(AvionicsConfig::Bmp280I2cAddress)) {
    return false;
  }

  bmp280_.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_1);
  return true;
}

bool MeasurementService::beginBmi270() {
  memset(&bmi270Dev_, 0, sizeof(bmi270Dev_));

  bmi270Context_.address = AvionicsConfig::Bmi270I2cAddress;
  bmi270Context_.wire = &Wire;

  bmi270Dev_.intf = BMI2_I2C_INTF;
  bmi270Dev_.read = MeasurementService::bmi270Read;
  bmi270Dev_.write = MeasurementService::bmi270Write;
  bmi270Dev_.delay_us = MeasurementService::bmi270DelayUs;
  bmi270Dev_.intf_ptr = &bmi270Context_;
  bmi270Dev_.read_write_len = 32;

  int8_t err = bmi270_init(&bmi270Dev_);
  if (err != BMI2_OK) {
    return false;
  }

  uint8_t sensors[] = {BMI2_ACCEL, BMI2_GYRO};
  err = bmi270_sensor_enable(sensors, 2, &bmi270Dev_);
  return err == BMI2_OK;
}

bool MeasurementService::beginGps() {
  gpsSerial_.begin(AvionicsConfig::GpsBaud,
                   SERIAL_8N1,
                   AvionicsConfig::GpsRxPin,
                   AvionicsConfig::GpsTxPin);
  return true;
}

bool MeasurementService::beginIna226() {
  if (!ina226_.init()) {
    return false;
  }
  return true;
}

bool MeasurementService::beginPmu() {
  if (!pmu_.begin(Wire,
                  AXP2101_SLAVE_ADDRESS,
                  AvionicsConfig::I2cSdaPin,
                  AvionicsConfig::I2cSclPin)) {
    return false;
  }

  const bool detectionOk = pmu_.enableBattDetection();
  const bool voltageMeasureOk = pmu_.enableBattVoltageMeasure();
  return detectionOk && voltageMeasureOk;
}

bool MeasurementService::readBmp280() {
  snapshot_.temperatureC = bmp280_.readTemperature();
  snapshot_.pressurePa = bmp280_.readPressure();
  return snapshot_.pressurePa > 0.0f;
}

bool MeasurementService::readBmi270() {
  bmi2_sens_data rawData{};
  const int8_t err = bmi2_get_sensor_data(&rawData, &bmi270Dev_);
  if (err != BMI2_OK) {
    return false;
  }

  snapshot_.imuRaw.accelX = rawData.acc.x;
  snapshot_.imuRaw.accelY = rawData.acc.y;
  snapshot_.imuRaw.accelZ = rawData.acc.z;
  snapshot_.imuRaw.gyroX = rawData.gyr.x;
  snapshot_.imuRaw.gyroY = rawData.gyr.y;
  snapshot_.imuRaw.gyroZ = rawData.gyr.z;

  float rawAx = rawData.acc.x;
  float rawAy = rawData.acc.y;
  float rawAz = rawData.acc.z;

  float corrAx = (-0.10503395f * rawAx) + (0.01122737f * rawAy) - (0.99440526f * rawAz);
  float corrAy = ( 0.01122737f * rawAx) + (0.99988593f * rawAy) + (0.01010336f * rawAz);
  float corrAz = ( 0.99440526f * rawAx) - (0.01010336f * rawAy) - (0.10514803f * rawAz);

  snapshot_.imu.accelX_g = corrAx * AvionicsConfig::Bmi270AccelLsbToG;
  snapshot_.imu.accelY_g = corrAy * AvionicsConfig::Bmi270AccelLsbToG;
  snapshot_.imu.accelZ_g = corrAz * AvionicsConfig::Bmi270AccelLsbToG;

  float rawGx = rawData.gyr.x;
  float rawGy = rawData.gyr.y;
  float rawGz = rawData.gyr.z;

  float corrGx = (-0.10503395f * rawGx) + (0.01122737f * rawGy) - (0.99440526f * rawGz);
  float corrGy = ( 0.01122737f * rawGx) + (0.99988593f * rawGy) + (0.01010336f * rawGz);
  float corrGz = ( 0.99440526f * rawGx) - (0.01010336f * rawGy) - (0.10514803f * rawGz);

  snapshot_.imu.gyroX_dps = corrGx * AvionicsConfig::Bmi270GyroLsbToDps;
  snapshot_.imu.gyroY_dps = corrGy * AvionicsConfig::Bmi270GyroLsbToDps;
  snapshot_.imu.gyroZ_dps = corrGz * AvionicsConfig::Bmi270GyroLsbToDps;
  return true;
}

bool MeasurementService::readGps(uint32_t nowMs) {
  bool consumedData = false;
  uint16_t bytesProcessed = 0;

  while (gpsSerial_.available() > 0 &&
         bytesProcessed < AvionicsConfig::MaxGpsBytesPerTick) {
    gpsParser_.encode(static_cast<char>(gpsSerial_.read()));
    consumedData = true;
    ++bytesProcessed;
  }

  snapshot_.gps.locationValid = gpsParser_.location.isValid();
  snapshot_.gps.altitudeValid = gpsParser_.altitude.isValid();
  snapshot_.gps.satellites = gpsParser_.satellites.isValid()
                                 ? gpsParser_.satellites.value()
                                 : 0;
  snapshot_.gps.hdop = gpsParser_.hdop.isValid()
                           ? gpsParser_.hdop.hdop()
                           : 0.0f;
  snapshot_.gps.locationAgeMs = gpsParser_.location.isValid()
                                    ? gpsParser_.location.age()
                                    : 0;
  snapshot_.gps.fixValid = snapshot_.gps.locationValid &&
                           snapshot_.gps.altitudeValid &&
                           snapshot_.gps.satellites > 0;

  if (gpsParser_.location.isUpdated()) {
    snapshot_.gps.latitudeDeg = gpsParser_.location.lat();
    snapshot_.gps.longitudeDeg = gpsParser_.location.lng();
  }

  if (gpsParser_.altitude.isUpdated()) {
    snapshot_.gps.altitudeMeters = gpsParser_.altitude.meters();
  }

  (void)nowMs;
  return consumedData;
}

bool MeasurementService::readIna226() {
  ina226_.readAndClearFlags(); // necessary to clear alerts and update some internals depending on lib, but mainly we just read values
  snapshot_.ina226.shuntVoltageMv = ina226_.getShuntVoltage_mV();
  snapshot_.ina226.busVoltageV = ina226_.getBusVoltage_V();
  snapshot_.ina226.currentMa = ina226_.getCurrent_mA();
  snapshot_.ina226.powerMw = ina226_.getBusPower();
  return true;
}

bool MeasurementService::readPmu() {
  if (!pmu_.isBatteryConnect()) {
    snapshot_.batteryMilliVolts = 0;
    return false;
  }

  snapshot_.batteryMilliVolts = pmu_.getBattVoltage();
  return snapshot_.batteryMilliVolts > 0;
}

BMI2_INTF_RETURN_TYPE MeasurementService::bmi270Read(uint8_t regAddr,
                                                     uint8_t *regData,
                                                     uint32_t length,
                                                     void *context) {
  Bmi270I2cContext *i2c = static_cast<Bmi270I2cContext *>(context);
  if (i2c == nullptr || i2c->wire == nullptr || regData == nullptr) {
    return BMI2_E_COM_FAIL;
  }

  i2c->wire->beginTransmission(i2c->address);
  i2c->wire->write(regAddr);
  if (i2c->wire->endTransmission(false) != 0) {
    return BMI2_E_COM_FAIL;
  }

  const uint8_t requested = static_cast<uint8_t>(length);
  const uint8_t received = i2c->wire->requestFrom(i2c->address, requested);
  if (received != requested) {
    return BMI2_E_COM_FAIL;
  }

  for (uint32_t i = 0; i < length; ++i) {
    regData[i] = i2c->wire->read();
  }

  return BMI2_INTF_RET_SUCCESS;
}

BMI2_INTF_RETURN_TYPE MeasurementService::bmi270Write(uint8_t regAddr,
                                                      const uint8_t *regData,
                                                      uint32_t length,
                                                      void *context) {
  Bmi270I2cContext *i2c = static_cast<Bmi270I2cContext *>(context);
  if (i2c == nullptr || i2c->wire == nullptr || regData == nullptr) {
    return BMI2_E_COM_FAIL;
  }

  i2c->wire->beginTransmission(i2c->address);
  i2c->wire->write(regAddr);
  for (uint32_t i = 0; i < length; ++i) {
    i2c->wire->write(regData[i]);
  }

  return i2c->wire->endTransmission() == 0 ? BMI2_INTF_RET_SUCCESS
                                           : BMI2_E_COM_FAIL;
}

void MeasurementService::bmi270DelayUs(uint32_t periodUs, void *context) {
  (void)context;
  delayMicroseconds(periodUs);
}
