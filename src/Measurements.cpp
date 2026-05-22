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
  nextSampleAtMs_ = 0;

  pinMode(AvionicsConfig::BatteryVoltageAdcPin, INPUT);
  pinMode(AvionicsConfig::Ky024AnalogPin, INPUT);
  pinMode(AvionicsConfig::Ky024DigitalPin, INPUT);

  return snapshot_.bmp280Ok && snapshot_.bmi270Ok && snapshot_.gpsOk && snapshot_.ina226Ok;
}

void MeasurementService::tick(uint32_t nowMs) {
  if (static_cast<int32_t>(nowMs - nextSampleAtMs_) < 0) {
    return;
  }

  snapshot_.updatedAtMs = nowMs;
  snapshot_.batteryAdc = analogRead(AvionicsConfig::BatteryVoltageAdcPin);

  snapshot_.ky024.analog = analogRead(AvionicsConfig::Ky024AnalogPin);
  snapshot_.ky024.digital = (digitalRead(AvionicsConfig::Ky024DigitalPin) == HIGH);

  snapshot_.bmp280ReadOk = snapshot_.bmp280Ok && readBmp280();
  snapshot_.bmi270ReadOk = snapshot_.bmi270Ok && readBmi270();
  snapshot_.gpsReadOk = snapshot_.gpsOk && readGps(nowMs);
  snapshot_.ina226ReadOk = snapshot_.ina226Ok && readIna226();

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

  snapshot_.imu.accelX = rawData.acc.x;
  snapshot_.imu.accelY = rawData.acc.y;
  snapshot_.imu.accelZ = rawData.acc.z;
  snapshot_.imu.gyroX = rawData.gyr.x;
  snapshot_.imu.gyroY = rawData.gyr.y;
  snapshot_.imu.gyroZ = rawData.gyr.z;
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
