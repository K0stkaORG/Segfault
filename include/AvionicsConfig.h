#pragma once

#include <Arduino.h>

namespace AvionicsConfig {

constexpr uint32_t SerialBaud = 115200;

constexpr uint32_t TelemetryIntervalMs = 400;
constexpr uint32_t BeforeLaunchHeartbeatIntervalMs = 10000;
constexpr uint32_t MeasurementIntervalMs = 20;

constexpr bool EnableSerial = false;              // DEBUG ONLY!
constexpr bool EnableTelemetrySerialDump = false; // DEBUG ONLY!
constexpr bool DisableFlightLogWrites = false;    // DEBUG ONLY!

constexpr long LoRaFrequencyHz = 439700000L;
constexpr float LoRaFrequencyMHz = 439.7f;
constexpr float LoRaBandwidthKHz = 250.0f;
constexpr uint8_t LoRaSpreadingFactor = 8;
constexpr uint8_t LoRaCodingRate = 8;
constexpr uint8_t LoRaSyncWord = 0x67;
constexpr int8_t LoRaTxPowerDbm = 10;
constexpr uint16_t LoRaPreambleLength = 8;
constexpr bool LoRaUseDio2RfSwitch = true;
constexpr uint16_t TelemetrySyncWord = 0x5AA5;

constexpr uint8_t I2cSdaPin = 21;
constexpr uint8_t I2cSclPin = 22;
constexpr uint32_t I2cClockHz = 400000;

constexpr uint8_t Bmp280I2cAddress = 0x77;
constexpr float TemperatureOffsetC = 0.0f;
constexpr uint8_t Bmi270I2cAddress = 0x68;
constexpr float Bmi270AccelLsbToG = 1.0f / 4096.0f;
constexpr float Bmi270GyroLsbToDps = 1.0f / 16.4f;
constexpr float Bmi270AccelGToLsbTelemetry = 16384.0f;
constexpr float Bmi270GyroDpsToLsbTelemetry = 16.4f;
constexpr uint8_t Ina226I2cAddress = 0x40;

constexpr uint8_t GpsUartPort = 1;
constexpr uint32_t GpsBaud = 9600;
constexpr int GpsRxPin = 34;
constexpr int GpsTxPin = 12;
constexpr uint16_t MaxGpsBytesPerTick = 96;

constexpr uint8_t LoRaSckPin = 5;
constexpr uint8_t LoRaMisoPin = 19;
constexpr uint8_t LoRaMosiPin = 27;
constexpr uint8_t LoRaCsPin = 18;
constexpr uint8_t LoRaResetPin = 23;
constexpr uint8_t LoRaDio1Pin = 33;
constexpr uint8_t LoRaBusyPin = 32;

constexpr int Ky024AnalogPin = 36;
constexpr int Ky024DigitalPin = 14;

constexpr int ParachuteServoPin = 13;
constexpr int ParachuteServoMinPulseUs = 500;
constexpr int ParachuteServoMaxPulseUs = 2400;
constexpr int ParachuteServoMinAngle = 0;
constexpr int ParachuteServoMaxAngle = 180;
constexpr int ParachuteServoStowedAngle = 0;
constexpr int ParachuteServoDeployedAngle = 90;
constexpr int ParachuteServoFrequencyHz = 50;

constexpr float LaunchAccelThresholdMps2 = 25.0f;
constexpr uint32_t LaunchAccelDurationMs = 100;
constexpr float ApogeeAltitudeDropM = 5.0f;
constexpr float ApogeeVelocityThresholdUpMps = 5.0f;
constexpr uint32_t ApogeeFailsafeTimerMs = 11400;

constexpr double BaseLatitudeDeg = 49.7983333333;
constexpr double BaseLongitudeDeg = 16.6866666667;
//constexpr double BaseLatitudeDeg = 49.16100; // DEBUG ONLY!
//constexpr double BaseLongitudeDeg = 16.56133; // DEBUG ONLY!

constexpr uint32_t FlightLogSyncWord = 0x5AA55AA5;
constexpr uint32_t FlightLogIntervalMs = 40;
constexpr float FlightLogArmedAccelThresholdG = 1.0f;
constexpr float FlightLogChuteVelocityThresholdMps = -2.0f;

constexpr const char *NvsNamespace = "rocket";

constexpr const char *RecoveryWifiSsid = "TIPRocket";
constexpr const char *RecoveryWifiPassword = "hlinena67";
constexpr uint8_t RecoveryLocalIp[4] = {172, 27, 67, 1};
constexpr uint8_t RecoveryGatewayIp[4] = {172, 27, 67, 1};
constexpr uint8_t RecoverySubnetMask[4] = {255, 255, 255, 240};

}  // namespace AvionicsConfig
