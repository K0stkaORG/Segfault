#pragma once

#include <Arduino.h>

namespace AvionicsConfig {

constexpr uint32_t SerialBaud = 115200;

constexpr uint32_t TelemetryIntervalMs = 200;
constexpr uint32_t BeforeLaunchHeartbeatIntervalMs = 10000;
constexpr uint32_t MeasurementIntervalMs = 20;

constexpr long LoRaFrequencyHz = 433375000L;
constexpr uint8_t LoRaSyncWord = 0x67;

constexpr uint8_t I2cSdaPin = 21;
constexpr uint8_t I2cSclPin = 22;
constexpr uint32_t I2cClockHz = 400000;

constexpr uint8_t Bmp280I2cAddress = 0x77;
constexpr uint8_t Bmi270I2cAddress = 0x68;

constexpr uint8_t GpsUartPort = 1;
constexpr uint32_t GpsBaud = 9600;
constexpr int GpsRxPin = 34;
constexpr int GpsTxPin = 12;
constexpr uint16_t MaxGpsBytesPerTick = 96;

constexpr uint8_t LoRaSckPin = 5;
constexpr uint8_t LoRaMisoPin = 19;
constexpr uint8_t LoRaMosiPin = 27;
constexpr uint8_t LoRaCsPin = 18;
constexpr uint8_t LoRaResetPin = 14;
constexpr uint8_t LoRaDio0Pin = 26;

constexpr int TriboVoltageAdcPin = 36;
constexpr int BatteryVoltageAdcPin = 35;
constexpr int BreakawayWirePin = 33;
constexpr int BuzzerPin = 25;

constexpr double BaseLatitudeDeg = 49.7983333333;
constexpr double BaseLongitudeDeg = 16.6866666667;
constexpr const char *BaseGridSquare = "JN89IT";

constexpr const char *NvsNamespace = "rocket";

}  // namespace AvionicsConfig
