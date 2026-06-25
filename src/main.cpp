#include <Arduino.h>

#include "AvionicsConfig.h"
#include "FlightFsm.h"
#include "Measurements.h"
#include "PersistentStore.h"
#include "StateLogic.h"
#include "Telemetry.h"
#include "ParachuteServo.h"

namespace {
PersistentStore persistentStore;
MeasurementService measurements;
TelemetryService telemetry;
StateLogic stateLogic;
FlightFsm flightFsm;
ParachuteServo parachuteServo;
}  // namespace

void setup() {
  if (AvionicsConfig::EnableSerial) {
    Serial.begin(AvionicsConfig::SerialBaud);
  }

  const bool persistentOk = persistentStore.begin();
  flightFsm.attachPersistentStore(persistentStore);
  flightFsm.setState(persistentStore.loadFlightState());
  telemetry.setPacketCounter(persistentStore.loadPacketCounter());

  const bool measurementsOk = measurements.begin();
  const bool telemetryOk = telemetry.begin();
  parachuteServo.begin(AvionicsConfig::ParachuteServoPin);

  persistentStore.saveInitStatus(measurements.latest().bmp280Ok,
                                 measurements.latest().bmi270Ok,
                                 measurements.latest().gpsOk,
                                 telemetryOk);

  if (AvionicsConfig::EnableSerial) {
    Serial.println(F("<Testing In Production>"));
    Serial.print(F("NVS: "));
    Serial.println(persistentOk ? F("ok") : F("failed"));
    Serial.print(F("Boot count: "));
    Serial.println(persistentStore.bootCount());
    Serial.print(F("State: "));
    Serial.println(flightFsm.stateName());
    Serial.print(F("BMP280: "));
    Serial.println(measurements.latest().bmp280Ok ? F("ok") : F("missing"));
    Serial.print(F("BMI270: "));
    Serial.println(measurements.latest().bmi270Ok ? F("ok") : F("missing"));
    Serial.print(F("NEO6M: "));
    Serial.println(measurements.latest().gpsOk ? F("ok") : F("missing"));
    Serial.print(F("AXP2101: "));
    Serial.println(measurements.latest().pmuOk ? F("ok") : F("missing"));
    Serial.print(F("Measurements: "));
    Serial.println(measurementsOk ? F("ok") : F("degraded"));
    Serial.print(F("Servo: "));
    Serial.println(parachuteServo.attached() ? F("attached") : F("failed"));
    Serial.print(F("LoRa: "));
    Serial.println(telemetryOk ? F("ok") : F("failed"));
  }
}

void loop() {
  const uint32_t nowMs = millis();
  measurements.tick(nowMs);
  stateLogic.tick(nowMs, flightFsm, measurements.latest(), telemetry);
  telemetry.tick(nowMs, flightFsm, measurements.latest(), persistentStore);
}
