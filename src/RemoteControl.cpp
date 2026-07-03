#include "RemoteControl.h"
#include "LoRaRadio.h"
#include "Telemetry.h"
#include "AvionicsConfig.h"
#include <Arduino.h>

void RemoteControlService::tick(ParachuteServo &servo, FlightFsm &fsm, MeasurementService &measurements) {
  if (TelemetryService::isTxInProgress()) {
    inRxMode_ = false;
    return;
  }

  if (!inRxMode_) {
    // Put radio into receive mode. startReceive is non-blocking.
    int16_t state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      inRxMode_ = true;
    }
  }

  if (inRxMode_ && loRaInterruptFired) {
    loRaInterruptFired = false;
    
    // An RX interrupt fired.
    size_t len = radio.getPacketLength();
    if (len == 4) {
      uint8_t buffer[4];
      int16_t state = radio.readData(buffer, 4);
      if (state == RADIOLIB_ERR_NONE) {
        if (buffer[0] == 0x47 && buffer[1] == 0x43) {
          if (buffer[2] == 0x55 && buffer[3] == 0x00) {
            servo.stow();
            if (AvionicsConfig::EnableSerial) {
              Serial.println(F("RX Cmd: Stow Parachute"));
            }
          } else if (buffer[2] == 0xAA && buffer[3] == 0x00) {
            servo.deploy();
            if (AvionicsConfig::EnableSerial) {
              Serial.println(F("RX Cmd: Deploy Parachute"));
            }
          } else if (buffer[2] == 0x01) {
            if (isValidFlightState(buffer[3])) {
              fsm.setState(static_cast<FlightState>(buffer[3]));
              if (AvionicsConfig::EnableSerial) {
                Serial.print(F("RX Cmd: Set State "));
                Serial.println(buffer[3]);
              }
            }
          } else if (buffer[2] == 0x67 && buffer[3] == 0x67) {
            if (fsm.currentState() < FlightState::Flight) {
              measurements.resetBaseline();
              if (AvionicsConfig::EnableSerial) {
                Serial.println(F("RX Cmd: Reset Baseline"));
              }
            } else {
              if (AvionicsConfig::EnableSerial) {
                Serial.println(F("RX Cmd: Reset Rejected (Already in flight)"));
              }
            }
          }
        }
      }
    } else {
      // Clear the buffer/radio state if it wasn't the correct length
      radio.standby();
    }
    
    // We need to re-enter RX mode on the next tick
    inRxMode_ = false;
  }
}
