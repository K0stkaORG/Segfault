#include "GroundControl.h"

#include <string.h>
#include "AvionicsConfig.h"

namespace GroundControl {
namespace {
ServoService *parachuteServo = nullptr;

bool decodePacket(const uint8_t *bytes, size_t length, GroundControlPacket &packet) {
  if (bytes == nullptr || length != sizeof(GroundControlPacket)) {
    return false;
  }

  memcpy(&packet, bytes, sizeof(packet));
  if (packet.magic != AvionicsConfig::GroundControlPacketMagic) {
    return false;
  }

  return true;
}

void printPacket(const uint8_t *bytes, size_t length, float rssi, float snr, const __FlashStringHelper *status) {
  if (!AvionicsConfig::EnableSerial) {
    return;
  }

  Serial.print(F("RX control "));
  Serial.print(status);
  Serial.print(' ');
  Serial.print(length);
  Serial.print(F(" bytes RSSI "));
  Serial.print(rssi);
  Serial.print(F(" dBm SNR "));
  Serial.print(snr);
  Serial.print(F(" dB:"));

  for (size_t i = 0; bytes != nullptr && i < length; ++i) {
    Serial.print(' ');
    if (bytes[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(bytes[i], HEX);
  }

  Serial.println();
}
}  // namespace

void attachServo(ServoService &servo) {
  parachuteServo = &servo;
}

void handlePacket(const uint8_t *bytes, size_t length, float rssi, float snr) {
  GroundControlPacket packet{};
  if (!decodePacket(bytes, length, packet)) {
    printPacket(bytes, length, rssi, snr, F("ignored"));
    return;
  }

  bool handled = false;
  switch (static_cast<GroundControlCommand>(packet.command)) {
    case GroundControlCommand::StowServo:
      handled = parachuteServo != nullptr && parachuteServo->stow();
      break;
    case GroundControlCommand::DeployServo:
      handled = parachuteServo != nullptr && parachuteServo->deploy();
      break;
    default:
      handled = false;
      break;
  }

  printPacket(bytes, length, rssi, snr, handled ? F("handled") : F("unhandled"));
}

}  // namespace GroundControl
