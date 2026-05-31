#include "GroundControl.h"

#include <stdio.h>
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

void printPacket(const uint8_t *bytes,
                 size_t length,
                 float rssi,
                 float snr,
                 const char *status) {
  if (!AvionicsConfig::EnableSerial) {
    return;
  }

  char line[160] = {};
  size_t pos = 0;
  const int written = snprintf(line + pos, sizeof(line) - pos,
                               "RX control %s %u bytes RSSI %.2f dBm SNR %.2f dB:",
                               status == nullptr ? "unknown" : status,
                               static_cast<unsigned>(length),
                               static_cast<double>(rssi),
                               static_cast<double>(snr));
  if (written <= 0) {
    return;
  }

  pos += static_cast<size_t>(written);
  for (size_t i = 0; bytes != nullptr && i < length; ++i) {
    if (pos + 3 >= sizeof(line)) {
      break;
    }
    line[pos++] = ' ';
    const uint8_t value = bytes[i];
    line[pos++] = "0123456789ABCDEF"[(value >> 4) & 0x0F];
    line[pos++] = "0123456789ABCDEF"[value & 0x0F];
  }

  if (pos + 1 >= sizeof(line)) {
    pos = sizeof(line) - 2;
  }
  line[pos++] = '\n';

  if (Serial.availableForWrite() < pos) {
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), pos);
}
}  // namespace

void attachServo(ServoService &servo) {
  parachuteServo = &servo;
}

void handlePacket(const uint8_t *bytes, size_t length, float rssi, float snr) {
  GroundControlPacket packet{};
  if (!decodePacket(bytes, length, packet)) {
    printPacket(bytes, length, rssi, snr, "ignored");
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

  printPacket(bytes, length, rssi, snr, handled ? "handled" : "unhandled");
}

}  // namespace GroundControl
