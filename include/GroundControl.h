#pragma once

#include <Arduino.h>
#include "AvionicsConfig.h"
#include "servo.h"

namespace GroundControl {

enum class GroundControlCommand : uint8_t {
  StowServo = AvionicsConfig::GroundControlCommandStowServo,
  DeployServo = AvionicsConfig::GroundControlCommandDeployServo,
};

#pragma pack(push, 1)
struct GroundControlPacket {
  uint16_t magic;
  uint8_t command;
  uint8_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(GroundControlPacket) <= 4, "GroundControlPacket must fit in 4 bytes");

void attachServo(ServoService &servo);
void handlePacket(const uint8_t *bytes, size_t length, float rssi, float snr);

}  // namespace GroundControl
