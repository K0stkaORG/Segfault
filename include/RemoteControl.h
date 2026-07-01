#pragma once

#include "ParachuteServo.h"

class RemoteControlService {
public:
  void tick(ParachuteServo &servo);
private:
  bool inRxMode_ = false;
};
