#pragma once

#include "ParachuteServo.h"
#include "FlightFsm.h"

class RemoteControlService {
public:
  void tick(ParachuteServo &servo, FlightFsm &fsm);
private:
  bool inRxMode_ = false;
};
