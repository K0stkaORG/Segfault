#pragma once

#include "ParachuteServo.h"
#include "FlightFsm.h"
#include "Measurements.h"

class RemoteControlService {
public:
  void tick(ParachuteServo &servo, FlightFsm &fsm, MeasurementService &measurements);
private:
  bool inRxMode_ = false;
};
