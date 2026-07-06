#include "ParachuteServo.h"

#include "AvionicsConfig.h"

bool ParachuteServo::begin(int controlPin) {
  controlPin_ = controlPin;

  servo_.setPeriodHertz(AvionicsConfig::ParachuteServoFrequencyHz);
  servo_.attach(controlPin_,
                AvionicsConfig::ParachuteServoMinPulseUs,
                AvionicsConfig::ParachuteServoMaxPulseUs);
  attached_ = servo_.attached();
  
  return attached_;
}

bool ParachuteServo::attached() const {
  return attached_;
}

bool ParachuteServo::writeAngle(int angle) {
  if (!attached_) {
    return false;
  }

  angle_ = clampAngle(angle);
  servo_.write(angle_);
  return true;
}

bool ParachuteServo::stow() {
  return writeAngle(AvionicsConfig::ParachuteServoStowedAngle);
}

bool ParachuteServo::deploy() {
  return writeAngle(AvionicsConfig::ParachuteServoDeployedAngle);
}

int ParachuteServo::clampAngle(int angle) {
  if (angle < AvionicsConfig::ParachuteServoMinAngle) {
    return AvionicsConfig::ParachuteServoMinAngle;
  }
  if (angle > AvionicsConfig::ParachuteServoMaxAngle) {
    return AvionicsConfig::ParachuteServoMaxAngle;
  }

  return angle;
}
