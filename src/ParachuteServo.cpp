#include "ParachuteServo.h"

#include "AvionicsConfig.h"

bool ParachuteServo::begin(int controlPin) {
  return begin(controlPin, AvionicsConfig::ParachuteServoStowedAngle);
}

bool ParachuteServo::begin(int controlPin, int initialAngle) {
  controlPin_ = controlPin;
  angle_ = clampAngle(initialAngle);

  servo_.setPeriodHertz(AvionicsConfig::ParachuteServoFrequencyHz);
  servo_.attach(controlPin_,
                AvionicsConfig::ParachuteServoMinPulseUs,
                AvionicsConfig::ParachuteServoMaxPulseUs);
  attached_ = servo_.attached();
  if (!attached_) {
    return false;
  }

  servo_.write(angle_);
  return attached_;
}

bool ParachuteServo::attached() const {
  return attached_;
}

int ParachuteServo::controlPin() const {
  return controlPin_;
}

int ParachuteServo::angle() const {
  return angle_;
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
