#include "servo.h"

#include "AvionicsConfig.h"

bool ServoService::begin(int controlPin) {
  return begin(controlPin, AvionicsConfig::ParachuteServoStowedAngle);
}

bool ServoService::begin(int controlPin, int initialAngle) {
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

bool ServoService::attached() const {
  return attached_;
}

int ServoService::controlPin() const {
  return controlPin_;
}

int ServoService::angle() const {
  return angle_;
}

bool ServoService::writeAngle(int angle) {
  if (!attached_) {
    return false;
  }

  angle_ = clampAngle(angle);
  servo_.write(angle_);
  return true;
}

bool ServoService::stow() {
  return writeAngle(AvionicsConfig::ParachuteServoStowedAngle);
}

bool ServoService::deploy() {
  return writeAngle(AvionicsConfig::ParachuteServoDeployedAngle);
}

int ServoService::clampAngle(int angle) {
  if (angle < AvionicsConfig::ParachuteServoMinAngle) {
    return AvionicsConfig::ParachuteServoMinAngle;
  }
  if (angle > AvionicsConfig::ParachuteServoMaxAngle) {
    return AvionicsConfig::ParachuteServoMaxAngle;
  }

  return angle;
}
