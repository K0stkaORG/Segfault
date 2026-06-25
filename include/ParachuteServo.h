#pragma once

#include <ESP32Servo.h>

class ParachuteServo {
public:
  bool begin(int controlPin);
  bool begin(int controlPin, int initialAngle);
  
  bool attached() const;
  int controlPin() const;
  int angle() const;

  bool writeAngle(int angle);
  bool stow();
  bool deploy();

private:
  int clampAngle(int angle);

  Servo servo_;
  int controlPin_ = -1;
  int angle_ = 0;
  bool attached_ = false;
};
