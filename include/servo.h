#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoService {
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
  static int clampAngle(int angle);

  Servo servo_;
  bool attached_ = false;
  int controlPin_ = -1;
  int angle_ = 0;
};
