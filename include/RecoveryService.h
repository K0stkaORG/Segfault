#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_partition.h"

class RecoveryService {
 public:
  bool begin();
  void start();
  void stop();
  void tick();
  bool isStarted() const { return started_; }

 private:
  WebServer server_{80};
  const esp_partition_t* partition_ = nullptr;
  bool started_ = false;
};
