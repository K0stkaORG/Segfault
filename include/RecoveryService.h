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
  bool isStarted() const { return started_; }

 private:
  static void serverTask(void *param);

  WebServer server_{80};
  const esp_partition_t* partition_ = nullptr;
  TaskHandle_t serverTaskHandle_ = nullptr;
  volatile bool started_ = false;
};
