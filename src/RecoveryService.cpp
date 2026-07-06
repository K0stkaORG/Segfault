#include "RecoveryService.h"
#include "AvionicsConfig.h"

bool RecoveryService::begin() {
  partition_ = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, static_cast<esp_partition_subtype_t>(0x99), "flightlog");
  return partition_ != nullptr;
}

void RecoveryService::start() {
  if (started_) return;

  WiFi.mode(WIFI_AP);

  IPAddress localIp(AvionicsConfig::RecoveryLocalIp[0],
                    AvionicsConfig::RecoveryLocalIp[1],
                    AvionicsConfig::RecoveryLocalIp[2],
                    AvionicsConfig::RecoveryLocalIp[3]);
  IPAddress gateway(AvionicsConfig::RecoveryGatewayIp[0],
                    AvionicsConfig::RecoveryGatewayIp[1],
                    AvionicsConfig::RecoveryGatewayIp[2],
                    AvionicsConfig::RecoveryGatewayIp[3]);
  IPAddress subnet(AvionicsConfig::RecoverySubnetMask[0],
                   AvionicsConfig::RecoverySubnetMask[1],
                   AvionicsConfig::RecoverySubnetMask[2],
                   AvionicsConfig::RecoverySubnetMask[3]);

  WiFi.softAPConfig(localIp, gateway, subnet);
  WiFi.softAP(AvionicsConfig::RecoveryWifiSsid, AvionicsConfig::RecoveryWifiPassword);

  server_.on("/", HTTP_GET, [this]() {
    if (partition_ == nullptr) {
      server_.send(404, "text/plain", "Flight log partition not found!");
      return;
    }

    server_.setContentLength(partition_->size);
    server_.sendHeader("Content-Disposition", "attachment; filename=\"flightlog.bin\"");
    server_.send(200, "application/octet-stream", "");

    uint8_t buffer[1024];
    uint32_t offset = 0;
    uint32_t totalSize = partition_->size;

    while (offset < totalSize && server_.client().connected()) {
      uint32_t chunk = std::min((uint32_t)sizeof(buffer), totalSize - offset);
      if (esp_partition_read(partition_, offset, buffer, chunk) != ESP_OK) {
        break;
      }
      server_.sendContent((const char*)buffer, chunk);
      offset += chunk;
      vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks to prevent WDT reset
    }
  });

  server_.begin();
  started_ = true;

  // Spin up the server processing loop on a background task pinned to Core 1
  xTaskCreatePinnedToCore(
      RecoveryService::serverTask,
      "recovery_server",
      4096,
      this,
      1,
      &serverTaskHandle_,
      1
  );

  if (AvionicsConfig::EnableSerial) {
    Serial.println(F("Recovery AP Started. SSID: TIPRocket, IP: 172.27.67.1"));
  }
}

void RecoveryService::stop() {
  if (!started_) return;

  started_ = false;

  if (serverTaskHandle_ != nullptr) {
    vTaskDelete(serverTaskHandle_);
    serverTaskHandle_ = nullptr;
  }

  server_.close();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  if (AvionicsConfig::EnableSerial) {
    Serial.println(F("Recovery AP Stopped."));
  }
}

void RecoveryService::serverTask(void *param) {
  RecoveryService *self = static_cast<RecoveryService*>(param);
  while (self->started_) {
    self->server_.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to prevent CPU core starvation
  }
  vTaskDelete(NULL);
}
