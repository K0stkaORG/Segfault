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
    WiFiClient client = server_.client();

    if (partition_ == nullptr) {
      server_.send(404, "text/plain", "Flight log partition not found!");
      return;
    }

    server_.sendHeader("Content-Disposition", "attachment; filename=\"flightlog.bin\"");
    server_.sendHeader("Content-Length", String(partition_->size));
    server_.send(200, "application/octet-stream", "");

    uint8_t buffer[1024];
    uint32_t offset = 0;
    uint32_t totalSize = partition_->size;

    while (offset < totalSize && client.connected()) {
      uint32_t chunk = std::min((uint32_t)sizeof(buffer), totalSize - offset);
      if (esp_partition_read(partition_, offset, buffer, chunk) != ESP_OK) {
        break;
      }
      client.write(buffer, chunk);
      offset += chunk;
      vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks to prevent WDT reset
    }
  });

  server_.begin();
  started_ = true;

  if (AvionicsConfig::EnableSerial) {
    Serial.println(F("Recovery AP Started. SSID: TIPRocket, IP: 172.27.67.1"));
  }
}

void RecoveryService::stop() {
  if (!started_) return;

  server_.close();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  started_ = false;

  if (AvionicsConfig::EnableSerial) {
    Serial.println(F("Recovery AP Stopped."));
  }
}

void RecoveryService::tick() {
  if (started_) {
    server_.handleClient();
  }
}
