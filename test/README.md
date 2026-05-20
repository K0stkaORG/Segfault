# Tests

No automated tests are currently implemented.

The current firmware is verified manually by building and uploading the PlatformIO environment `ttgo-t-beam`, then checking Serial output and hardware behavior.

Manual checks used for the current skeleton:

- startup prints NVS, boot count, FSM state, BMP280, BMI270, NEO-6M, measurement, and LoRa status
- first boot defaults to `BeforeLaunch`
- restored FSM state survives restart through NVS
- telemetry packet counter survives restart through NVS
- `BeforeLaunch` sends heartbeat telemetry at `BeforeLaunchHeartbeatIntervalMs`
- transmitted telemetry bytes are printed on Serial
- firmware continues running when a sensor is unavailable
