# Segfault Rocket Avionics Firmware

Firmware skeleton for a TTGO T-Beam ESP32 used as rocket avionics. The current code initializes persistent storage, telemetry, and measurements, then runs a cooperative loop where measurements are refreshed, FSM logic is evaluated, and telemetry is sent when its configured interval expires.

## Current Runtime Flow

`src/main.cpp` owns the top-level orchestration:

1. Start Serial at `115200`.
2. Open ESP32 NVS through `PersistentStore`.
3. Attach NVS persistence to `FlightFsm`.
4. Restore the saved FSM state, defaulting to `BeforeLaunch` if no valid state exists.
5. Restore the telemetry packet counter.
6. Initialize measurements: BMP280, BMI270, NEO-6M GPS UART, ADC pins, buzzer pin, and breakaway input pin.
7. Initialize LoRa telemetry.
8. Save current hardware init status into NVS.
9. In `loop()`:
   - `measurements.tick(nowMs)`
   - `stateLogic.tick(nowMs, flightFsm, measurements.latest(), telemetry)`
   - `telemetry.tick(nowMs, flightFsm, measurements.latest(), persistentStore)`

There are no flight transitions implemented yet. The only active FSM behavior is the `BeforeLaunch` state setting telemetry to the heartbeat interval.

## Modules

### `AvionicsConfig`

Central constants for timing, pins, I2C addresses, LoRa settings, GPS settings, base location, and the NVS namespace.

Current important values:

- Serial: `115200`
- Measurement interval: `20 ms`
- Before-launch heartbeat interval: `10000 ms`
- Fast telemetry interval constant: `200 ms`
- LoRa frequency: `433375000 Hz`
- LoRa sync word: `0x67`
- GPS: UART1, `9600` baud, RX `GPIO34`, TX `GPIO12`
- I2C: SDA `GPIO21`, SCL `GPIO22`, `400 kHz`

### `PersistentStore`

Thin wrapper around ESP32 `Preferences` / NVS.

Persisted values:

- boot count
- FSM state
- telemetry packet counter
- last hardware init status bitfield

If NVS is unavailable, load operations fall back to safe defaults and save operations become no-ops.

### `FlightFsm`

Holds the active flight state.

States currently defined:

- `BeforeLaunch`
- `RBFRemoved`
- `Flight`
- `ApogeeReached`
- `ChuteDeployed`

`FlightFsm::setState()` persists every state change after `attachPersistentStore()` has been called in setup. First boot defaults to `BeforeLaunch` through `PersistentStore::loadFlightState()`.

`stateFlags()` currently maps the telemetry flags as:

- bit 0: launch/flight has occurred
- bit 1: apogee reached
- bit 2: chute deployed

### `Measurements`

Owns sensor and input sampling.

Current sources:

- BMP280 pressure and temperature over I2C
- BMI270 raw accelerometer and gyroscope values over I2C using Bosch SensorAPI
- NEO-6M GPS NMEA parsing over UART using TinyGPS++
- tribo voltage ADC
- battery voltage ADC

`MeasurementService::tick()` runs at `AvionicsConfig::MeasurementIntervalMs`. GPS parsing is bounded by `MaxGpsBytesPerTick` so one tick cannot drain an unbounded UART backlog.

The current `MeasurementSnapshot` contains latest sensor health flags, latest sensor read flags, raw IMU values, pressure, temperature, GPS fix data, and ADC values.

### `StateLogic`

Contains per-state decisions. It currently implements only `BeforeLaunch`.

Current `BeforeLaunch` behavior:

- keep the current FSM state unchanged
- set telemetry to `BeforeLaunchHeartbeatIntervalMs`

### `Telemetry`

Owns LoRa setup, telemetry packet packing, send scheduling, packet counter management, and packet-counter persistence after successful send.

Current packet format is the packed `RocketTelemetry` struct from `defvals.txt`. It is checked with:

```cpp
static_assert(sizeof(RocketTelemetry) == 27, "RocketTelemetry must be 27 bytes");
```

Telemetry scheduling is controlled by:

- `telemetry.setInterval(intervalMs)`
- `telemetry.disable()`
- `telemetry.tick(nowMs, fsm, measurement, persistentStore)`

`TelemetryService::tick()` checks whether the timeout has fired, builds a packet from the latest FSM state and measurement snapshot, sends it over LoRa, and persists the packet counter after successful send.

Transmission uses `LoRa.endPacket(true)` for asynchronous TX. Every accepted packet is also printed to Serial as a hex byte dump.

## Non-Blocking Behavior

Runtime code is cooperative and tick-based. The main loop does not use `delay()` and does not wait for telemetry or GPS data.

Bounded/non-blocking parts:

- telemetry is scheduled by timestamp
- LoRa TX uses async `endPacket(true)`
- GPS parser consumes at most `MaxGpsBytesPerTick` bytes per measurement tick
- missing sensors do not stop the firmware

Known synchronous parts:

- BMP280 reads are synchronous I2C calls
- BMI270 reads are synchronous I2C calls
- ADC reads are synchronous
- NVS writes happen when FSM state changes and after successful telemetry sends
- Serial byte dumps can slow down telemetry during testing

## Persistence and Restart Behavior

On restart:

- boot count increments
- FSM state is restored from NVS, or defaults to `BeforeLaunch`
- packet counter is restored from NVS
- telemetry interval is not persisted; it is derived again by `StateLogic` from the restored FSM state

State transitions must use `flightFsm.setState(...)` so the new state is persisted.

## Current Build Dependencies

Declared in `platformio.ini`:

- `sandeepmistry/LoRa`
- `adafruit/Adafruit BMP280 Library`
- `adafruit/Adafruit Unified Sensor`
- `sparkfun/SparkFun BMI270 Arduino Library`
- `mikalhart/TinyGPSPlus`

## Current Hardware Assumptions

Board:

- PlatformIO environment: `ttgo-t-beam`
- Framework: Arduino

Radio:

- SX127x LoRa module through `LoRa.h`
- SPI pins from `AvionicsConfig`
- frequency `433375000 Hz`

Sensors:

- BMP280 at I2C address `0x76`
- BMI270 at I2C address `0x68`
- NEO-6M GPS on UART1

Inputs/outputs:

- tribo ADC placeholder: `GPIO36`
- battery ADC placeholder: `GPIO35`
- breakaway input placeholder: `GPIO33`
- buzzer placeholder: `GPIO25`

## Current Limitations

No flight detection, RBF logic, apogee detection, parachute deployment, buzzer behavior, filtering, or calibration logic has been implemented yet.
