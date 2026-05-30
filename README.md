# Segfault Rocket Avionics Firmware

Firmware skeleton for a TTGO T-Beam ESP32 used as rocket avionics. The current code initializes persistent storage, telemetry, and measurements, then runs a cooperative loop where measurements are refreshed, FSM logic is evaluated, and telemetry is sent when its configured interval expires.

## Current Runtime Flow

`src/main.cpp` owns the top-level orchestration:

1. Start Serial at `115200`.
2. Open ESP32 NVS through `PersistentStore`.
3. Attach NVS persistence to `FlightFsm`.
4. Restore the saved FSM state, defaulting to `BeforeLaunch` if no valid state exists.
5. Restore the telemetry packet counter.
6. Initialize measurements: BMP280, BMI270, NEO-6M GPS UART, INA226, AXP2101 PMU, and KY-024 inputs.
7. Initialize the parachute servo at the configured stowed angle.
8. Initialize LoRa telemetry.
9. Save current hardware init status into NVS.
10. In `loop()`:
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
- LoRa frequency: `439700000 Hz`
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
- INA226 bus/current/power telemetry over I2C
- AXP2101 PMU battery voltage over I2C
- KY-024 analog Hall sensor ADC

`MeasurementService::tick()` runs at `AvionicsConfig::MeasurementIntervalMs`. GPS parsing is bounded by `MaxGpsBytesPerTick` so one tick cannot drain an unbounded UART backlog.

The current `MeasurementSnapshot` contains latest sensor health flags, latest sensor read flags, raw IMU values, pressure, temperature, GPS fix data, INA226 values, AXP2101 battery millivolts, and KY-024 values.

### `StateLogic`

Contains per-state decisions. It currently implements only `BeforeLaunch`.

Current `BeforeLaunch` behavior:

- keep the current FSM state unchanged
- set telemetry to `BeforeLaunchHeartbeatIntervalMs`

### `ServoService`

Owns the parachute deployment servo.

Current behavior:

- attaches the servo once during startup on `ParachuteServoPin`
- sets the servo to `ParachuteServoStowedAngle`
- exposes `writeAngle(angle)`, `stow()`, and `deploy()`

The servo API is immediate and does not use delays or sweep loops.

### `GroundControl`

Owns incoming LoRa control-packet handling.

Current behavior:

- receives decoded LoRa packet bytes from `TelemetryService`
- accepts fixed 4-byte control packets with magic, command, and reserved byte
- directly calls parachute servo `stow()` or `deploy()` for valid commands
- does not change FSM state or flight logic
- prints received control bytes and radio metadata to Serial only when Serial output is enabled

### `Telemetry`

Owns LoRa setup, incoming control packet receive scheduling, telemetry packet packing, send scheduling, packet counter management, and packet-counter persistence after successful send.

Current packet format is the packed `RocketTelemetry` struct from `defvals.txt`. It is checked with:

```cpp
static_assert(sizeof(RocketTelemetry) == 29, "RocketTelemetry must be 29 bytes");
```

The `batteryVoltage` byte is encoded from AXP2101 battery millivolts in `20 mV` units. A receiver decodes it as `batteryVoltage * 20`.

Telemetry scheduling is controlled by:

- `telemetry.setInterval(intervalMs)`
- `telemetry.disable()`
- `telemetry.tick(nowMs, fsm, measurement, persistentStore)`

`TelemetryService::tick()` first services the radio, including completed TX and completed RX packets. It then checks whether the telemetry timeout has fired, builds a packet from the latest FSM state and measurement snapshot, sends it over LoRa, and persists the packet counter after successful send.

Transmission uses RadioLib `startTransmit()` with the SX1262 packet-sent callback. RX uses RadioLib `startReceive()` with the SX1262 packet-received callback whenever TX is not in progress. Every accepted TX packet is also printed to Serial as a hex byte dump.

## Non-Blocking Behavior

Runtime code is cooperative and tick-based. The main loop does not use `delay()` and does not wait for telemetry or GPS data.

Bounded/non-blocking parts:

- telemetry is scheduled by timestamp
- LoRa TX uses RadioLib async `startTransmit()`
- LoRa RX uses RadioLib async `startReceive()` while the radio is not transmitting
- GPS parser consumes at most `MaxGpsBytesPerTick` bytes per measurement tick
- missing sensors do not stop the firmware

Known synchronous parts:

- BMP280 reads are synchronous I2C calls
- BMI270 reads are synchronous I2C calls
- ADC reads are synchronous
- AXP2101 battery voltage reads are synchronous I2C calls
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

- `jgromes/RadioLib`
- `adafruit/Adafruit BMP280 Library`
- `adafruit/Adafruit Unified Sensor`
- `sparkfun/SparkFun BMI270 Arduino Library`
- `mikalhart/TinyGPSPlus`
- `wollewald/INA226_WE`
- `lewisxhe/XPowersLib`
- `madhephaestus/ESP32Servo`

## Current Hardware Assumptions

Board:

- PlatformIO environment: `ttgo-t-beam`
- Framework: Arduino

Radio:

- SX1262 LoRa module through RadioLib
- SPI pins from `AvionicsConfig`
- DIO1 and BUSY pins from `AvionicsConfig`
- frequency `439700000 Hz`

Sensors:

- BMP280 at I2C address `0x77`
- BMI270 at I2C address `0x68`
- NEO-6M GPS on UART1
- INA226 at I2C address `0x40`
- AXP2101 PMU for 18650 battery voltage monitoring

Inputs/outputs:

- KY-024 analog input: `GPIO39`
- KY-024 digital input: `GPIO14`
- parachute servo signal: `GPIO13`

## Current Limitations

No flight detection, RBF logic, apogee detection, automatic parachute deployment decision, buzzer behavior, filtering, or calibration logic has been implemented yet.
