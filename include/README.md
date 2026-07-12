# Include Directory

This directory contains the public interfaces for the rocket avionics firmware modules.

## Headers

### `AvionicsConfig.h`
Central configuration namespace defining system-wide constants:
- Serial console settings.
- Sampling and transmission intervals.
- LoRa radio frequencies, sync words, and SPI pin assignments.
- I2C bus pins, speeds, and device sensor addresses.
- GPS hardware pins and parsing budget parameters.
- KY-024 Hall sensor pin mappings.
- Servo control pulse widths and angles.
- State-machine thresholds and failsafe timers.
- Recovery Access Point credentials and IP parameters.

### `FlightFsm.h`
Defines the `FlightState` enum (BeforeLaunch, Armed, Flight, ApogeeReached, ChuteDeployed) and the `FlightFsm` class. 

The FSM tracks the active flight state and registers listeners to trigger hardware events during state transitions. All changes are persisted to NVS.

### `FlightLogger.h`
Defines the `LogPacket` struct and the `FlightLogger` service.

* **LogPacket**: Exactly 40 bytes, packed to 1-byte alignment. Contains frame markers, timestamp offsets, state flags, raw IMU values, Kalman-filtered altitude and vertical speed, raw pressure, battery voltage, GPS offset coords, Hall values, GPS quality, board temperature, and a Fletcher16 checksum.
* **FlightLogger**: Erases the partition and manages a FreeRTOS task that pulls log packets from a queue to write them asynchronously to the SPI flash partition.

### `LoRaRadio.h`
Declares the global RadioLib radio object to share the SX1262 LoRa module between the Telemetry sender and RemoteControl receiver.

### `Measurements.h`
Defines sensor sample data structures and the `MeasurementService`.

The service reads the physical sensors (barometer, IMU, GPS, current sensor, PMU, Hall sensor) and runs a Kalman Filter estimating above-ground-level (AGL) altitude and vertical speed, utilizing a pre-flight exponential moving average ground baseline.

### `ParachuteServo.h`
Defines the `ParachuteServo` class. Coordinates stowing and deploying the parachute recovery mechanism.

### `PersistentStore.h`
A thin wrapper around ESP32 Preferences/NVS. Persists the FSM state, sensor ground baselines, and elapsed flight time to survive mid-flight power cuts.

### `RecoveryService.h`
Defines the `RecoveryService` class. Starts the soft WiFi AP and web server, and manages streaming the raw flash logs back to the recovery team.

### `RemoteControl.h`
Defines the `RemoteControlService` class. Processes remote commands received over LoRa.

### `StateLogic.h`
Defines the FSM transition scheduler. Evaluates sensors and timings to drive FSM state transitions, controls the telemetry rate, and handles parachute servo trigger and recovery AP events.

### `Telemetry.h`
Defines the `RocketTelemetry` struct and the `TelemetryService`.

* **RocketTelemetry**: Exactly 33 bytes, packed to 1-byte alignment. Transmits telemetric variables including IMU, GPS offsets, Kalman altitude/velocity, and Hall sensor voltages.
* **TelemetryService**: Manages the LoRa transceiver timing scheduler andasync transmissions.
