# Include Directory

This directory contains the public interfaces for the firmware modules.

## Headers

### `AvionicsConfig.h`

Central configuration for constants used across the firmware:

- serial baud rate
- measurement and telemetry intervals
- LoRa frequency, sync word, and pins
- I2C pins, clock, and sensor addresses
- GPS UART pins and parser budget
- KY-024 input pins
- base GPS location
- NVS namespace

### `FlightFsm.h`

Defines `FlightState` and the `FlightFsm` class.

`FlightFsm` stores the current state and persists changes after a `PersistentStore` has been attached. Use `setState()` for transitions; do not add separate state variables elsewhere.

### `Measurements.h`

Defines measurement data structures and `MeasurementService`.

Important structures:

- `Bmi270RawSample`
- `GpsSample`
- `Ina226Sample`
- `Ky024Sample`
- `MeasurementSnapshot`

`MeasurementSnapshot` is the data passed into FSM logic and telemetry. New measured values should be added here only when they are part of the shared avionics state.

### `PersistentStore.h`

Defines the NVS wrapper used for data that must survive resets:

- boot count
- FSM state
- packet counter
- hardware init status

### `StateLogic.h`

Defines the FSM behavior layer.

`StateLogic::tick()` receives current time, FSM state, latest measurements, and telemetry control. It currently implements the `BeforeLaunch` heartbeat behavior.

### `Telemetry.h`

Defines the packed telemetry packet and `TelemetryService`.

`TelemetryService` owns:

- LoRa initialization
- packet counter
- telemetry interval
- send timeout
- packet packing
- packet-counter persistence after successful send

The telemetry packet is 29 bytes and is checked at compile time with `static_assert`.
