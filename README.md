# Segfault Rocket Avionics Firmware

Highly optimized, cooperative, and non-blocking firmware for a TTGO T-Beam ESP32 board used as high-performance rocket avionics. 

The firmware implements real-time sensor ingestion, Kalman filtering, state-dependent flash logging, custom LoRa telemetry/remote control, and a localized WiFi Recovery access point for post-flight log downloads.

---

## System Runtime Flow

The top-level execution is orchestrated in `src/main.cpp`:

1. **Setup Sequence**:
   - Initializes Serial console (debug mode).
   - Opens the non-volatile storage (NVS) flash partition.
   - Restores the saved FSM state from NVS (defaults to `BeforeLaunch` if empty).
   - Initializes hardware helper modules: `FlightLogger` and `RecoveryService`.
   - Attaches NVS parameters to the `FlightFsm` and `StateLogic` controllers.
   - Restores previous flight time and baseline parameters from NVS if booting into a post-launch state; otherwise, begins baseline calibration.
   - Initializes sensor buses and hardware pins (I2C PMU, barometric pressure, IMU, GPS UART, current sensor, Hall inputs, and parachute servo).
   - Initializes the LoRa radio module.
2. **Main Execution Loop (`loop()`)**:
   Runs a cooperative tick-based loop without blocking operations or synchronous delays:
   - `measurements.tick(nowMs)`: Samples all hardware sensors at the configured rate and updates the state estimate.
   - `stateLogic.tick(...)`: Evaluates transition conditions, updates FSM states, controls telemetry intervals, and drives the parachute servo.
   - `telemetry.tick(...)`: Transmits packet telemetry via LoRa when the active interval expires.
   - `remoteControl.tick(...)`: Listens for incoming LoRa command overrides from the ground station between telemetry periods.

---

## Finite State Machine (Flight FSM)

The avionics state is managed centrally by a state machine containing five states:

1. **`BeforeLaunch`**: Telemetry runs at a slow heartbeat rate. Baselining calibration is active to settle sensor noise.
2. **`Armed`**: Telemetry runs at a slow heartbeat rate. State-dependent logging is active (recording only on acceleration events).
3. **`Flight`**: High-rate telemetry is enabled. Flight logging runs unconditionally. Baselining calibration is locked.
4. **`ApogeeReached`**: Triggers immediate deployment of the parachute servo, then transitions to ChuteDeployed.
5. **`ChuteDeployed`**: High-rate telemetry is enabled. Flight logging is active only when falling. The local WiFi Recovery AP is active.

### State Transitions

- **`BeforeLaunch` ➡️ `Armed`**: Initiated manually or via a remote control state command.
- **`Armed` ➡️ `Flight`**: Automatically triggered when Z-axis vertical acceleration (compensated by the ground baseline) exceeds the configured launch threshold for the required minimum duration.
- **`Flight` ➡️ `ApogeeReached`**: Automatically triggered when either:
  - **Primary Trigger**: Filtered altitude falls below the maximum altitude reached by the configured drop margin, and the vertical velocity falls below the apogee threshold.
  - **Backup Trigger**: The flight timer exceeds the apogee failsafe duration.
- **`ApogeeReached` ➡️ `ChuteDeployed`**: Transitions automatically on the next clock cycle after command deployment is dispatched to the parachute servo.

### State-Transition Event Actions

- **Entering `Armed`**: Automatically triggers a flash erase of the `flightlog` partition to clear old flight logs and prepare sectors for high-speed writes.
- **Entering `Flight`**: Disables the sensor baselining calibration, locks the current ground altitude and gravity acceleration offsets, writes the baseline parameters to NVS, and starts the elapsed flight timer.
- **Exiting `Flight` back to pre-launch states**: Re-enables the sensor baselining calibration.
- **Entering `ChuteDeployed`**: Powers up the WiFi radio, configures the soft Access Point, and spins up the background web server task on Core 1.
- **Exiting `ChuteDeployed`**: Shuts down the WiFi AP, closes the web server, and disables the WiFi radio.

---

## Telemetry and Logging Services

### LoRa Telemetry Transmission
Telemetric snapshots are formatted and dispatched using non-blocking packet transmits over the LoRa transceiver. Telemetry intervals are dynamic and managed by the current FSM state (slow heartbeat pre-flight, fast telemetry during flight/descent).

The transmitted telemetry packet includes:
- Frame sync words and timestamp offsets.
- Active FSM state flags.
- Raw IMU linear accelerations and angular gyro rates.
- Kalman-filtered altitude (AGL).
- Raw barometric pressure.
- Triboelectric sensor probe voltage.
- Main battery voltage.
- GPS coordinates and fix quality.
- Kalman-filtered vertical velocity.
-KY-024 Hall sensor values.

### State-Dependent Flash Logging
High-speed flight data is written to a dedicated raw SPI flash partition (`flightlog`). To conserve flash memory write cycles and queue buffers, logging uses state-dependent rules:
- **`Armed`**: Logs only when Z-axis acceleration rises above the launch baseline.
- **`Flight` & `ApogeeReached`**: Logs unconditionally at the high-frequency rate.
- **`ChuteDeployed`**: Logs only when falling (vertical velocity is below a negative threshold).

The log packet includes all telemetry parameters plus raw GPS satellites, GPS fix validity, system sensor health flags, internal board temperature, and is protected with a Fletcher16 checksum.

---

## Baselining and Calibration
Before launch, the avionics runs an exponential moving average over the barometric pressure and Z-axis accelerometer readings to establish a ground baseline (altitude above sea level and vertical gravity offset). 

When launch is detected, baselining is locked and stored in NVS. This baseline is used throughout the flight to compute:
- Filtered AGL (above ground level) altitude.
- Gravity-compensated vertical acceleration for the state estimator.
- Re-baselining is allowed prior to launch via remote command.

---

## Remote Control Override System
In between telemetry transmissions, the LoRa radio enters a non-blocking receive window to process incoming commands. Commands can override the autonomous logic to:
- Deploy the parachute (forces servo to deploy angle).
- Stow the parachute (forces servo to stow angle).
- Set the active FSM state manually.
- Trigger a sensor baseline reset (allowed only when not in flight).

---

## Recovery Access Point & File Server
Upon entering the `ChuteDeployed` state, the avionics spins up a WiFi soft Access Point and hosts a localized HTTP web server. 

Connecting to the AP and requesting the root URL starts a raw binary stream of the entire flash logging partition as a file download. The server processes downloads in small blocks, yielding to the system task scheduler to prevent triggering the hardware watchdog timer during transfer.
