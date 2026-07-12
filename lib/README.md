# Local Libraries

No project-private PlatformIO libraries are currently stored in this directory.

The firmware currently uses source files in `src/` and external libraries declared in `platformio.ini`.

## External Dependencies

The following libraries are declared in `platformio.ini` and automatically retrieved by PlatformIO:

- **`jgromes/RadioLib`**: Non-blocking LoRa transceivers (SX1262 driver).
- **`adafruit/Adafruit BMP280 Library`**: Barometric pressure and temperature sensor driver.
- **`adafruit/Adafruit Unified Sensor`**: Unified sensor abstraction layer.
- **`sparkfun/SparkFun BMI270 Arduino Library`**: IMU sensor API wrapper.
- **`mikalhart/TinyGPSPlus`**: NMEA GPS parsing.
- **`wollewald/INA226_WE`**: Current and power monitoring sensor API.
- **`lewisxhe/XPowersLib`**: Power Management Unit (AXP2101 PMU) API for battery monitoring.

Use this directory only if a hardware driver or reusable component becomes large enough to be maintained as a private library with its own source tree.
