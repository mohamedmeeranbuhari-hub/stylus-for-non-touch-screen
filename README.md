# Stylus for Non-Touch Screen

An embedded motion-tracking stylus prototype for non-touch screens using an ESP32 and an MPU-6050 (6-DOF IMU).

## Project Overview

This project explores handwriting replication and 3D air-gesture tracking by embedding an IMU sensor into a stylus. The firmware features:
- **Comprehensive 6-Axis Hardware Self-Test**: Validates the electrostatic actuation of MEMS proof masses on both accelerometer and gyroscope against factory trim limits.
- **Die Temperature Sanity Monitoring**: Tracks on-chip temperature to compensate for zero-rate gyro drift ($ZRO$).
- **Direct-Socket Pin Mapping**: Supports direct header socketing into the ESP32 DevKit to eliminate loose jumper wires.
- **On-Demand Serial Diagnostics**: Trigger hardware self-tests over serial without rebooting or tripping strapping pins.
- **Real-Time Telemetry & Plotter Support**: Streams formatted acceleration and angular rate streams compatible with the Arduino IDE Serial Plotter and standalone Python waveform monitors.

---

## Hardware Configuration (Direct Header Socket)

The MPU-6050 module (GY-521) connects directly across a contiguous row on the ESP32 DevKit:

| ESP32 Pin | MPU-6050 Pin | Function |
| :--- | :--- | :--- |
| **VIN / 5V** | **VCC** | Power (feeds onboard 3.3V LDO regulator) |
| **GND** | **GND** | Ground reference |
| **GPIO 13** | **SCL** | I2C Clock (100 kHz) |
| **GPIO 12** | **SDA** | I2C Data |
| **GPIO 14** | **XDA** | MPU Aux I2C Data (High-Z) |
| **GPIO 27** | **XCL** | MPU Aux I2C Clock (High-Z) |
| **GPIO 26** | **AD0** | Driven LOW in software (locks I2C address to `0x68`) |
| **GPIO 25** | **INT** | Data-Ready / Motion Interrupt |

---

## Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) (CLI or VS Code Extension)
- Python 3 with `pyserial` and `matplotlib` (for standalone plotting)

### Build & Upload
```bash
# Compile firmware
pio run

# Flash to ESP32
pio run -t upload

# Open Serial Monitor
pio device monitor -b 115200
```

### On-Demand Diagnostics
In the serial monitor, type **`t`** and press **Enter** to run the hardware self-test diagnostic while the sensor is resting on your desk.

### Live Waveform Plotter
Run the included Python plotter to view real-time rolling waveforms:
```bash
python plotter.py COM7
```
