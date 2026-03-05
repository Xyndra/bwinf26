# Watchy Rover Controller

Firmware for the [Watchy](https://watchy.sqfmi.com/) smartwatch that controls a rover car over ESP-NOW. Tilt the watch to steer; the e-paper display shows the car's position and detected obstacles as a live map.

## Setup

1. **Install dependencies** (once):
   ```bash
   ./local_setup.sh
   ```

2. **Build, flash, and monitor**:
   ```bash
   ./run.sh
   ```

## Buttons

| Button | Action |
|--------|--------|
| Top-Right | Start / stop driving |
| Top-Left | Reboot watch |

## Architecture

Three FreeRTOS tasks run concurrently:

| Task | Priority | Role |
|------|----------|------|
| `accel_task` | 4 | Reads BMA423 accelerometer, sends `drive_data` to car via ESP-NOW |
| `map_task` | 3 | Redraws car position and obstacles on the e-paper display every second |
| `sensor_task` | 2 | Receives `sensor_data` from car, updates position and obstacle map |

## Files

| File | Description |
|------|-------------|
| `main/main.cpp` | Entry point, task creation, display rendering, ESP-NOW setup |
| `main/sensor.cpp` | Converts car IMU/LiDAR data into display coordinates |
| `main/config.h` | All tunable constants (MACs, thresholds, task periods) |
| `main/interface.h` | Shared packet structs (`sensor_data`, `drive_data`) used by both car and watch |
| `main/helper.h` | Pin definitions and BMA423 accelerometer initialization |
| `patches/` | Compatibility patches for ESP-IDF and library components |
