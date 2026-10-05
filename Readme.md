# ESP32 Gesture Glove: Hardware & Pin Mapping

The following tables detail the exact GPIO connections required for the ESP32 Gesture Glove, based on the `GestureGlove.ino` firmware.

## I2C Bus Devices
The MPU6050 IMU and the SSD1306 OLED display share the same I2C pins and can be wired in parallel.

| Device | Pin | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **MPU6050** | SDA | GPIO 21 | Default address `0x68` |
| | SCL | GPIO 22 | |
| | VCC | 3.3V | |
| | GND | GND | |
| **OLED (SSD1306)** | SDA | GPIO 21 | Default address `0x3C` |
| | SCL | GPIO 22 | |
| | VCC | 3.3V | |
| | GND | GND | |

## Button Inputs
All buttons utilize the ESP32's internal pull-up resistors (`INPUT_PULLUP`). Wire one leg of each mechanical button to the specified GPIO pin and the other leg to a common Ground (GND).

| Finger | Action | ESP32 GPIO | Firmware Logic |
| :--- | :--- | :--- | :--- |
| **Index** | Jump / Mode / Center | GPIO 27 | Tap = Jump, Hold (0.6s) = Mode Switch, Deep Hold (1.8s) = Re-center pose |
| **Middle** | Shoot | GPIO 26 | Direct mapping (software debounced) |
| **Ring** | Reload | GPIO 32 | Direct mapping (software debounced) |
| **Pinky** | Melee | GPIO 33 | Direct mapping (software debounced) |

## Haptics & Power Sensing

| Component | ESP32 GPIO | Wiring Notes |
| :--- | :--- | :--- |
| **Vibration Motor** | GPIO 25 | Cannot be driven directly by the GPIO pin. Must be wired through an NPN transistor (e.g., 2N2222) with a base resistor. |
| **Battery Sense** | GPIO 34 | Optional feature. Requires a 100k/100k voltage divider to step down the battery voltage for safe analog reading. |

## Assembly Notes
* **Software Debouncing:** Mechanical switch bounce is handled via a 20ms debounce window in the firmware, so external hardware debounce capacitors are not required.
* **DMP Calibration:** The MPU6050 requires unique gyro and accelerometer offsets. You must run the `MPU6050_6Axis_MotionApps20` DMP6 example sketch to find these offsets for your specific chip before finalized use.
* 
