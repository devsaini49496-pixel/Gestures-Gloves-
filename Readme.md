# ESP32 Gesture Glove: Hardware & Pin Mapping

The following tables detail the exact GPIO connections required for the ESP32 Gesture Glove, based on the `GestureGlove.ino` firmware[span_0](start_span)[span_0](end_span). 

## I2C Bus Devices
The MPU6050 IMU and the SSD1306 OLED display share the same I2C pins and can be wired in parallel[span_1](start_span)[span_1](end_span).

| Device | Pin | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **MPU6050** | SDA | GPIO 21 | Default address `0x68`[span_2](start_span)[span_2](end_span) |
| | SCL | GPIO 22 | |
| | VCC | 3.3V | |
| | GND | GND | |
| **OLED (SSD1306)** | SDA | GPIO 21 | Default address `0x3C`[span_3](start_span)[span_3](end_span) |
| | SCL | GPIO 22 | |
| | VCC | 3.3V | |
| | GND | GND | |

## Button Inputs
All buttons utilize the ESP32's internal pull-up resistors (`INPUT_PULLUP`)[span_4](start_span)[span_4](end_span). Wire one leg of each mechanical button to the specified GPIO pin and the other leg to a common Ground (GND).

| Finger | Action | ESP32 GPIO | Firmware Logic |
| :--- | :--- | :--- | :--- |
| **Index** | Jump / Mode / Center | GPIO 27 | Tap = Jump, Hold (0.6s) = Mode Switch, Deep Hold (1.8s) = Re-center pose[span_5](start_span)[span_5](end_span) |
| **Middle** | Shoot | GPIO 26 | Direct mapping (software debounced)[span_6](start_span)[span_6](end_span) |
| **Ring** | Reload | GPIO 32 | Direct mapping (software debounced)[span_7](start_span)[span_7](end_span) |
| **Pinky** | Melee | GPIO 33 | Direct mapping (software debounced)[span_8](start_span)[span_8](end_span) |

## Haptics & Power Sensing

| Component | ESP32 GPIO | Wiring Notes |
| :--- | :--- | :--- |
| **Vibration Motor** | GPIO 25 | Cannot be driven directly by the GPIO pin. Must be wired through an NPN transistor (e.g., 2N2222) with a base resistor[span_9](start_span)[span_9](end_span). |
| **Battery Sense** | GPIO 34 | Optional feature. Requires a 100k/100k voltage divider to step down the battery voltage for safe analog reading[span_10](start_span)[span_10](end_span). |

## Assembly Notes
* **Software Debouncing:** Mechanical switch bounce is handled via a 20ms debounce window in the firmware, so external hardware debounce capacitors are not required[span_11](start_span)[span_11](end_span).
* **DMP Calibration:** The MPU6050 requires unique gyro and accelerometer offsets. You must run the `MPU6050_6Axis_MotionApps20` DMP6 example sketch to find these offsets for your specific chip before finalized use[span_12](start_span)[span_12](end_span).
* 
