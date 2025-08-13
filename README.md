# ESP32_OLED_HUD
ESP32 based HUD for RC plane

# ESP32 OLED HUD Display

This project uses an **ESP32** and an **SSD1306 OLED** module to create a simple **Head-Up Display (HUD)**.  
It shows pitch, roll, heading, speed, and altitude, and can receive live data from an external sensor or flight controller over UART.

## Features
- Developed using the **Arduino** framework (compatible with PlatformIO or Arduino IDE)
- Renders graphics using **Adafruit SSD1306** + **Adafruit GFX**
- Draws horizon line, crosshair, heading, speed scale, and altitude scale
- Parses flight data from a serial port
- Works with I²C SSD1306 OLED modules (128x64 resolution)

---

## Hardware Wiring

| ESP32 Pin  | Function     | Connects To |
|------------|-------------|-------------|
| GPIO 12    | I2C SDA     | OLED SDA    |
| GPIO 13    | I2C SCL     | OLED SCL    |
| GPIO 8     | UART1 RX    | External TX |
| GPIO 9     | UART1 TX    | External RX |
| 3V3        | Power       | OLED VCC    |
| GND        | Ground      | OLED GND    |

> **Note:** UART1 is used for receiving flight data. Default baud rate: `115200`.

---


## Build & Run

1. Save the code as a PlatformIO project
2. VScode weill install the dependencies listed
3. Upload the code  
   The OLED will display simulated data or data from the external serial source

---

## Serial Data Format

The external flight controller or sensor should send one line of data (ending with `\n`) in the following format:


- **P**: Pitch angle (°)
- **R**: Roll angle (°)
- **Y**: Yaw / Heading (°)
- **S1**: Servo 1 PWM
- **S2**: Servo 2 PWM
- **Flight**: Flight mode (customizable)

---

## Display Layout

- **Top**: Heading (HDG)
- **Center**: Horizon line and crosshair
- **Left**: Speed scale
- **Right**: Altitude scale
- **Upper middle**: Pitch value

---

## Example Output



---

## Possible Improvements
- Add track/flight path visualization
- Mavlink support

