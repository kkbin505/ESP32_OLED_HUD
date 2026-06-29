# ESP32 ILI9341 MAVLink HUD

Real-time flight HUD for fixed-wing / multirotor aircraft. Receives MAVLink telemetry over UART, renders an attitude indicator on a 2.8" IPS TFT display, and supports touch-based arm/disarm via a capacitive FT6206 overlay.

## Hardware

| Signal | GPIO |
|--------|------|
| TFT CS | 10 |
| TFT DC | 46 |
| TFT MOSI | 11 |
| TFT SCLK | 12 |
| TFT MISO | 13 |
| Backlight | 45 |
| MAVLink RX (UART0) | 43 |
| MAVLink TX (UART0) | 44 |
| Touch SDA (I2C) | 16 |
| Touch SCL (I2C) | 15 |
| Touch RST | 18 |
| Touch INT | 17 |

- **MCU**: ESP32-S3
- **Display**: 2.8" IPS 240×320 ILI9341V, 4-line SPI @ 40 MHz
- **Touch**: FT6206 capacitive controller, I2C
- **MAVLink**: 115200 baud, UART0

## Display Layout (portrait 240×320)

```
┌────────────────────────┐
│   Heading tape  [333]  │  25 px
├────────────────────────┤
│                        │
│   Artificial horizon   │  210 px
│   + pitch ladder ±20°  │
│   + aircraft symbol    │
│                        │
├────────────────────────┤
│  SPD Km/h │  ALT  m    │  45 px  (border: red=ARMED, green=DISARMED)
├────────────────────────┤
│  < slide to ARM >      │  40 px  (touch slider)
└────────────────────────┘
```

## Features

**Artificial horizon** — per-scanline sky/ground fill; geometrically correct at any roll angle. Pitch ladder ±20° every 5°. Fixed yellow aircraft symbol at screen centre.

**Heading tape** — scrolling tape with degree marks every 5°, labelled every 10°. Current heading highlighted in a white box.

**ARM / DISARM slider** — swipe right from the left edge to arm; swipe left from the right edge to disarm. Pending state flashes orange with "ARMING…" / "DISARMING…" text; reverts after 4 s timeout if the FC does not confirm. Confirmation comes from the HEARTBEAT `base_mode` flag.

**Info panel** — speed (km/h, yellow) and altitude (m, green). Top border is red when armed, green when disarmed. ARM/DISARMED badge in the top-right corner. Flight mode name shown in the slider bar when armed (ArduCopter and ArduPlane mode tables built in).

**GCS keepalive** — sends a GCS heartbeat every 1 s so ArduPilot starts telemetry streams. Re-requests ATTITUDE (10 Hz) and VFR_HUD (5 Hz) streams every 5 s to survive FC reboots.

**Signal lost** — full red screen with "SIGNAL LOST" text if no MAVLink message is received for 1 s.

**Demo mode** — uncomment `#define DEMO_MODE` in `src/main.cpp` to drive the HUD with synthetic sine-wave data (no FC required).

## Rendering

All drawing is done into a full-screen `GFXcanvas16` (153 KB heap), then pushed to the display in a single `writePixels` burst — zero flicker.

## MAVLink messages used

| Message | Fields used |
|---------|-------------|
| `HEARTBEAT` | `base_mode` (armed flag), `custom_mode`, `type` |
| `ATTITUDE` | `roll`, `pitch`, `yaw` |
| `VFR_HUD` | `groundspeed`, `alt` |

## Build

```
pio run            # compile
pio run -t upload  # flash
```

Dependencies managed by PlatformIO:
- `adafruit/Adafruit ILI9341`
- `adafruit/Adafruit GFX Library`
- `adafruit/Adafruit FT6206 Library`
- MAVLink C library (via `-I` build flag pointing to the common header set)


Support:
https://www.lcdwiki.com/2.8inch_ESP32-S3_Display

todo:
More parameter support
esp now wireless telemetry support
Tiny ground station with out PC