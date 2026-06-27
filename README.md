# ⚡ Spot Welder Pro

[![License: GPL-3.0](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20%28ATmega328P%29-00979D?logo=arduino)](https://www.arduino.cc/)
[![Version](https://img.shields.io/badge/Firmware-v4.0.3-green)](Spot-Welder-Pro.h)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-informational)](https://isocpp.org/)

> 🇺🇦 Arduino-based spot welder controller for battery pack welding — featuring an OLED display, INA226 power monitoring, dual-pulse mode, foot switch support, and persistent EEPROM settings.

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Mapping](#pin-mapping)
- [Firmware Architecture](#firmware-architecture)
- [Library Dependencies](#library-dependencies)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Operating Modes](#operating-modes)
- [Menu System](#menu-system)
- [Default Parameters](#default-parameters)
- [Project Structure](#project-structure)
- [Safety Notes](#safety-notes)
- [Contributing](#contributing)
- [License](#license)

---

## Features

- **Dual-pulse welding** — configurable short pre-pulse + main pulse for better weld quality
- **Auto & Manual modes** — automatic pulse via touch sensor or manual trigger via foot switch / encoder button
- **Real-time monitoring** — battery voltage and weld current via INA226 over I2C
- **Voltage compensation** — pulse duration automatically adjusted for battery voltage variation
- **OLED UI** — 128×64 SSD1306 display with menu navigation via rotary encoder
- **Protection system** — low/high battery voltage alarms, over-temperature alarm (NTC), watchdog timer (250 ms)
- **EEPROM persistence** — all settings saved automatically every 30 s with CRC-16 integrity check
- **Weld counter** — cumulative weld count stored in EEPROM
- **Standby mode** — auto standby after configurable inactivity timeout
- **Sound feedback** — piezo buzzer on weld events and alarms
- **Screen invert** — 0°/180° OLED rotation option
- **Multi-language skeleton** — EN / DE / FR / ES / IT (English active by default)
- **Debug output** — Serial diagnostic mode (`_DEVELOPMENT_` build flag, 115200 baud)

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| MCU | Arduino (ATmega328P), e.g. Arduino Nano / Uno |
| OLED display | 128×64 SSD1306, I2C |
| Current/voltage sensor | INA226, I2C (addr `0x40`) |
| Rotary encoder | Incremental, with push button |
| Weld pulse switch | MOSFET gate or SSR driven from digital pin 5 |
| Foot switch | Normally-open, pull-up |
| Auto-pulse sensor | Touch / proximity, active-high on pin 3 |
| Buzzer | Passive piezo |
| Temperature sensor | NTC thermistor on A3 |
| Current sensor (weld) | Hall effect sensor SS49E on A0 |
| Shunt resistor | 0.01 Ω / INA226 path |
| Power source | LiPo / Li-ion pack (supported range: 3.5 V – 15 V) |

---

## Pin Mapping

| Signal | Arduino Pin | Notes |
|---|---|---|
| Encoder CLK | **D2** | INT0 — hardware interrupt |
| Encoder DT | **D8** | Phase detection |
| Encoder SW | **D6** | Pull-up, active-low |
| Foot switch | **D7** | Pull-up, active-low |
| Auto-pulse sensor | **D3** | Active-high |
| Weld pulse output | **D5** | To MOSFET gate / SSR |
| Buzzer | **A1** | Passive piezo |
| Temperature (NTC) | **A3** | Voltage divider to GND |
| Hall sensor (weld current) | **A0** | SS49E — read during pulse |
| OLED + INA226 (I2C) | **A4 / A5** | SDA / SCL |

**I2C addresses:**
- SSD1306 OLED: `0x3C`
- INA226: `0x40`

---

## Firmware Architecture

The firmware is built around an **event-driven state machine**:

```
ISR (INT0) ──► mEvent (volatile) ──► stateMachine()
Timers      ──► mEvent               │
Button poll ──► mEvent               ▼
                                 14 States × 12 Events
                                     │
                          ┌──────────┴──────────┐
                          │                     │
                     Display layer         EEPROM layer
                     (b_display.ino)  (c_eeprom_sound.ino)
```

**States (14):** `STANDBY`, `MAIN_SCREEN`, `MAIN_SCREEN_CNT`, `MENU_SCREEN`, `SUB_MENU_1/2`, `BATTERY_HIGH`, `TEMP_HIGH`, `SYSTEM_SCREEN`, `SYSTEM_MENU`, `REBOOT_MENU`, `MAXWELD_SCREEN`, `INVERT_SCREEN`, and more.

**Events (12):** `NONE`, `BTNDN`, `BTNUP`, `ENCUP`, `ENCDN`, `BOOTDN`, `STBY_TIMEOUT`, `BATT_LV`, `BATT_HV`, `TEMP_HIGH`, `EEUPD`.

All access to the shared `mEvent` volatile is wrapped in `cli()/sei()` atomic helpers.

---

## Library Dependencies

Install via **Arduino Library Manager** or manually:

| Library | Purpose | Install name |
|---------|---------|---|
| [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) | OLED display driver | `Adafruit SSD1306` |
| [INA226](https://github.com/RobTillaart/INA226) | Voltage/current sensing | `INA226` by Rob Tillaart |
| EEPROM | Settings persistence | Built-in (AVR) |
| Wire | I2C bus | Built-in (Arduino) |
| avr/wdt.h | Watchdog timer | Built-in (AVR) |

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/MYMDO/Spot-Welder-Pro.git
```

### 2. Open in Arduino IDE

Open `Spot-Welder-Pro.ino` — the IDE will automatically include all `.ino` tab files.

### 3. Install dependencies

Use **Sketch → Include Library → Manage Libraries** and install `Adafruit SSD1306` and `INA226`.

### 4. Configure build flags (optional)

Edit the top of `Spot-Welder-Pro.h` before flashing:

```cpp
#define _DEVELOPMENT_    // Enable Serial debug output (115200 baud)
// #define _BOOTSYS_     // Force boot into system menu (for testing)
// #define _TESTING_     // Ignore low-battery alarm
#define _LANG_EN_        // Language: _LANG_EN_ / _LANG_DE_ / _LANG_FR_ / _LANG_ES_ / _LANG_IT_
```

### 5. Select board and flash

- **Board:** Arduino Nano (ATmega328P) or compatible
- **Programmer:** USB (standard bootloader)
- Upload via **Sketch → Upload**

### 6. First boot

On first boot with blank EEPROM, the firmware loads defaults automatically. Hold the encoder button during power-on to enter the system menu.

---

## Configuration

All parameters are adjustable at runtime via the on-device menu and persisted to EEPROM. Key compile-time limits defined in `Spot-Welder-Pro.h`:

| Define | Default | Description |
|--------|---------|-------------|
| `_SERIAL_BAUD_` | `115200` | Serial debug baud rate |
| `INA_MAX_CURRENT_A` | `8` | INA226 full-scale current (A) |
| `INA_SHUNT_RESISTANCE_OHM` | `0.01` | Shunt resistor value (Ω) |
| `INA_AVERAGING_MODE` | `4` | INA226 averaging samples |
| `STANDBY_TIME_OUT` | `640000` ms | Idle-to-standby timeout |
| `EEPROM_UPDATE_T` | `30000` ms | EEPROM auto-save interval |
| `WP_RETRIGGER_DELAY` | `50` (×10 ms) | Min delay between welds |
| `FS_TRIGGER_DELAY` | `200` ms | Foot switch debounce delay |

---

## Operating Modes

### Auto-pulse mode
Weld triggered automatically when the auto-pulse sensor (D3) detects electrode contact. After the configurable delay (`autoPulseDelay`), the pulse fires without manual input.

### Manual mode
Weld triggered by pressing the foot switch (D7) or the encoder button. Pulse starts immediately on activation.

### Dual-pulse sequence
When `shortPulseTime > 0`, each weld fires a short pre-pulse (% of main pulse time) followed by the full main pulse. This cleans the nickel strip surface and improves weld penetration.

---

## Menu System

Navigate with the rotary encoder; press to confirm.

```
Main Screen
│
├── [Pulse Set]        — Weld pulse duration (1–100 ms)
├── [Shrt Pulse]       — Short pre-pulse (0–100% of pulse time)
├── [Mode]             — Auto / Manual
├── [Delay]            — Auto-pulse delay
├── [Batt Alarm]       — Low / High battery voltage alarms
├── [Sound]            — Weld sound on/off
├── [Exit]             — Return to main screen
│
└── System Menu (hold button on boot)
    ├── Settings
    │   ├── Max Pulse  — Maximum allowed pulse time
    │   ├── Vnom       — Nominal voltage for compensation
    │   └── Calibrate Battery
    ├── Display
    │   └── Screen Orientation — Normal / Inverted
    └── Boot
        ├── Reboot
        ├── Safe Reset — Reload defaults, keep weld count
        └── Full Reset — Clear all EEPROM
```

---

## Default Parameters

| Parameter | Default | Range |
|-----------|---------|-------|
| Pulse time | 5 ms | 1–100 ms |
| Max pulse time | 100 ms | 1–100 ms |
| Short pulse | 0 % | 0–100 % |
| Auto-pulse delay | 2000 ms | 500–5000 ms |
| Low battery alarm | 7500 mV | 3500–15000 mV |
| High battery alarm | 15000 mV | 3500–15000 mV |
| Nominal voltage | 8500 mV | — |
| High-temp alarm | 65 °C | — |
| Auto-pulse | Enabled | — |
| Weld sound | Enabled | — |
| OLED orientation | Normal | Normal / Inverted |

---

## Project Structure

```
Spot-Welder-Pro/
├── Spot-Welder-Pro.ino      # Entry point: setup(), loop(), ISR
├── Spot-Welder-Pro.h        # Config, pin defines, macros, structs
├── a_state_machine.ino      # Full event-driven state machine
├── b_display.ino            # All OLED display routines
├── c_eeprom_sound.ino       # EEPROM load/save and buzzer
├── printf.h                 # printf() support for AVR
├── implementation_plan.md   # Technical analysis & improvement roadmap
└── LICENSE                  # GPL-3.0
```

---

## Safety Notes

> ⚠️ **This device controls high-current welding pulses. Incorrect wiring or software misconfiguration can cause fire, battery damage, or personal injury. Build and operate at your own risk.**

- Always verify MOSFET/SSR wiring before first power-on.
- The foot switch is checked at startup — if pressed during boot, the device halts and requires a clean restart.
- The watchdog timer (250 ms) will force a hardware reset if the main loop stalls.
- Battery voltage is measured every 2 s; audible and visual alarms activate outside the configured thresholds.
- Temperature is sampled every 10 s; the welder disables pulse output above 65 °C (default) until cooling.
- Remove power before modifying hardware connections.

---

## Contributing

Pull requests and issues are welcome.

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-improvement`
3. Commit your changes: `git commit -m "Add: description"`
4. Push and open a Pull Request

Please respect existing code style and document any new `#define` constants in `Spot-Welder-Pro.h`.

---

## License

This project is licensed under the **GNU General Public License v3.0** — see [LICENSE](LICENSE) for details.

---

*Made in Ukraine 🇺🇦 · [Sponsor](http://coindrop.to/mymdo)*
