# ⚡ Spot Welder Pro

[![License: GPL-3.0](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20%28ATmega328P%29-00979D?logo=arduino)](https://www.arduino.cc/)
[![Version](https://img.shields.io/badge/Firmware-v4.0.3-green)](Spot-Welder-Pro.h)
[![Build](https://img.shields.io/badge/Recommended%20Build-Fix-orange)](Spot-Welder-Pro-Fix.ino)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-informational)](https://isocpp.org/)

> 🇺🇦 Arduino-based spot welder controller for battery pack welding — featuring an OLED display, INA226 power monitoring, dual-pulse mode, live weld-resistance measurement, foot switch support, and persistent EEPROM settings.

---

## Table of Contents

- [Features](#features)
- [Firmware Variants](#firmware-variants)
- [Hardware Requirements](#hardware-requirements)
- [Pin Mapping](#pin-mapping)
- [Firmware Architecture](#firmware-architecture)
- [Library Dependencies](#library-dependencies)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Operating Modes](#operating-modes)
- [Menu System](#menu-system)
- [Default Parameters](#default-parameters)
- [Fault Handling & Safety Overrides](#fault-handling--safety-overrides)
- [Project Structure](#project-structure)
- [Safety Notes](#safety-notes)
- [Changelog](#changelog)
- [Contributing](#contributing)
- [License](#license)

---

## Features

- **Dual-pulse welding** — configurable short pre-pulse + main pulse for better weld quality
- **Auto & Manual modes** — automatic pulse via touch sensor or manual trigger via foot switch / encoder button
- **Real-time monitoring** — battery voltage and weld current via INA226 over I2C
- **Live weld-resistance measurement** — asynchronous, non-blocking voltage-sag sampling calculates the welding-circuit resistance (mΩ) on every pulse, shown on the main screen
- **Voltage compensation** — pulse duration automatically adjusted for battery voltage variation
- **OLED UI** — 128×64 SSD1306 display with menu navigation via rotary encoder
- **Protection system** — low/high battery voltage alarms, over-temperature alarm (NTC), NTC/INA226 fault detection, watchdog timer (250 ms)
- **User-overridable alarms** — battery voltage and NTC faults can be bypassed from the encoder button when a sensor is missing or a threshold needs to be overridden, and auto-reset once conditions are back to normal
- **Electrode contact check** — manual (foot switch) trigger is blocked with an on-screen warning if the electrodes aren't making contact
- **EEPROM persistence** — settings auto-saved every 30 s (CRC-16 integrity check), plus an instant force-save when leaving the settings menu
- **Weld counter** — cumulative weld count stored in EEPROM
- **Standby mode** — auto standby after configurable inactivity timeout
- **Sound feedback** — piezo buzzer on weld events and alarms
- **Screen invert** — 0°/180° OLED rotation option
- **Debug output** — Serial diagnostic mode (`_DEVELOPMENT_` build flag, 115200 baud)
- **Flash-optimized** — custom lightweight number formatting removes the `sprintf` dependency, freeing 2+ KB of program space

---

## Firmware Variants

The repository ships two `.ino` entry points that share the same headers, state machine, display and EEPROM code:

| File | Status | Description |
|---|---|---|
| **`Spot-Welder-Pro-Fix.ino`** | ✅ Recommended | Current build. Includes the full fix set described in [Changelog](#changelog): non-blocking current/voltage sampling, resistance measurement, NTC/INA226/voltage fault handling with bypass, contact check, instant EEPROM save, flash optimization. |
| `Spot-Welder-Pro.ino` | ⚠️ Legacy | Original entry point kept for reference. Lacks the fixes above and will reboot-loop if the INA226 is not detected. |

> **Important:** the Arduino IDE compiles every `.ino` file in the sketch folder as a single sketch. Having both entry points present at once causes a **duplicate `setup()`/`loop()`** compile error. Before building, either delete/move `Spot-Welder-Pro.ino` out of the folder, or rename `Spot-Welder-Pro-Fix.ino` to `Spot-Welder-Pro.ino` — keep only one.

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
| Hall sensor (weld current) | **A0** | SS49E — sampled asynchronously mid-pulse |
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

All access to the shared `mEvent` volatile is wrapped in atomic helpers (`atomicReadEvent()`, `atomicReadAndClearEvent()`, `atomicSetEvent()`) — never touched directly.

During `sendWeldPulse()`, the current ADC conversion (A0) is triggered asynchronously at the pulse midpoint (`ADCSRA |= (1 << ADSC)`) and read back only after the pulse ends, so measurement never extends the pulse duration. INA226 bus-voltage sag is sampled immediately before and after the pulse (with averaging temporarily forced to 1 sample) to compute the welding-circuit resistance.

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

### 2. Pick one firmware entry point

Keep only `Spot-Welder-Pro-Fix.ino` (recommended) in the folder — remove or rename `Spot-Welder-Pro.ino`, otherwise the sketch will fail to compile (see [Firmware Variants](#firmware-variants)).

### 3. Open in Arduino IDE

Open the remaining `.ino` file — the IDE will automatically include all other `.ino` tab files (`a_state_machine.ino`, `b_display.ino`, `c_eeprom_sound.ino`) and `Spot-Welder-Pro.h`.

### 4. Install dependencies

Use **Sketch → Include Library → Manage Libraries** and install `Adafruit SSD1306` and `INA226`.

### 5. Configure build flags (optional)

Edit the top of `Spot-Welder-Pro.h` before flashing:

```cpp
#define _DEVELOPMENT_    // Enable Serial debug output (115200 baud)
// #define _BOOTSYS_     // Force boot into system menu (for testing)
// #define _TESTING_     // Ignore low-battery alarm
```

### 6. Select board and flash

- **Board:** Arduino Nano (ATmega328P) or compatible
- **Programmer:** USB (standard bootloader)
- Upload via **Sketch → Upload**

### 7. First boot

On first boot with blank EEPROM, the firmware loads defaults automatically. Hold the encoder button during power-on to enter the system menu. If the INA226 isn't detected, the display shows `SENSOR ERROR` instead of reboot-looping.

---

## Configuration

All parameters are adjustable at runtime via the on-device menu and persisted to EEPROM. Key compile-time limits defined in `Spot-Welder-Pro.h`:

| Define | Default | Description |
|--------|---------|-------------|
| `_SERIAL_BAUD_` | `115200` | Serial debug baud rate |
| `INA_MAX_CURRENT_A` | `8` | INA226 full-scale current (A) |
| `INA_SHUNT_RESISTANCE_OHM` | `0.01` | Shunt resistor value (Ω) |
| `INA_AVERAGING_MODE` | `4` | INA226 averaging samples (restored after each pulse) |
| `STANDBY_TIME_OUT` | `640000` ms | Idle-to-standby timeout |
| `EEPROM_UPDATE_T` | `30000` ms | EEPROM background auto-save interval |
| `WP_RETRIGGER_DELAY` | `50` (×10 ms) | Min delay between welds |
| `FS_TRIGGER_DELAY` | `200` ms | Foot switch debounce delay |

---

## Operating Modes

### Auto-pulse mode
Weld triggered automatically when the auto-pulse sensor (D3) detects electrode contact. After the configurable delay (`autoPulseDelay`), the pulse fires without manual input. On entering the main screen, if the sensor is already closed, auto-trigger is suppressed until contacts are released first — this prevents accidental double-firing.

### Manual mode
Weld triggered by pressing the foot switch (D7) or the encoder button. Before firing, the auto-pulse sensor state is checked; if the electrodes aren't touching the workpiece, the pulse is blocked, a warning beep sounds, and `NO CONTACT` is shown on screen.

### Dual-pulse sequence
When `shortPulseTime > 0`, each weld fires a short pre-pulse (% of main pulse time) followed by the full main pulse. This cleans the nickel strip surface and improves weld penetration.

### Weld resistance readout
After every pulse, the welding-circuit resistance (mΩ) — derived from the voltage sag under load and peak current — is shown on the main screen beneath the weld counter.

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
├── [Exit]             — Return to main screen (settings saved instantly)
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

## Fault Handling & Safety Overrides

| Condition | Display | Recovery |
|---|---|---|
| INA226 not detected at boot | `SENSOR ERROR` | Firmware continues booting instead of reboot-looping; reconnect sensor and reboot |
| NTC open (`ADC > 900`) or shorted (`ADC < 15`) | `NTC ERROR / Sensor open/short` | Press encoder button to bypass (high-temp alarm at 65 °C remains enforced); auto-clears when the sensor is reconnected |
| Battery voltage outside low/high alarm thresholds | `BATT LOW` / `BATT HIGH` | Press encoder button to temporarily bypass and resume welding; bypass auto-resets once voltage returns to range |
| Electrodes not touching workpiece on manual trigger | `NO CONTACT / Press electrodes` | Make contact with the electrodes and retry |

---

## Project Structure

```
Spot-Welder-Pro/
├── Spot-Welder-Pro-Fix.ino  # Recommended entry point: setup(), loop(), ISR (with fixes)
├── Spot-Welder-Pro.ino      # Legacy entry point — kept for reference only
├── Spot-Welder-Pro.h        # Config, pin defines, macros, structs
├── a_state_machine.ino      # Full event-driven state machine, sendWeldPulse(), fault logic
├── b_display.ino            # All OLED display routines
├── c_eeprom_sound.ino       # EEPROM load/save, valStr() formatter, buzzer
├── printf.h                 # printf() support for AVR
├── implementation_plan.md   # Technical analysis & improvement roadmap
├── walkthrough.md           # Detailed changelog of the Fix build (UA)
├── AGENTS.md                # Codebase notes for AI coding agents
├── .github/FUNDING.yml      # Sponsorship links
└── LICENSE                  # GPL-3.0
```

---

## Safety Notes

> ⚠️ **This device controls high-current welding pulses. Incorrect wiring or software misconfiguration can cause fire, battery damage, or personal injury. Build and operate at your own risk.**

- Always verify MOSFET/SSR wiring before first power-on.
- The foot switch is checked at startup — if pressed during boot, the device halts and requires a clean restart.
- The watchdog timer (250 ms) will force a hardware reset if the main loop stalls.
- Battery voltage is measured every 2 s; audible and visual alarms activate outside the configured thresholds and can be bypassed manually — only bypass if you understand the risk.
- Temperature is sampled every 10 s; the welder disables pulse output above 65 °C (default) until cooling, and this alarm cannot be bypassed even when the NTC fault override is active.
- Remove power before modifying hardware connections.

---

## Changelog

### Fix build (current)

- **Non-blocking current/voltage sampling** — ADC and INA226 reads no longer extend pulse duration.
- **Weld-circuit resistance measurement** — computes and displays mΩ per weld using pre/post-pulse voltage sag; sensor averaging is temporarily disabled during capture to avoid the pulse being masked by 128-sample averaging.
- **NTC fault detection with bypass** — open/short thermistor shows a dedicated warning instead of the normal temperature screen, overridable via the encoder button, with automatic recovery.
- **Battery voltage alarm bypass** — low/high alarms can be overridden from the encoder button and auto-reset when voltage returns to range.
- **No more INA226 reboot loop** — a missing/failed sensor now boots to a `SENSOR ERROR` screen instead of retrying `reboot()` indefinitely.
- **Corrected compensation default** — nominal voltage default aligned to 8.5 V so pulse-time compensation scales in both directions instead of clamping low.
- **Fixed sound menu display bug** — the sound toggle screen was reading the auto-pulse flag instead of the sound flag.
- **Instant EEPROM save** — settings are force-written on menu exit instead of waiting for the 30 s background timer.
- **Auto-pulse re-trigger guard** — prevents a false weld if the auto-pulse contact is already closed when the main screen is entered.
- **Electrode contact check on manual trigger** — blocks the pulse and warns if the electrodes aren't in contact.
- **Boot menu entry fix** — button state read is latched at the very start of `setup()` so encoder interrupts during boot delays/beeps can no longer overwrite the boot-menu request.
- **Flash footprint reduction** — replaced `sprintf_P`-based value formatting with a lightweight custom formatter, freeing 2+ KB of program memory.

See [`walkthrough.md`](walkthrough.md) for the full technical write-up (Ukrainian) and [`implementation_plan.md`](implementation_plan.md) for the underlying issue analysis.

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
