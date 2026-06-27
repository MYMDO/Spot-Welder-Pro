# AGENTS.md

## Project

Arduino spot welder controller firmware (v4.0.3) for ATmega328P (Nano/Uno). Event-driven state machine with OLED UI, INA226 power monitoring, EEPROM persistence, and dual-pulse welding.

## Build

Arduino IDE only — no CLI build system. Open `Spot-Welder-Pro.ino` (IDE auto-includes all `.ino` tabs).

**Board:** Arduino Nano (ATmega328P), standard USB bootloader.

**Dependencies** (install via Library Manager):
- `Adafruit SSD1306`
- `INA226` by Rob Tillaart

**Build flags** in `Spot-Welder-Pro.h` (lines 21–25):
```cpp
#define _DEVELOPMENT_   // Serial debug at 115200 baud
// #define _BOOTSYS_    // Force boot into system menu
// #define _TESTING_    // Ignore low-battery alarm
#define _LANG_EN_       // Language: EN/DE/FR/ES/IT
```

## File structure

Alphabetical naming controls Arduino IDE tab order:
- `Spot-Welder-Pro.ino` — entry point: `setup()`, `loop()`, ISR, globals, pin/init
- `Spot-Welder-Pro.h` — all `#define` constants, pin mapping, structs, prototypes, language strings
- `a_state_machine.ino` — FSM, event handlers, `sendWeldPulse()`, sensor reading
- `b_display.ino` — all OLED display/draw routines
- `c_eeprom_sound.ino` — EEPROM load/save/reset, buzzer, `valStr()` formatter, `reboot()`

## Architecture

- **14 states**, **12 events** routed through `mEvent` volatile
- All `mEvent` access via atomic helpers: `atomicReadEvent()`, `atomicReadAndClearEvent()`, `atomicSetEvent()` — use these, never touch `mEvent` directly
- `mEvent` is set by: ISR (encoder), `checkForBtnEvent()`, `checkForLowVoltageEvent()`, `checkTemp()`, `checkForSleepEvent()`
- `stateMachine()` called every `loop()` iteration
- Watchdog (250ms) enabled at end of `setup()`, reset every `loop()` and in long-running pulse sequences

## Key gotchas

- **`sendWeldPulse()`** (`a_state_machine.ino:431`) uses busy-wait `micros()` for pulse timing, not `delay()`. WDT is reset during the loop.
- **`batteryVoltage`** is updated in `checkForLowVoltageEvent()` every 2s and also read by `INA.getBusVoltage_mV()` during pulse sequences. Display routines only read the cached global.
- **EEPROM** auto-saves every 30s via `updateEEPROM()` with CRC-16 (MODBUS) checksum. Unique ID `0x18fae9c8` at address 0 validates EEPROM.
- **I2C clock** set to 800kHz (`Wire.setClock(800000)`) before weld pulse for fast INA226 reads.
- **Voltage units** vary: `batteryVoltage` is in mV, `pData.batteryAlarm` is in units of 100mV (e.g., 75 = 7500mV). Don't mix them up.
- **Strings** are stored in PROGMEM (`FPSTR()` macro). Use `PSTR()` for `sprintf_P` format strings.

## Known issues

`implementation_plan.md` documents 15 issues (4 critical, 4 significant, 7 quality). Notable: blocking `delay()` during pulse is now replaced with `micros()` busy-wait (fixed), WDT is now enabled (fixed), atomic access helpers exist (fixed). Review the plan before adding new features.
