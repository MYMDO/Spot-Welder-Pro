# Аналіз проекту контактної зварки на ATmega328P

## Огляд проекту

**Arduino Spot Welder Control Firmware v4.0.3** — прошивка для управління пристроєм контактної (точкової) зварки на базі ATmega328P (Arduino). Проект включає:

- **Основний файл**: [Spot-Welder-Pro-Fix.ino](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino) (~1465 рядків)
- **Заголовочний файл**: [Spot-Welder-Pro.h](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro.h) (~322 рядки)
- **Printf підтримка**: [printf.h](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/printf.h) (36 рядків)

### Функціональність
| Компонент | Опис |
|---|---|
| OLED дисплей 128×64 (SSD1306) | Відображення UI, меню, попереджень |
| INA226 | Вимірювання напруги та струму батареї |
| Rotary encoder + кнопка | Навігація по меню, налаштування часу імпульсу |
| Ножний перемикач | Ручний тригер зварювального імпульсу |
| Автоімпульсний сенсор (PIN 3) | Автоматичний тригер при дотику електродів |
| NTC термістор (A3) | Моніторинг температури |
| Датчик Холла SS49E (A0) | Вимірювання струму зварювання |
| П'єзо-зумер (A1) | Звукові сигнали та попередження |
| EEPROM з CRC-16 | Збереження налаштувань |

### Архітектура
State machine з 14 станами, event-driven модель з 12 подіями, неблокуючі попередження напруги, динамічна компенсація часу імпульсу від напруги батареї.

---

## Знайдені проблеми та рекомендації

### 🔴 Критичні проблеми (безпека)

---

#### 1. Блокуючий `delay()` під час зварювального імпульсу

**Файл**: [Spot-Welder-Pro-Fix.ino:761](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L760-L767)

```cpp
weldPulse(Pulse_ON);           // >>> Start Pulse
delay(compensatedPulseTime);   // ⚠️ БЛОКУВАННЯ CPU
// ... читання ADC ...
weldPulse(Pulse_OFF);          // <<< Stop Pulse
```

> [!CAUTION]
> Під час `delay()` CPU повністю заблоковано. Якщо трапиться апаратний збій або застрягне переривання — MOSFET залишиться відкритим, що може призвести до перегріву, пошкодження акумулятора або пожежі.

**Рекомендація**: Використовувати апаратний таймер (Timer1) для точного керування тривалістю імпульсу з обов'язковим відключенням через overflow interrupt. Альтернативно — Watchdog Timer як аварійний захист.

```cpp
// Приклад з Timer1:
void startPulseWithTimer(uint16_t duration_ms) {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12); // CTC, prescaler 256
    OCR1A = (F_CPU / 256 / 1000) * duration_ms;
    TCNT1 = 0;
    TIMSK1 = (1 << OCIE1A);
    weldPulse(Pulse_ON);
}

ISR(TIMER1_COMPA_vect) {
    weldPulse(Pulse_OFF);
    TIMSK1 = 0; // Disable interrupt
}
```

---

#### 2. `analogRead()` під час активного імпульсу

**Файл**: [Spot-Welder-Pro-Fix.ino:764-765](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L764-L765)

```cpp
uint16_t PulseGauss = analogRead(A0);          // ~100µs
uint16_t busVoltageDuring = INA.getBusVoltage_mV(); // I2C — до ~1ms
```

> [!WARNING]
> `analogRead()` займає ~112µs, а I2C читання INA226 — до 1ms. Під час імпульсу тривалістю 1-5мс це додає суттєву непередбачувану затримку. Більш того, ця затримка відбувається **після** `delay(compensatedPulseTime)`, тобто реальний час імпульсу = `compensatedPulseTime + ADC_time + I2C_time`.

**Рекомендація**: Виконувати вимірювання **паралельно** з імпульсом, використовуючи асинхронний ADC:

```cpp
weldPulse(Pulse_ON);
unsigned long startMicros = micros();

// Запускаємо ADC асинхронно
ADMUX = (ADMUX & 0xF0) | (A0 & 0x0F);
ADCSRA |= (1 << ADSC); // Start conversion

// Чекаємо основний час імпульсу
while (micros() - startMicros < (uint32_t)compensatedPulseTime * 1000) {}

weldPulse(Pulse_OFF); // Спочатку вимикаємо!

// Потім читаємо результати
while (ADCSRA & (1 << ADSC)) {}
uint16_t PulseGauss = ADC;
uint16_t busVoltageDuring = INA.getBusVoltage_mV();
```

---

#### 3. Відсутній апаратний watchdog під час нормальної роботи

**Файл**: [Spot-Welder-Pro-Fix.ino:72-76](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L72-L76)

```cpp
void reset_mcusr(void) {
    MCUSR = 0;
    wdt_disable(); // ⚠️ WDT вимкнено і більше ніде не вмикається
}
```

> [!CAUTION]
> Watchdog Timer відключається при завантаженні і більше ніколи не активується. Якщо програма зависне під час зварювального імпульсу, MOSFET залишиться відкритим назавжди.

**Рекомендація**: Увімкнути WDT після ініціалізації з періодичним скиданням в `loop()`:

```cpp
void setup() {
    // ... ініціалізація ...
    wdt_enable(WDTO_250MS); // 250мс timeout
}

void loop() {
    wdt_reset(); // Скидаємо WDT кожен цикл
    stateMachine();
    updateEEPROM();
}
```

---

#### 4. Доступ до `volatile` змінної `mEvent` без атомарності

**Файл**: [Spot-Welder-Pro-Fix.ino:67](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L67) та ISR на [рядку 943](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L943-L955)

```cpp
volatile uint8_t mEvent;  // Змінюється в ISR
// ... і читається/записується в main loop без захисту:
if (mEvent == EV_BTNDN) { ... }
mEvent = EV_NONE;
```

> [!WARNING]
> `uint8_t` на AVR читається атомарно (1 байт), тому **на практиці** гонка даних малоймовірна. Але це поганий паттерн: між `if (mEvent == ...)` і `mEvent = EV_NONE` ISR може записати нову подію, і вона буде втрачена.

**Рекомендація**: Використовувати `cli()/sei()` навколо критичних секцій:

```cpp
uint8_t localEvent;
cli();
localEvent = mEvent;
mEvent = EV_NONE;
sei();

switch (localEvent) { ... }
```

---

### 🟠 Суттєві проблеми (надійність та коректність)

---

#### 5. Компенсація часу імпульсу за напругою — некоректна формула

**Файл**: [Spot-Welder-Pro-Fix.ino:727](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L723-L750)

```cpp
float compensationFactor = (float)NOMINAL_VOLTAGE_FOR_COMPENSATION / (float)batteryVoltage;
```

> [!IMPORTANT]
> `NOMINAL_VOLTAGE_FOR_COMPENSATION = 4200` (мВ), а `batteryVoltage` зберігається як значення з INA226 в мВ (напр. 5400мВ). Коефіцієнт = 4200/5400 ≈ 0.78 — це **зменшить** час при вищій напрузі, що правильно за ідеєю (потужність ∝ V²), але:
> 1. Еталонна напруга 4.2В виглядає як напруга LiPo, хоча система працює на 5.4В — невідповідність.
> 2. Для компенсації потужності потрібне квадратичне співвідношення: `factor = (Vnom/Vact)²`, оскільки `P = V²/R`.
> 3. Використання `float` на AVR — дорога операція (~70 тактів на множення).

**Рекомендація**: Виправити формулу та уникнути float:

```cpp
// Компенсація потужності (P = V²/R), цілочисельна арифметика
// compensatedTime = baseTime * (Vnom² / Vact²)
uint32_t vnom2 = (uint32_t)NOMINAL_VOLTAGE * NOMINAL_VOLTAGE;
uint32_t vact2 = (uint32_t)batteryVoltage * batteryVoltage;
uint16_t compensatedTime = (uint16_t)((uint32_t)pData.pulseTime * vnom2 / vact2);
```

---

#### 6. Розрахунок струму через датчик Холла — незрозумілий та потенційно помилковий

**Файл**: [Spot-Welder-Pro-Fix.ino:771-774](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L771-L774)

```cpp
pData.PulseAmps = map(PulseGauss, NominalGauss, 1023, 0, (1023 - NominalGauss) * 10);
```

> [!WARNING]
> - Формула `map()` лінійно масштабує з діапазону `[NominalGauss..1023]` в `[0..(1023-NominalGauss)*10]`, що фактично множить різницю на 10. Але це не відповідає закоментованому калібруванню `10A/поділка`.
> - `NominalGauss` читається з `analogRead(A0)` за 9мс **до** імпульсу, але може змінитися через шуми.
> - Немає усереднення ADC значень.
> - Закоментований варіант `map(PulseGauss, NominalGauss, 900, 0, 3880)` виглядає більш каліброваним.

**Рекомендація**: Оформити калібрувальні параметри як `#define` константи та додати усереднення ADC:

```cpp
#define HALL_ADC_MAX 900        // ADC відповідний максимальному струму
#define HALL_CURRENT_MAX 3880   // Максимальний струм × 10

uint16_t readADCAvg(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) sum += analogRead(pin);
    return sum / samples;
}
```

---

#### 7. `batteryVoltage` оновлюється в двох місцях некоординовано

**Файли**:
- [checkForLowVoltageEvent():809](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L809): `batteryVoltage = INA.getBusVoltage_mV();`
- [drawStatusLine():1056](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L1056): `batteryVoltage = (INA.getBusVoltage_mV());`

> [!WARNING]
> `drawStatusLine()` неочевидно оновлює глобальну `batteryVoltage` як побічний ефект малювання UI. Це порушує принцип поділу відповідальності і може призвести до неочікуваної поведінки: кожне перемалювання екрана запускає I2C транзакцію (~1мс), що може мигтіти дисплей.

**Рекомендація**: Винести оновлення `batteryVoltage` в єдину точку (наприклад, `checkForLowVoltageEvent()`), а в `drawStatusLine()` лише відображати вже зчитане значення.

---

#### 8. Повна очистка EEPROM при скиданні — зайвий знос

**Файл**: [Spot-Welder-Pro-Fix.ino:1309-1311](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L1309-L1311)

```cpp
for (size_t i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);  // ⚠️ write() — безумовний запис
}
```

> [!WARNING]
> `EEPROM.write()` записує **завжди**, навіть якщо значення не змінилося. EEPROM має ресурс ~100 000 циклів запису. Повна очистка 1024 байтів при кожному скиданні — це зайвий знос.

**Рекомендація**: Використовувати `EEPROM.update()` замість `EEPROM.write()`:

```cpp
for (size_t i = 0; i < EEPROM.length(); i++) {
    EEPROM.update(i, 0); // Записує тільки якщо значення відрізняється
}
```

Або взагалі не очищати все: достатньо перезаписати `pData` і оновити checksum/uniqueID.

---

#### 9. `sprintf_P` з невірними форматами

**Файл**: [Spot-Welder-Pro-Fix.ino:1285-1293](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L1285-L1293)

```cpp
case VF_BATTALM: sprintf_P(str, PSTR("%1.1u.%01u"), val / 10, val % 10); break;
case VF_BATTV:   sprintf_P(str, PSTR("%2.1u.%02u"), val / 100, val % 100); break;
```

> [!WARNING]
> Формат `%2.1u` не має сенсу для цілих чисел — `.precision` для `%u` задає мінімальну кількість цифр, а не десяткові розряди. Код працює випадково, бо ви емулюєте десяткові дроби вручну (ділення/остача). Але формат `%1.1u` виведе мінімум 1 символ — це фактично те саме що `%u`.

**Рекомендація**: Спростити формати:

```cpp
case VF_BATTALM: sprintf_P(str, PSTR("%u.%u"), val / 10, val % 10); break;
case VF_BATTV:   sprintf_P(str, PSTR("%u.%02u"), val / 100, val % 100); break;
```

---

### 🟡 Покращення (якість коду та підтримуваність)

---

#### 10. Монолітна архітектура — 1465 рядків в одному файлі

Весь код зосереджено в одному `.ino` файлі. При розмірі ~65КБ це ускладнює навігацію та підтримку.

**Рекомендація**: Розділити на логічні модулі:

```
Spot-Welder-Pro-Fix/
├── Spot-Welder-Pro-Fix.ino   # setup(), loop(), stateMachine()
├── Spot-Welder-Pro.h         # Конфігурація та макроси
├── display.h / display.cpp   # Все що стосується OLED
├── menu.h / menu.cpp         # Логіка меню
├── pulse.h / pulse.cpp       # Зварювальний імпульс
├── sensors.h / sensors.cpp   # INA226, NTC, датчик Холла
├── eeprom_mgr.h / .cpp       # EEPROM управління
└── buzzer.h / buzzer.cpp     # Звукові сигнали
```

---

#### 11. Magic numbers у коді

Приклади з різних місць коду:

```cpp
INA.setMaxCurrentShunt(8, 0.01);   // Що означає 8? 0.01?
INA.setAverage(4);                  // AVR Bit [0-7] — яке реальне усереднення?
display(..., 800000L);              // 800kHz — чому саме?
delay(9);                           // Рядок 757 — магічне число 9мс
```

**Рекомендація**: Замінити на `#define` з пояснювальними іменами:

```cpp
#define INA_MAX_CURRENT_A    8      // Максимальний очікуваний струм (А)
#define INA_SHUNT_RESISTANCE 0.01   // Опір шунта (Ом)
#define INA_AVERAGING        4      // Кількість усереднень (AVG bit)
#define I2C_FAST_CLOCK       800000L // I2C Fast Mode Plus
#define ADC_SETTLE_TIME_MS   9      // Час стабілізації ADC (мс)
```

---

#### 12. `boolean` замість `bool` (Arduino legacy)

У коді використовується Arduino-специфічний тип `boolean`:

```cpp
boolean sysMenu = false;
static boolean lastBtnState = B_UP;
void sendWeldPulse(..., boolean senseActiveLevel);
```

**Рекомендація**: Замінити на стандартний C++ `bool` для портативності.

---

#### 13. Неузгоджена система одиниць напруги

| Змінна | Одиниці | Джерело |
|---|---|---|
| `batteryVoltage` | мВ (з INA226) | INA.getBusVoltage_mV() |
| `pData.batteryAlarm` | мВ/100 (40 = 4.0В) | EEPROM |
| `pData.PulseBatteryVoltage` | мВ (прямо з INA) | sendWeldPulse |
| `NOMINAL_VOLTAGE_FOR_COMPENSATION` | мВ (4200) | #define |
| `DEF_NOM_BATT_V` | мВ (5400) | #define |

> [!IMPORTANT]
> Змішування одиниць ускладнює розуміння та збільшує ризик помилок конвертації. Наприклад, `pData.batteryAlarm` зберігається як `мВ/100`, але порівнюється як `* 100` — це інвертована логіка.

**Рекомендація**: Уніфікувати все до мВ або ввести typedef:

```cpp
typedef uint16_t millivolts_t;
millivolts_t batteryVoltage;
millivolts_t batteryAlarm; // Зберігати в мВ безпосередньо
```

---

#### 14. Відсутній `default` у `valStr()` switch

**Файл**: [Spot-Welder-Pro-Fix.ino:1284-1294](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L1284-L1294)

```cpp
switch (fType) {
    case VF_BATTALM: ...; break;
    // ... інші case ...
    // ⚠️ Немає default:
}
```

**Рекомендація**: Додати `default:` з обробкою помилки:

```cpp
default: str[0] = '?'; str[1] = '\0'; break;
```

---

#### 15. Використання `float` на AVR без необхідності

**Файл**: [Spot-Welder-Pro-Fix.ino:727-740](file:///e:/Documents/Arduino/Spot-Welder-Pro-Fix/Spot-Welder-Pro-Fix.ino#L727-L740)

На ATmega328P немає апаратного FPU — операції з `float` виконуються програмно і споживають ~2КБ Flash + значний час CPU. В проекті `float` використовується для:
- Компенсації часу імпульсу
- `NTC` розрахунків (log, /)
- Виведення напруги в Serial

**Рекомендація**: Для компенсації імпульсу — перейти на цілочисельну арифметику (див. п.5). Для NTC — використовувати lookup-таблицю з інтерполяцією (значно швидше і компактніше). Serial-вивід — використовувати `valStr()`.

---

## Підсумок пріоритетів

| Пріоритет | Проблема | Вплив |
|---|---|---|
| 🔴 Критичний | Відсутній WDT під час роботи | Безпека: імпульс без контролю |
| 🔴 Критичний | `delay()` під час імпульсу | Безпека: CPU заблоковано |
| 🔴 Критичний | ADC/I2C під час імпульсу подовжують його | Неточний час імпульсу |
| 🔴 Критичний | `mEvent` без атомарного доступу | Втрата подій |
| 🟠 Суттєвий | Некоректна формула компенсації | Неточна зварка |
| 🟠 Суттєвий | Дублювання читання `batteryVoltage` | I2C шум, непередбачуваність |
| 🟠 Суттєвий | `EEPROM.write()` замість `update()` | Передчасний знос EEPROM |
| 🟠 Суттєвий | Невірні `sprintf_P` формати | Некоректне відображення |
| 🟡 Якість | Монолітна архітектура | Складність підтримки |
| 🟡 Якість | Magic numbers | Читабельність |
| 🟡 Якість | `float` на AVR | Продуктивність та Flash |
| 🟡 Якість | Неузгоджені одиниці | Ризик помилок |

## Рекомендований план дій

1. **Першочергово**: Увімкнути WDT + замінити `delay()` в імпульсі на апаратний таймер
2. **Далі**: Виправити порядок ADC/I2C (читати після вимкнення імпульсу) 
3. **Потім**: Атомарний доступ до `mEvent`, виправити формулу компенсації
4. **Після стабілізації**: Рефакторинг (модулі, magic numbers, одиниці)

> [!IMPORTANT]
> Бажаєте, щоб я реалізував будь-які з цих рекомендацій? Оберіть пріоритетні пункти, і я підготую конкретний план змін.
