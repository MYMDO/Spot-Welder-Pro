#define _Spot_Welder_Pro

#include <avr/wdt.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "Spot-Welder-Pro.h"
#include <INA226.h>

INA226 INA(0x40);

// Стани пристрою
enum states {
  ST_STANDBY,          // Стан пристрою: режим очікування
  ST_MAIN_SCREEN,      // Стан: відобразити головний екран
  ST_MAIN_SCREEN_CNT,  // Стан: відобразити статистику
  ST_MENU_SCREEN,      // Стан: відобразити екран меню
  ST_SUB_MENU_1,       // Стан: відобразити sub-menu1
  ST_SUB_MENU_2,       // Стан: відобразити sub-menu2
  ST_TEMP_HIGH,        // Стан: висока температура
  ST_SYSTEM_SCREEN,    // Стан: відображення системного екрана
  ST_SYSTEM_MENU,      // Стан: відображення системного меню
  ST_REBOOT_MENU,      // Стан: відобразити меню перезавантаження
  ST_MAXWELD_SCREEN,   // Стан: відобразити екран налаштування максимального зварного шва
  ST_INVERT_SCREEN,    // Стан: відобразити екран налаштування інвертування
};

// Події пристрою
enum event {
  // Приватні події пристрою
  EV_NONE,   // Подія: немає подій, що очікують на розгляд
  EV_BTNDN,  // Подія: натиснута кнопка
  EV_BTNUP,  // Подія: Кнопку відпущено
  EV_ENCUP,  // Подія: енкодер повертається вправо
  EV_ENCDN,  // Подія: енкодер обертається вліво

  // Загальнодоступні події пристрою
  EV_BOOTDN,        // Подія: натиснута кнопка при завантаженні
  EV_STBY_TIMEOUT,  // Подія: закінчився таймер очікування
  EV_TEMP_HIGH,     // Подія: досягнута максимальна температура
};

/***************************************************************************************************
* Глобальні програмні змінні та об'єкти                                                            *
***************************************************************************************************/

// Конструкції та об'єкти
progData pData;                                                 // Робочі дані програми
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET, 800000L);  // Об'єкт OLED-дисплея // 800kHz

// Статичні змінні
uint8_t mState = ST_MAIN_SCREEN;           // Поточний стан пристрою
uint8_t TCelsius;                          // Температура системи в градусах Цельсія
uint16_t batteryVoltage = DEF_NOM_BATT_V;  // Поточна напруга батареї x10
bool sysMenu = false;                   // У структурі меню системи
bool highVoltageAlarmActive = false;    // Прапор для неблокуючого сповіщення про високу напругу
bool lowVoltageAlarmActive = false;     // Прапор для неблокуючого сповіщення про низьку напругу
bool ina226Error = false;               // Прапор помилки сенсора INA226
bool ntcError = false;                  // Прапор помилки температурного датчика
bool ntcBypassed = false;               // Прапор ігнорування помилки температурного датчика
bool lowVoltageBypassed = false;        // Прапор ігнорування низької напруги
bool highVoltageBypassed = false;       // Прапор ігнорування високої напруги

// Нестабільні змінні - будуть змінені ІСР
volatile unsigned long lastActiveTime;
volatile uint8_t mEvent;  // Поточна очікувана подія пристрою

/**
 *  \brief  Атомарне зчитування та скидання події.
 *  Вимикає переривання на час доступу до volatile змінної mEvent,
 *  щоб запобігти втраті подій від ISR між читанням і очищенням.
 */
uint8_t atomicReadAndClearEvent() {
  cli();
  uint8_t ev = mEvent;
  mEvent = EV_NONE;
  sei();
  return ev;
}

/**
 *  \brief  Атомарне зчитування події без скидання.
 */
uint8_t atomicReadEvent() {
  cli();
  uint8_t ev = mEvent;
  sei();
  return ev;
}

/**
 *  \brief  Атомарний запис події.
 */
void atomicSetEvent(uint8_t ev) {
  cli();
  mEvent = ev;
  sei();
}

/**
 *  \brief  Атомарне зчитування lastActiveTime (4 байти, не атомарне на 8-біт AVR).
 */
unsigned long atomicReadLastActiveTime() {
  cli();
  unsigned long t = lastActiveTime;
  sei();
  return t;
}

/**
 *  \brief  Атомарний запис lastActiveTime.
 */
void atomicSetLastActiveTime(unsigned long t) {
  cli();
  lastActiveTime = t;
  sei();
}

void reset_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void reset_mcusr(void) {
  MCUSR = 0;
  wdt_disable();  // Тимчасово вимикаємо WDT під час завантаження; увімкнемо наприкінці setup()
}

/***************************************************************************************************
* Налаштування програми                                                                            *
***************************************************************************************************/
void setup() {
#if defined _DEVELOPMENT_ || defined _BOOTSYS_
  Serial.begin(_SERIAL_BAUD_);
#endif /* _DEVELOPMENT_ || _BOOTSYS_*/

  // Налаштуйте стан і напрямок контактів.
  pinMode(PIN_PULSE, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_CLK, INPUT);
  pinMode(PIN_DT, INPUT);
  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(PIN_FOOT_SWITCH, INPUT_PULLUP);
  pinMode(PIN_AUTO_PULSE, INPUT);
  digitalWrite(PIN_SW, HIGH);
  digitalWrite(PIN_FOOT_SWITCH, HIGH);
  bool bootMenuRequested = (btnState() == B_DN);

  // Ініціалізуємо I2C перед дисплеєм та INA226
  Wire.begin();

  // Налаштуйте OLED-дисплей і очистіть його.
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();  // очищати залишки при перезавантаженні
  display.display();

  // Переривання використовується для визначення обертання кодера.
  attachInterrupt(ENC_INT_NUM, isr, FALLING);

  // Це використовується таймером режиму очікування для визначення періоду бездіяльності.
  lastActiveTime = millis();

  // Stored bootMenuRequested instead of setting mEvent immediately to avoid ISR overwrite during boot delays.

  loadEEPROM();

  // Інвертуйте дисплей, якщо потрібно, і малюйте заставку.
  display.setRotation(pData.pFlags.en_oledInvert ? OLED_INVERT : 0);
  splash();

  if (!digitalRead(PIN_FOOT_SWITCH)) {
    foot_switch_error();  // Показуємо помилку і чекаємо відпускання перемикача
    reboot();             // Перезавантажуємо для чистого старту
  }

  batteryVoltage = DEF_NOM_BATT_V;
  Boot_Sound();

#ifdef _DEVELOPMENT_
  Serial.print(F("Pulse Voltage    "));
  Serial.println(pData.PulseBatteryVoltage);
  Serial.print(F("Pulse Amps       "));
  Serial.println(pData.PulseAmps);
  Serial.print(F("Battery Alarm    "));
  Serial.println(pData.batteryAlarm);
  Serial.print(F("Weld Count       "));
  Serial.println(pData.weldCount);
  Serial.print(F("Auto Pulse Delay "));
  Serial.println(pData.autoPulseDelay);
  Serial.print(F("Max Pulse Time   "));
  Serial.println(pData.maxPulseTime);
  Serial.print(F("Short Pulse Time "));
  Serial.println(pData.shortPulseTime);
  Serial.print(F("Auto Pulse       "));
  Serial.println(pData.pFlags.en_autoPulse ? "On" : "Off");
  Serial.print(F("Weld Sound       "));
  Serial.println(pData.pFlags.en_Sound ? "On" : "Off");
  Serial.print(F("Display Invert   "));
  Serial.println(pData.pFlags.en_oledInvert ? "Invert" : "Normal");
  Serial.print(F("Pulse Time       "));
  Serial.println(pData.pulseTime);
#endif /* _DEVELOPMENT_ */

  // Ініціалізація INA226 з повторними спробами
  {
    uint8_t inaRetries = 3;
    bool inaOk = false;
    while (inaRetries > 0) {
      if (INA.begin()) {
        inaOk = true;
        break;
      }
      inaRetries--;
      delay(100);
    }
    if (!inaOk) {
#ifdef _DEVELOPMENT_
      Serial.println(F("INA226 init failed after 3 retries! Starting with SENSOR ERROR..."));
#endif
      ina226Error = true;
    }
  }
  INA.setMaxCurrentShunt(INA_MAX_CURRENT_A, INA_SHUNT_RESISTANCE_OHM);
  INA.setAverage(INA_AVERAGING_MODE);

  // Зчитуємо первинну напругу батареї після ініціалізації
  batteryVoltage = INA.getBusVoltage_mV();
  if (batteryVoltage == 0) {
    batteryVoltage = DEF_NOM_BATT_V;
  }

  if (bootMenuRequested) {
    mEvent = EV_BOOTDN;
  } else {
    mEvent = EV_NONE;
  }

  // Увімкнення Watchdog Timer як захист від зависання.
  wdt_enable(WDTO_250MS);
}

/***************************************************************************************************
* Основний програмний цикл                                                                         *
***************************************************************************************************/

//Основний цикл програми.
void loop() {
  wdt_reset();  // Скидаємо Watchdog кожну ітерацію — доказ що CPU живий

  // Запускається кожен цикл - настільки швидко, наскільки це можливо.
  stateMachine();

  // EEPROM оновлюється зміненими змінними.
  updateEEPROM();
}

/***************************************************************************************************
* Перервати програму обслуговування                                                                *
***************************************************************************************************/

// Перервати рутину обслуговування.
void isr() {
  static volatile unsigned long lastInterruptTime = 0;

  // Ігноруйте зміни, які відбуваються протягом періоду дебаунсу.
  if ((millis() - lastInterruptTime) > RS_DEBOUNCE) {

    // До кодера є два контакти. Один пін видає переривання. Друга шпилька
    // тому вказує на фазу, яка фактично є напрямком обертання.
    // Зчитайте вивід фази та видайте подію напрямку енкодера на основі його стану.
    mEvent = digitalRead(PIN_DT) ? EV_ENCUP : EV_ENCDN;
    lastActiveTime = lastInterruptTime = millis();  // Оновлюємо час останньої активності
  }
}
