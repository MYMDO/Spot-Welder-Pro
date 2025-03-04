#define _Spot_Welder_Pro

#include <avr/wdt.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "printf.h"
#include "Spot-Welder-Pro.h"
#include <INA226.h>

INA226 INA(0x40);

// Стани машини
enum states {
  ST_STANDBY,          // Стан машини: режим очікування
  ST_MAIN_SCREEN,      // Стан: відобразити головний екран
  ST_MAIN_SCREEN_CNT,  // Стан: відобразити статистику
  ST_MENU_SCREEN,      // Стан: відобразити екран меню
  ST_SUB_MENU_1,       // Стан: відобразити sub-menu1
  ST_SUB_MENU_2,       // Стан: відобразити sub-menu2
  ST_BATTERY_LOW,      // Стан: низький заряд батареї
  ST_BATTERY_HIGH,     // Стан: висока напруга акумулятора
  ST_TEMP_HIGH,        // Стан: висока температура
  ST_SYSTEM_SCREEN,    // Стан: відображення системного екрана
  ST_SYSTEM_MENU,      // Стан: відображення системного меню
  ST_REBOOT_MENU,      // Стан: відобразити меню перезавантаження
  ST_MAXWELD_SCREEN,   // Стан: відобразити екран налаштування максимального зварного шва
  ST_INVERT_SCREEN,    // Стан: відобразити екран налаштування інвертування
  ST_PULSE_VOLTAGE,    // Стан: відображення екрана виміряної імпульсної напруги
};

// Події машини
enum event {
  // Приватні події машини
  EV_NONE,   // Подія: немає подій, що очікують на розгляд
  EV_BTNDN,  // Подія: натиснута кнопка
  EV_BTNUP,  // Подія: Кнопку відпущено
  EV_ENCUP,  // Подія: енкодер повертається вправо
  EV_ENCDN,  // Подія: енкодер обертається вліво

  // Загальнодоступні події машини
  EV_BOOTDN,        // Подія: натиснута кнопка при завантаженні
  EV_STBY_TIMEOUT,  // Подія: закінчився таймер очікування
  EV_BATT_LV,       // Подія: подія низької напруги акумулятора
  EV_BATT_HV,       // Подія: подія високої напруги акумулятора
  EV_TEMP_HIGH,     // Подія: досягнута максимальна температура
  EV_EEUPD,         // Подія: EEPROM потребує оновлення
  EV_PULSE_VOLTS,   // Подія: утримання ножного перемикача під час імпульсу
};

/***************************************************************************************************
* Глобальні програмні змінні та об’єкти                                                            *
***************************************************************************************************/

// Конструкції та об'єкти
progData pData;                                                 // Робочі дані програми
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET, 800000L);  // Об'єкт OLED-дисплея // 800kHz

// Статичні змінні
uint8_t mState = ST_MAIN_SCREEN;           // Поточний стан машини
uint8_t TCelsius;                          // Температура системи в градусах Цельсія
uint16_t batteryVoltage = DEF_NOM_BATT_V;  // Поточна напруга батареї x10
boolean sysMenu = false;                   // У структурі меню системи
bool autoPulseTriggered = false;           // Змінна для відстеження стану

// Нестабільні змінні - будуть змінені ІСР
volatile unsigned long lastActiveTime;
volatile uint8_t mEvent;  // Поточна очікувана подія машини

// unsigned long btnPressStart = 0;
// const unsigned long LONG_PRESS_THRESHOLD = 999; // ПОРІГ ТРИВАЛОГО НАТИСКАННЯ

void reset_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void reset_mcusr(void) {
  MCUSR = 0;
  wdt_disable();
}

/***************************************************************************************************
* Налаштування програми                                                                            *
***************************************************************************************************/

// Налаштування програми.
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

  // Налаштуйте OLED-дисплей і очистіть його.
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();  // очищати залишки при перезавантаженні
  display.display();

  // Переривання використовується для визначення обертання кодера. Так само можна було б провести опитування
  // без втрати чутливості. Це було фактично спробовано, але не було помітної продуктивності
  // спостерігається деградація. Переривання корисні для високошвидкісних кодерів, таких як
  // використовується в сервосистемах. Налаштовані вручну кодери дуже повільні.
  attachInterrupt(ENC_uint16_t, isr, FALLING);

  // Це використовується таймером режиму очікування для визначення періоду бездіяльності.
  lastActiveTime = millis();

  // Перевірити, чи натиснуто кнопку під час завантаження. Якщо так, то забезпечте вхід у системе
  // меню за допомогою події натискання кнопки завантаження.
#ifdef _BOOTSYS_
  mEvent = EV_BOOTDN;
#else
  mEvent = btnState() == B_DN ? EV_BOOTDN : EV_NONE;
#endif /* _BOOTSYS_ */

  loadEEPROM();

  // Інвертуйте дисплей, якщо потрібно, і малюйте заставку. Це можна зробити лише після
  // завантаження даних програми (яка містить перемикач інвертування екрана).
  display.setRotation(pData.pFlags.en_oledInvert ? OLED_INVERT : 0);
  splash();

  if (!digitalRead(PIN_FOOT_SWITCH)) {
    foot_switch_error();
    FootSwitch_Alarm();
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


  Wire.begin();  // Запускаємо I2C, якщо ще не запущено
  if (!INA.begin()) {
    Serial.println("Ініціалізація INA226 завершилася невдало!");
    while (1)
      ;  // Зупинка, якщо ініціалізація не вдалася
  }
  INA.setMaxCurrentShunt(8, 0.01);  // (79, 0.001);
  INA.setAverage(4);                // Default: [0] // AVG Bit [0-7]
}

/***************************************************************************************************
* Основний програмний цикл                                                                         *
***************************************************************************************************/

//Основний цикл програми.
void loop() {
  // Машина станів запускається кожен цикл циклу - настільки швидко, наскільки це можливо.
  stateMachine();

  // EEPROM оновлюється зміненими змінними. Внутрішня процедура оновлення забезпечує це
  // EEPROM оновлюється лише кожні кілька секунд, щоб обмежити зношення.
  updateEEPROM();
}

/***************************************************************************************************
* Стан машини                                                                                      *
***************************************************************************************************/

//Реалізація стану машини.

// Глобальна структура для стану меню
struct MenuState {
  uint8_t selectedMenu = 0;
  uint8_t selectedMainMenu = 0;
  uint8_t selectedSubMenu = 0;
  bool btn = false;
};

MenuState menuState;  // Глобальна змінна для стану меню

void stateMachine() {
  char str[5];
  // Сканування подій - довжина черги подій дорівнює одиниці.
  // Обробляти будь-які публічні події завантаження. Ці події мають найвищий пріоритет
  // і має бути оброблено перед будь-якою приватною подією.
  if (mEvent == EV_BOOTDN) {
    mState = ST_SYSTEM_SCREEN;
    mEvent = EV_NONE;

  } else {
    // Шукайте та обробляйте будь-які приватні події.
    checkForBtnEvent();
    checkForSleepEvent();
    checkForLowVoltageEvent();
    checkTemp();

    // Обробка подій

    switch (mEvent) {
      case EV_STBY_TIMEOUT:
        mState = ST_STANDBY;
        mEvent = EV_NONE;
        break;
      case EV_BATT_LV:

        if (!sysMenu) mState = ST_BATTERY_LOW;

        mEvent = EV_NONE;
        break;
      case EV_BATT_HV:

        if (!sysMenu) mState = ST_BATTERY_HIGH;

        mEvent = EV_NONE;
        break;
      case EV_TEMP_HIGH:

        if (!sysMenu) mState = ST_TEMP_HIGH;

        mEvent = EV_NONE;
        break;
      case EV_BTNUP:

        if (mState != ST_REBOOT_MENU) mEvent = EV_NONE;

        break;
      case EV_PULSE_VOLTS:
        mState = ST_PULSE_VOLTAGE;
        mEvent = EV_NONE;
        break;
    }
  }

  // Машина станів
  switch (mState) {
    case ST_PULSE_VOLTAGE:
      handlePulseVoltageState();
      break;

    case ST_STANDBY:
      handleStandbyState();
      break;

    case ST_BATTERY_LOW:
      handleBatteryLowState();
      break;

    case ST_BATTERY_HIGH:
      handleBatteryHighState();
      break;

    case ST_TEMP_HIGH:
      handleTempHighState();
      break;

    case ST_MAIN_SCREEN:
      enterMainScreen();
      break;

    case ST_MAIN_SCREEN_CNT:
      handleMainScreenCnt();
      break;

    case ST_MENU_SCREEN:
      handleMenuScreen(str);
      break;

    case ST_SUB_MENU_1:
      handleSubMenu1(str);
      break;

    case ST_SUB_MENU_2:
      handleSubMenu2(str);
      break;

    case ST_SYSTEM_SCREEN:
      enterSystemScreen();
      break;

    case ST_SYSTEM_MENU:
      handleSystemMenu(str);
      break;

    case ST_REBOOT_MENU:
      handleRebootMenu(str);
      break;

    case ST_MAXWELD_SCREEN:
      handleMaxWeldScreen(str);
      break;

    case ST_INVERT_SCREEN:
      handleInvertScreen();
      break;

    default:
      break;
  }
}

void handlePulseVoltageState() {
  if (digitalRead(PIN_FOOT_SWITCH)) {
    mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  } else {
    mEvent = EV_NONE;
  }
}

void handleStandbyState() {
  if (mEvent == EV_BTNDN) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  message(FPSTR(LS_STANDBY), FPSTR(LS_CLICKBTN), FPSTR(LS_EXITSTBY));
  mEvent = EV_NONE;
}

void handleBatteryLowState() {
  if (mEvent == EV_BTNDN) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  displayLowBattery();
  LowBattery_Sound();
  mEvent = EV_NONE;
}

void handleBatteryHighState() {
  if (mEvent == EV_BTNDN) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  displayHighBattery();
  mEvent = EV_NONE;
}

void handleTempHighState() {
  if (mEvent == EV_BTNDN) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  displayHighTemperature();
  mEvent = EV_NONE;
}

void enterMainScreen() {
  mState = ST_MAIN_SCREEN_CNT;
  sysMenu = false;
  menuState.selectedMenu = 0;
  displayMainScreen();
}

void handleMainScreenCnt() {
  static bool autoPulseTriggered = false;

  // Обробка автоматичного імпульсу
  if (digitalRead(PIN_AUTO_PULSE) && pData.pFlags.en_autoPulse) {
    if (!autoPulseTriggered) {
      autoPulseTriggered = true;  // Запобігання повторній активації
      displayMainScreen(1);
      if (pData.pFlags.en_Sound) {
        playBeep(1000, 100);  // Звуковий сигнал
      }
      sendWeldPulse(PIN_AUTO_PULSE, pData.autoPulseDelay * 100, WP_RETRIGGER_DELAY);
      logPulseInfo();
      displayMainScreen();
    }
  } else if (!digitalRead(PIN_FOOT_SWITCH)) {
    // Обробка ножного перемикача
    sendWeldPulse(PIN_FOOT_SWITCH, FS_TRIGGER_DELAY, WP_RETRIGGER_DELAY, PL_ACTIVE_L);
    logPulseInfo();
  } else {
    // Скидання стану автоімпульсу
    autoPulseTriggered = false;
  }

  // Обробка події натискання кнопки
  if (mEvent == EV_BTNDN) {
    mState = ST_MENU_SCREEN;
    menuState.selectedMenu = 0;
    mEvent = EV_NONE;
    displayMenuType1(nullptr, FPSTR(LS_APULSE), FPSTR(LS_BATTALM1), FPSTR(LS_SHORTPLS1), 0);
    return;
  }

  // Обробка подій енкодера
  if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    handleEncoderEvent();
    mEvent = EV_NONE;
  }

  displayMainScreen();
}
// Обробка подій енкодера
void handleEncoderEvent() {
  if (mEvent == EV_ENCUP) {
    pData.pulseTime = (pData.pulseTime < pData.maxPulseTime) ? pData.pulseTime + 1 : pData.maxPulseTime;
  } else {
    pData.pulseTime = (pData.pulseTime > MIN_PULSE_TIME) ? pData.pulseTime - 1 : MIN_PULSE_TIME;
  }
}

void handleMenuScreen(char *str) {
  // Подія натискання кнопки перейде на наступний рівень структури меню. Є
  // три пункти в меню, щоб вирішити, яке нове меню ввійти.
  if (mEvent == EV_BTNDN) {
    mState = ST_SUB_MENU_1;
    menuState.selectedMainMenu = menuState.selectedMenu;

    // Увійдіть у підменю, щоб вибрати додаткові параметри.
    if (menuState.selectedMainMenu == 0)
      displayMenuType1(FPSTR(LS_APULSE), FPSTR(LS_MANAUTO),
                       FPSTR(LS_DELAY), FPSTR(LS_WELDSOUND), menuState.selectedMenu);

    // Дозволити налаштування напруги сигналізації про низький рівень батареї.
    else if (menuState.selectedMainMenu == 1) displayMenuType2(FPSTR(LS_BATTALM),
                                                               valStr(str, pData.batteryAlarm, VF_BATTALM), FPSTR(LS_VOLTAGE));

    // Дозволити налаштування тривалості імпульсу попереднього нагріву.
    else if (menuState.selectedMainMenu == 2) displayMenuType2(FPSTR(LS_SHORTPLS),
                                                               valStr(str, pData.shortPulseTime, VF_SHTPLS), FPSTR(LS_PCOF));

    mEvent = EV_NONE;
    menuState.selectedMenu = 0;

    // Події поворотного енкодера дозволять прокручувати параметри меню.
  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    if (mEvent == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
    else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

    mEvent = EV_NONE;
    displayMenuType1(NULL, FPSTR(LS_APULSE), FPSTR(LS_BATTALM1),
                     FPSTR(LS_SHORTPLS1), menuState.selectedMenu);
  }
}

void handleSubMenu1(char *str) {
  // Подія натискання кнопки перейде на наступний рівень структури меню.
  // Є три пункти в меню, щоб вирішити, яке нове меню ввійти.
  if (mEvent == EV_BTNDN) {
    if (menuState.selectedMainMenu == 0) {
      mState = ST_SUB_MENU_2;
      menuState.selectedSubMenu = menuState.selectedMenu;

      // Встановіть статус автоматичного пульсу.
      if (menuState.selectedSubMenu == 0) displayMenuType2(FPSTR(LS_AUTOPLSON), NULL,
                                                           pData.pFlags.en_autoPulse ? FPSTR(LS_AUTO) : FPSTR(LS_MANUAL));

      // Дозволити установку часу затримки автоматичного імпульсу.
      else if (menuState.selectedSubMenu == 1) displayMenuType2(FPSTR(LS_AUTOPLSDLY),
                                                                valStr(str, pData.autoPulseDelay, VF_DELAY), FPSTR(LS_SECONDS));

      // Встановіть звук активації імпульсу.
      else if (menuState.selectedSubMenu == 2) displayMenuType2(FPSTR(LS_WELDSOUNDM), NULL,
                                                                pData.pFlags.en_autoPulse ? FPSTR(LS_SOUNDON) : FPSTR(LS_SOUNDOFF));

    } else if (menuState.selectedMainMenu == 1) mState = ST_MAIN_SCREEN;

    else if (menuState.selectedMainMenu == 2) mState = ST_MAIN_SCREEN;
    else mState = ST_MAIN_SCREEN;

    mEvent = EV_NONE;
    menuState.selectedMenu = 0;

    // Тип запису меню вже вибрано з попереднього меню.
    // Ось змінні змінюються відповідно до вибраного типу.
  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    if (menuState.selectedMainMenu == 0) {
      if (mEvent == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
      else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

      displayMenuType1(FPSTR(LS_APULSE), FPSTR(LS_MANAUTO),
                       FPSTR(LS_DELAY), FPSTR(LS_WELDSOUND), menuState.selectedMenu);

      // Встановіть тривожну напругу батареї, реагуючи на події поворотного енкодера.
      // Напруга тривоги батареї не може бути вище поточної напруги батареї
      // і в будь-якому випадку не вище максимально допустимої напруги тривоги.
    } else if (menuState.selectedMainMenu == 1) {
      if (mEvent == EV_ENCDN) pData.batteryAlarm = pData.batteryAlarm > MIN_BATT_BALARM / 100 ? pData.batteryAlarm - 1 : MIN_BATT_BALARM / 100;
      else pData.batteryAlarm = pData.batteryAlarm < min((uint16_t)batteryVoltage / 100, (uint16_t)MAX_BATT_ALARM / 100) ? pData.batteryAlarm + 1 : min((uint16_t)batteryVoltage / 100, (uint16_t)MAX_BATT_ALARM / 100);

      displayMenuType2(FPSTR(LS_BATTALM),
                       valStr(str, pData.batteryAlarm, VF_BATTALM), FPSTR(LS_VOLTAGE));

      // Встановіть тривалість імпульсу попереднього нагрівання, реагуючи на події поворотного регулятора.
    } else if (menuState.selectedMainMenu == 2) {
      if (mEvent == EV_ENCDN) pData.shortPulseTime = pData.shortPulseTime > MIN_SPULSE_TIME ? pData.shortPulseTime - 1 : MIN_SPULSE_TIME;
      else pData.shortPulseTime = pData.shortPulseTime < MAX_SPULSE_TIME ? pData.shortPulseTime + 1 : MAX_SPULSE_TIME;

      displayMenuType2(FPSTR(LS_SHORTPLS),
                       valStr(str, pData.shortPulseTime, VF_SHTPLS), FPSTR(LS_PCOF));
    }
    mEvent = EV_NONE;
  }
}

void handleSubMenu2(char *str) {
  if (mEvent == EV_BTNDN) {
    mState = ST_MAIN_SCREEN;
    mEvent = EV_NONE;
    menuState.selectedMenu = 0;

  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    if (menuState.selectedSubMenu == 0) {
      pData.pFlags.en_autoPulse ^= 1;
      displayMenuType2(FPSTR(LS_AUTOPLSON), NULL, pData.pFlags.en_autoPulse ? FPSTR(LS_AUTO) : FPSTR(LS_MANUAL));
    } else if (menuState.selectedSubMenu == 1) {
      if (mEvent == EV_ENCDN) pData.autoPulseDelay = pData.autoPulseDelay > MIN_APULSE_DELAY ? pData.autoPulseDelay - 1 : MIN_APULSE_DELAY;
      else pData.autoPulseDelay = pData.autoPulseDelay < MAX_APULSE_DELAY ? pData.autoPulseDelay + 1 : MAX_APULSE_DELAY;

      displayMenuType2(FPSTR(LS_AUTOPLSDLY),
                       valStr(str, pData.autoPulseDelay, VF_DELAY), FPSTR(LS_SECONDS));
    } else if (menuState.selectedSubMenu == 2) {
      pData.pFlags.en_Sound ^= 1;
      displayMenuType2(FPSTR(LS_WELDSOUNDM), NULL, pData.pFlags.en_Sound ? FPSTR(LS_SOUNDON) : FPSTR(LS_SOUNDOFF));
    }
    mEvent = EV_NONE;
  }
}

void enterSystemScreen() {
  mState = ST_SYSTEM_MENU;
  sysMenu = true;
  menuState.selectedMenu = 0;
  displayMenuType1(FPSTR(LS_SYSMENU), FPSTR(LS_MAXPULSE),
                   FPSTR(LS_DISPLAY), FPSTR(LS_BOOT), 0);
}

void handleSystemMenu(char *str) {
  if (mEvent == EV_BTNDN) {
    mEvent = EV_NONE;

    if (menuState.selectedMenu == 0) {
      displayMenuType2(FPSTR(LS_MAXPLSMENU),
                       valStr(str, pData.maxPulseTime, VF_PLSDLY), FPSTR(LS_MS));
      mState = ST_MAXWELD_SCREEN;
      mEvent = EV_NONE;

    } else if (menuState.selectedMenu == 1) {
      menuState.selectedSubMenu = 0;
      mState = ST_INVERT_SCREEN;
      displayMenuType2(FPSTR(LS_INVERTMENU), NULL, pData.pFlags.en_oledInvert ? FPSTR(LS_SCRINV) : FPSTR(LS_SCRNORM));

    } else if (menuState.selectedMenu == 2) {
      menuState.btn = false;
      mState = ST_REBOOT_MENU;
      displayMenuType1(FPSTR(LS_BOOTMENU), FPSTR(LS_REBOOT),
                       FPSTR(LS_SAFERST), FPSTR(LS_FULLRST), menuState.selectedMenu = 0);
    }

    menuState.selectedMenu = 0;

  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    if (mEvent == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
    else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

    mEvent = EV_NONE;
    displayMenuType1(FPSTR(LS_SYSMENU), FPSTR(LS_MAXPULSE),
                     FPSTR(LS_DISPLAY), FPSTR(LS_BOOT), menuState.selectedMenu);
  }
}

void handleRebootMenu(char */*str*/) {
  if (mEvent == EV_BTNDN) menuState.btn = true;
  else if ((mEvent == EV_BTNUP) && menuState.btn) {
    if (menuState.selectedMenu == 1)
      resetEEPROM(false);
    else if (menuState.selectedMenu == 2)
      resetEEPROM(true);

    message(FPSTR(LS_REBOOT), menuState.selectedMenu == 1 ? FPSTR(LS_REBOOTSR) : menuState.selectedMenu == 2 ? FPSTR(LS_REBOOTFR)
                                                                                                             : FPSTR(LS_REBOOTNR),
            FPSTR(LS_WAITMSG), 2);

    // updateEEPROM();
    delay(1000);
    reboot();


  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    if (mEvent == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
    else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

    displayMenuType1(FPSTR(LS_BOOTMENU), FPSTR(LS_REBOOT),
                     FPSTR(LS_SAFERST), FPSTR(LS_FULLRST), menuState.selectedMenu);
  }
  mEvent = EV_NONE;
}

void handleMaxWeldScreen(char *str) {
  if (mEvent == EV_BTNDN) {
    message(FPSTR(LS_MAXPULSE), FPSTR(LS_MAXPMSG), FPSTR(LS_WAITMSG), 2);
    mState = ST_SYSTEM_SCREEN;
    menuState.selectedMenu = 0;

  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    if (mEvent == EV_ENCDN) pData.maxPulseTime = pData.maxPulseTime > MIN_PULSE_TIME ? pData.maxPulseTime - 1 : MIN_PULSE_TIME;
    else pData.maxPulseTime = pData.maxPulseTime < MAX_PULSE_TIME ? pData.maxPulseTime + 1 : MAX_PULSE_TIME;

    pData.maxPulseTime = pData.maxPulseTime < MIN_PULSE_TIME ? MIN_PULSE_TIME : pData.maxPulseTime > MAX_PULSE_TIME ? MAX_PULSE_TIME
                                                                                                                    : pData.maxPulseTime;
    displayMenuType2(FPSTR(LS_MAXPLSMENU),
                     valStr(str, pData.maxPulseTime, VF_PLSDLY), FPSTR(LS_MS));
  }
  mEvent = EV_NONE;
}

void handleInvertScreen() {
  if (mEvent == EV_BTNDN) {
    mState = ST_SYSTEM_SCREEN;
    mEvent = EV_NONE;
    menuState.selectedMenu = 0;

  } else if (mEvent == EV_ENCUP || mEvent == EV_ENCDN) {
    pData.pFlags.en_oledInvert = !pData.pFlags.en_oledInvert;
    display.setRotation(pData.pFlags.en_oledInvert ? 2 : 0);
    displayMenuType2(FPSTR(LS_INVERTMENU), NULL, pData.pFlags.en_oledInvert ? FPSTR(LS_SCRINV) : FPSTR(LS_SCRNORM));
  }
  mEvent = EV_NONE;
}

/***************************************************************************************************
* Процедури управління обладнанням                                                                 *
***************************************************************************************************/
struct progress {
  uint16_t time;
#define PGR_OFF (0 << 0)
#define PGR_ON (1 << 0)
#define PGR_INIT (1 << 7)
  uint16_t opt;
  uint16_t step;
  unsigned long millis;
};

/**
 *  \brief                      Малює індикатор прогресу
 *  \param [in/out] o           структура контролю прогресу
 *  \param [in] clear           очистити індикатор прогресу, якщо true
 *  \return Нуль, якщо прогрес не завершено, інакше нуля немає
 */

uint16_t drawProgress(struct progress *o, bool clear) {
  const uint16_t steps = 126;  // 25;
  const uint16_t height = 8;   // 4;
  const uint16_t x = SSD1306_LCDWIDTH - steps - 1;
  const uint16_t y = 55;  //21;

  if (clear) {
    // Очищення індикатора
    display.fillRect(x - 1, y - 1, steps + 2, height + 2, BLACK);
    return 0;
  }

  if (!(o->opt & PGR_INIT)) {
    // Ініціалізація індикатора
    display.drawRect(x - 1, y - 1, steps + 2, height + 2, WHITE);
    display.fillRect(x, y, steps, height, (o->opt & PGR_ON) ? BLACK : WHITE);
    display.display();
    o->step = 1;
    o->millis = millis();
    o->opt |= PGR_INIT;
    return 0;
  }

  // Оновлення прогресу
  uint16_t elapsed = millis() - o->millis;
  uint16_t expectedStep = elapsed / (o->time / steps);

  if (expectedStep > o->step) {
    o->step = expectedStep;
    display.fillRect(x, y, o->step, height, (o->opt & PGR_ON) ? WHITE : BLACK);
    display.display();
  }

  return (o->step >= steps) ? 1 : 0;
}



/**
 *  \brief      Генерує зварювальний імпульс.
 *  \param [in] uint8_t  sensePin         Штифт датчика включення імпульсу
 *  \param [in] uint16_t delayEngage      Затримка перед спрацьовуванням імпульсу (затримка спрацьовування)
 *  \param [in] uint16_t delayRelease     Затримка після відключення імпульсу (затримка повторного запуску)
 *  \param [in] boolean  senseActiveLevel Логіка виводу датчика. За замовчуванням = PL_ACTIVE_H.
 */

void sendWeldPulse(uint8_t sensePin, uint16_t delayEngage, uint16_t delayRelease, bool senseActiveLevel) {
  struct progress wait;
  bool activePinState = (senseActiveLevel == PL_ACTIVE_H);
  unsigned long shortPulseDelay = max(1UL, (pData.pulseTime * pData.shortPulseTime) / 100);

#ifdef _DEVELOPMENT_
  Serial.println(F("Авто-імпульс активовано!"));
#endif

  // Затримка активації
  if (sensePin == PIN_AUTO_PULSE) {
    wait.opt = PGR_ON;
    wait.time = delayEngage;
    while (!drawProgress(&wait, false)) {
      if (digitalRead(PIN_AUTO_PULSE) != activePinState) {
        drawProgress(&wait, true);
        return;
      }
    }
  } else {
    delay(delayEngage);
  }

  // Звуковий сигнал активації
  if (pData.pFlags.en_Sound) playBeep(1500, 100);

  // Попередній нагрів (короткий імпульс)
  if (pData.shortPulseTime > 0 && pData.pulseTime > 3) {
    weldPulse(Pulse_ON);
    delay(shortPulseDelay);
    weldPulse(Pulse_OFF);
    delay(shortPulseDelay);
  }

  // Основний імпульс
  Wire.setClock(800000);

  // Зчитування напруги до імпульсу
  uint16_t NominalGauss = analogRead(A0);
  delay(9);
  weldPulse(Pulse_ON);  // >>> Start Pulse Enabled
  delay((pData.pulseTime));

  // Зчитування напруги під час імпульсу
  uint16_t PulseGauss = analogRead(A0);
  uint16_t busVoltageDuring = INA.getBusVoltage_mV();

  weldPulse(Pulse_OFF);  // <<< Stop Pulse Disabled

  // Розрахунок імпульсного струму в Амперах:
  pData.PulseBatteryVoltage = (float)(busVoltageDuring);
  pData.PulseAmps = map(PulseGauss, NominalGauss, 1023, 0, (1023 - NominalGauss) * 10);
  // pData.PulseAmps = map(PulseGauss, NominalGauss, 900, 0, 3880);
  // 10A 1-поділка з A0 = 5mV (900 - 512 = 388 * 10) // 3880/(900-512)=10A при 5mV з SS49E.
  // PulseAmps: Струм імпульсу (А), датчик SS49E (A0), ~10А/поділка, калибровка нуля по NominalGauss.

  unsigned long startMillis = millis();

  // Очікування деактивації датчика
  while (digitalRead(sensePin) == activePinState && (millis() - startMillis < PV_DELAY)) {
    weldPulse(Pulse_OFF);
  }

  if (millis() - startMillis > PV_DELAY) {
    mEvent = EV_PULSE_VOLTS;
  }

  // Затримка перед повторною активацією
  wait.opt = PGR_OFF;
  wait.time = delayRelease;
  while (!drawProgress(&wait, false)) {
    continue;
  }

  // Оновлення стану
  pData.weldCount++;
  lastActiveTime = millis();

  if (!digitalRead(PIN_FOOT_SWITCH)) {
    mEvent = EV_PULSE_VOLTS;
  }
}

void checkForLowVoltageEvent() {
  static unsigned long lastBVTime = 1;
  if (millis() - lastBVTime > BV_INTERVAL) {
    lastBVTime = millis();

#ifdef _TESTING_
    // Ігнорувати низький заряд акумулятора для тестування інтерфейсу.
    // Це запобігає блокуванню низької напруги під час роботи від USB без підключеного акумулятора 12 В.
    batteryVoltage = DEF_NOM_BATT_V;
#endif /* _TESTING_ */

    // Якщо напруга акумулятора низька, видайте подію про низьку напругу.
    if (pData.batteryAlarm > batteryVoltage) mEvent = EV_BATT_LV;

    // Якщо напруга акумулятора надто висока, з'явиться подія про високу напругу.
    if (pData.batteryhighAlarm < batteryVoltage) mEvent = EV_BATT_HV;
  }
}


//Перевіряє температуру пристрою та реагує на результат.
void checkTemp() {

  static unsigned long lastTTime = 0;

  // Не зчитуйте напругу акумулятора занадто часто.
  if (millis() - lastTTime > T_INTERVAL) {
    lastTTime = millis();

    // Зчитайте аналогове значення з аналогового входу та обчисліть опір NTC
    int bitwertNTC = analogRead(PIN_TEMP);

    // Розрахунок опору NTC
    double ratio = (double)bitwertNTC / 1024;              // Відношення значення до максимального
    double widerstandNTC = 10000 * (ratio / (1 - ratio));  // 10kΩ - номінальний опір

    // Обчислити температуру в градусах Цельсія
    double TdCelsius = (1 / ((1 / 298.15) + ((double)1 / 3900) * log((double)widerstandNTC / 10000))) - 273.15;  //273.15 = 0° Celsius in kelvin , 298.15 = 273.15 + 25, where 25 is the nominal temperature of the NTC, 3900 = B-value of the NTC
    TCelsius = uint8_t(TdCelsius + 0.5);                                                                         //округлити до найближчого повного числа та перетворити на ціле число

    // Якщо температура занадто висока, видайте подію про високу температуру
    if (TCelsius > DEF_HIGH_TEMP_ALARM) mEvent = EV_NONE;  // EV_TEMP_HIGH;
  }
}

// Видає подію тайм-ауту режиму очікування, якщо час очікування минув без жодної активності.
void checkForSleepEvent() {
  // Час останньої активності оновлюється кожного разу, коли відбувається якась активність. Якщо тайм-аут режиму очікування
  // закінчився період без будь-якої активності, тоді видається подія тайм-ауту.
  if (lastActiveTime + STANDBY_TIME_OUT < millis())
    if (mState != ST_BATTERY_LOW) mEvent = EV_STBY_TIMEOUT;
}

// Зчитує подію перемикання кнопкового перемикача поворотного енкодера.
void checkForBtnEvent() {
  static unsigned long lastBtnTime = 0;
  static boolean lastBtnState = B_UP;
  boolean thisBtnState;

  thisBtnState = btnState();

  // Ігноруйте зміни, які відбуваються протягом періоду дебаунсу.
  if (millis() - lastBtnTime > RS_DEBOUNCE) {

    // Реагувати лише на зміну стану кнопки.
    if (thisBtnState != lastBtnState) {
      // Згенерувати подію на основі поточного стану кнопки.
      mEvent = thisBtnState == B_UP ? EV_BTNUP : EV_BTNDN;

      lastActiveTime = lastBtnTime = millis();
      lastBtnState = thisBtnState;
    }
  }
}

/***************************************************************************************************
* Процедури звуку зумера                                                                           *
***************************************************************************************************/

// Процедури звукового сигналу.

void Boot_Sound() {
  playBeep(1000, 200);
  playBeep(2000, 200);
  playBeep(3000, 200);
  playBeep(2000, 200);
}

void LowBattery_Sound() {
  unsigned long previousMillis = 0;
  const long interval = 2000;

  while (btnState() != B_DN) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      playBeep(1000, 200);
    }
  }
}

void FootSwitch_Alarm() {
  while (1) {
    playBeep(1000, 200);
    delay(1000);
  }
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

/***************************************************************************************************
* Процедури керування меню                                                                         *
***************************************************************************************************/
/**
 *  \brief      Відображає на РК-дисплеї меню типу 1.
 *  \param [in] const __FlashStringHelper *title       Menu title.
 *  \param [in] const __FlashStringHelper *line1       Line 1 text.
 *  \param [in] const __FlashStringHelper *line2       Line 2 text.
 *  \param [in] const __FlashStringHelper *line3       Line 3 text.
 *  \param [in] uint16_t      SelectedItem Вибраний пункт меню.
 */
void displayMenuType1(const __FlashStringHelper *title,
                      const __FlashStringHelper *line1,
                      const __FlashStringHelper *line2,
                      const __FlashStringHelper *line3,
                      uint8_t SelectedItem) {

  display.clearDisplay();

  if (title == NULL)
    drawStatusLine();
  else {
    setTextProp(1, 1, 1, WHITE);
    display.print(title);
    display.drawLine(0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE);
  }

  setTextProp(2, 2, 16, WHITE, SelectedItem == 0);
  display.print(line1);
  setTextProp(2, 2, 16 + 2 * CHR_H + 1, WHITE, SelectedItem == 1);
  display.print(line2);
  setTextProp(2, 2, 16 + 4 * CHR_H + 2, WHITE, SelectedItem == 2);
  display.print(line3);
  display.drawRect(0, SelectedItem == 0 ? 16 : SelectedItem == 1 ? 16 + 2 * CHR_H + 1
                                                                 : 16 + 4 * CHR_H + 2,
                   2, 2 * CHR_H, WHITE);
  display.display();
}
/**
 *  \brief      Відображає на РК-дисплеї меню типу 2.
 *  \param [in] const __FlashStringHelper *title  Menu title.
 *  \param [in] const char                *value  Item value.
 *  \param [in] const __FlashStringHelper *units  Item units.
 */
void displayMenuType2(const __FlashStringHelper *title,
                      const char *value,
                      const __FlashStringHelper *units) {

  display.clearDisplay();

  setTextProp(1, 1, 1, WHITE);
  display.print(title);
  display.drawLine(0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE);
  setTextProp(2, 2, 16 + LINE_H);

  if (value == NULL) display.print(units);
  else {
    display.print(value);
    drawText(1, 2, 16 + 3 * LINE_H, units);
  }

  display.display();
}

/***************************************************************************************************
* Процедури керування дисплеєм                                                                      *
***************************************************************************************************/
/**
 *  \brief                      Задає властивості текстового рядка, який буде намальовано на екрані.
 *  \param [in] uint16_t  size   Розмір тексту.
 *  \param [in] uint16_t  xpos   Текстова x-координата в пікселях зліва.
 *  \param [in] uint16_t  ypos   Текстова x-координата в пікселях зверху.
 *  \param [in] uint16_t color  Колір переднього плану тексту (за замовчуванням БІЛИЙ).
 *  \param [in] boolean  invert True, якщо кольори мають бути інвертовані (bg=color,fg=BLACK), за замовчуванням false.
 */
void setTextProp(uint8_t size, uint8_t xpos, uint8_t ypos, uint16_t color, boolean invert) {

  display.setTextSize(size);
  display.setCursor(xpos, ypos);

  if (invert)
    display.setTextColor(BLACK, color);
  else
    display.setTextColor(color);
}

// Малює рядок стану на РК-дисплеї.
void drawStatusLine() {
  char str[16];

  drawText(1, 0, 1, (pData.pFlags.en_autoPulse ? FPSTR(LS_AUTO_BAR) : FPSTR(LS_MANUAL_BAR)));
  uint16_t batteryAmphere = abs(INA.getCurrent_mA());
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 14, 1, valStr(str, batteryAmphere / 10, VF_BATTV), FPSTR(LS_AUNITS));

  // Малюємо лінію під статусною строкою
  display.drawLine(0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE);

  // Відображення середньої напруги

  batteryVoltage = (INA.getBusVoltage_mV());  // Читаємо напругу з модуля INA226
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 6, 1, valStr(str, (abs(batteryVoltage)) / 10, VF_BATTV), FPSTR(LS_VUNITS));
}

// Малюємо головний екран на РК-дисплеї.
void displayMainScreen(bool signaled) {
  char str[16];

  display.clearDisplay();
  drawStatusLine();

  // Запишіть поточне значення тривалості зварювального імпульсу та одиниці виміру.
  setTextProp(4, 0, 16 + CHR_H / 2);
  display.print(valStr(str, pData.pulseTime, VF_PLSDLY));
  setTextProp(2, 12 * CHR_W, 32, WHITE, signaled);
  display.print(FPSTR(LS_MS));

  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 6, 20, valStr(str, pData.weldCount, VF_WELDCNT), FPSTR(LS_WELDS));

  // Буферні змінні для значень
  char pulseAmpsStr[8];

  // Відображення на РК-дисплеї повідомлення про імпульсну напругу та напругу акумулятора, виміряну під час імпульсу.
  drawText(1, 0, 56, FPSTR(LS_PULSE));
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 14, 56, valStr(pulseAmpsStr, pData.PulseAmps, VF_BATTA), FPSTR(LS_AUNITS));
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 6, 56, valStr(str, pData.PulseBatteryVoltage / 10, VF_BATTV), FPSTR(LS_VUNITS));

  display.display();
}

void displayMainScreen() {
  displayMainScreen(false);
}

void drawText(uint8_t size, int16_t x, int16_t y, const __FlashStringHelper *text) {
  setTextProp(size, x, y);
  display.print(text);
}

void drawValueWithUnits(uint8_t size, int16_t x, int16_t y, const char *value, const __FlashStringHelper *units) {
  setTextProp(size, x, y);
  display.print(value);
  display.print(units);
}

// Відобразити на РК-дисплеї повідомлення НИЗЬКИЙ РІВЕНЬ напруги АККУМУЛЯТОРА.
void displayBatteryStatus(const __FlashStringHelper *statusText, const char */*value*/) {
  char str[5];

  display.clearDisplay();
  drawStatusLine();

  // Заголовок "Battery"
  drawText(2, (SSD1306_LCDWIDTH - (sizeof(LS_BATTERY) - 1) * CHR_W * 2) / 2, 16, FPSTR(LS_BATTERY));

  // Статус напруги
  drawText(2, (SSD1306_LCDWIDTH - (sizeof(statusText) - 1) * CHR_W * 2) / 2, 16 + 2 * LINE_H, statusText);

  // Значення напруги
  drawValueWithUnits(1, (SSD1306_LCDWIDTH - 1 * CHR_W) / 2, 16 + 4 * LINE_H, valStr(str, batteryVoltage, VF_BATTV), FPSTR(LS_VUNITS));

  display.display();
}

// Відобразити на РК-дисплеї повідомлення про РІВЕНЬ та СТАТУС ТЕМПЕРАТУРИ.
void displayTemperatureStatus(const __FlashStringHelper *statusText, const __FlashStringHelper *adviceText) {
  char str[5];

  display.clearDisplay();
  drawStatusLine();

  // Значення температури
  drawValueWithUnits(2, (SSD1306_LCDWIDTH - (sizeof(LS_HIGHT) - 1) * CHR_W * 2) / 2, 16, valStr(str, TCelsius, VF_TEMP), FPSTR(LS_TUNITS));

  // Статус температури
  drawText(2, (SSD1306_LCDWIDTH - (sizeof(LS_HIGHT) - 1) * CHR_W * 2) / 2, 16 + 2 * LINE_H, statusText);

  // Порада
  drawText(1, (SSD1306_LCDWIDTH - (sizeof(adviceText) - 1) * CHR_W) / 2, 16 + 4 * LINE_H, adviceText);

  display.display();
}

void displayLowBattery() {
  char str[16];
  displayBatteryStatus(FPSTR(LS_LOWV), valStr(str, batteryVoltage, VF_BATTV));
}

void displayHighBattery() {
  char str[16];
  displayBatteryStatus(FPSTR(LS_HIGHV), valStr(str, batteryVoltage, VF_BATTV));
}

void displayHighTemperature() {
  displayTemperatureStatus(FPSTR(LS_HIGHT), FPSTR(LS_COOL));
}

/**
 *  \brief    Відобразити екран ПОВІДОМЛЕННЯ.
 *  \param [in] const __FlashStringHelper* line1        Line 1 text.
 *  \param [in] const __FlashStringHelper* line1        Line 2 text.
 *  \param [in] const __FlashStringHelper* line3        Line 3 text.
 *  \param [in] uint16_t                    displayTime  Delay time (default = 0)
 */
void message(const __FlashStringHelper *line1,
             const __FlashStringHelper *line2,
             const __FlashStringHelper *line3,
             uint8_t displayTime) {

  display.clearDisplay();

  drawText(1, (SSD1306_LCDWIDTH - (sizeof(LS_MSGHDR) - 1) * CHR_W) / 2, 1, FPSTR(LS_MSGHDR));
  display.drawLine(0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE);

  drawText(2, 1, 16, line1);
  drawText(1, 1, 16 + 2 * LINE_H, line2);
  drawText(1, 1, 16 + 3 * LINE_H, line3);

  display.display();

  if (displayTime) delay(displayTime * 1000);
}

//  Відобразити екран SPLASH.
void splash() {
  display.clearDisplay();
  display.display();

  drawText(1, 1, 1, F(_DEVICENAME_));
  drawText(1, 1, 16, F("Ver " xstr(_VERSION_MAJOR_) "." xstr(_VERSION_MINOR_) "." xstr(_REVISION_) " " __DATE__));
  drawText(1, 1, 16 + 2 * LINE_H, F("Copyright (c) " _COPYRIGHT_));

  display.display();

  uint16_t timer = 0;

  while (btnState() != B_DN && timer < SPLASHTIME) {
    delay(10);
    timer += 10;
  }

  while (btnState() == B_DN)
    continue;

  display.clearDisplay();
  display.display();
}

//  Повідомлення "FOOT SWITCH ERROR!".
void foot_switch_error() {
  display.clearDisplay();
  display.display();

  drawText(1, 1, 1, F("FOOT SWITCH ERROR!"));
  drawText(1, 1, 16, F("Please:"));
  drawText(1, 1, 16 + LINE_H, F("- turn off welder"));
  drawText(1, 1, 16 + 2 * LINE_H, F("- remove foot switch"));
  drawText(1, 1, 16 + 3 * LINE_H, F("- correct the wiring"));

  display.display();

  while (!digitalRead(PIN_FOOT_SWITCH))
    ;
}

/***************************************************************************************************
* Утиліта перетворення функцій                                                                     *
***************************************************************************************************/
/**
 *  \brief                    Повертає символьний рядок, що представляє відформатоване числове значення.
 *  \param [in] char *str     Вказівник на рядок для отримання відформатованого результату.
 *  \param [in] uint16_t val  Ціле значення, яке потрібно відформатувати.
 *  \param [in] vf_Type fType Тип змінної для форматування.
 *  \return     char *        Вказівник на відформатований рядок.
 */
char *valStr(char *str, uint16_t val, vf_Type fType) {

  // Ми повинні вдатися до цієї незручної дурниці, щоб відформатувати змінні, тому що змінна ширина і
  // точність (* специфікатор) не реалізовано в avr gcc - помилка!!!

  switch (fType) {
    case VF_BATTALM: sprintf_P(str, PSTR("%1.1u.%01u"), val / 10, val % 10); break;
    case VF_TEMPALM: sprintf_P(str, PSTR("%4.1u"), val); break;
    case VF_BATTV: sprintf_P(str, PSTR("%2.1u.%02u"), val / 100, val % 100); break;
    case VF_BATTA: sprintf_P(str, PSTR("%5.1u"), val); break;
    case VF_WELDCNT: sprintf_P(str, PSTR("%5.1u"), val); break;
    case VF_TEMP: sprintf_P(str, PSTR("%5.1u"), val); break;
    case VF_PLSDLY: sprintf_P(str, PSTR("%3.1u"), val); break;  // pData.pulseTime
    case VF_SHTPLS: sprintf_P(str, PSTR("%3.1u"), val); break;
    case VF_DELAY: sprintf_P(str, PSTR("%1.1u.%01u"), val / 10, val % 10); break;
  }

  return str;
}

/***************************************************************************************************
* Утиліти EEPROM функцій                                                                           *
***************************************************************************************************/
/**
 *  \brief                    Скиньте EEPROM і програмні дані до заводських налаштувань за замовчуванням.
 *  \param [in] boolean full  True для скидання кількості зварювальних швів, зміщення акумулятора та інверсії екрана.
 */
void resetEEPROM(boolean full) {

  // Повністю видаліть всі дані з EEPROM
  for (size_t i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }

  // Запис заводських даних у EEPROM. У разі повного скидання кількість зварних швів
  // та орієнтація екрана обнуляються, інакше вони залишаються незмінними.
  pData.autoPulseDelay = DEF_AUTO_PLSDELAY;
  pData.PulseBatteryVoltage = DEF_PULSE_VOLTAGE;
  pData.PulseAmps = DEF_PULSE_AMPS;
  pData.PulseShuntVoltage = DEF_SHUNT_VOLTAGE;
  pData.batteryAlarm = DEF_BATT_ALARM / 100;
  pData.batteryhighAlarm = DEF_HIGH_BATT_ALARM;
  pData.weldCount = full == EE_FULL_RESET ? 0 : pData.weldCount;
  pData.pulseTime = DEF_PULSE_TIME;
  pData.maxPulseTime = DEF_MAX_PULSE_TIME;
  pData.shortPulseTime = DEF_SPULSE_TIME;
  pData.pFlags.en_autoPulse = DEF_AUTO_PULSE;
  pData.pFlags.en_Sound = DEF_WELD_SOUND;
  pData.pFlags.en_oledInvert = full ? DEF_OLED_INVERT : pData.pFlags.en_oledInvert;

  // Функція put не записує нові дані, якщо таким чином існуючі дані збігаються
  // обмеження зносу EEPROM.
  EEPROM.put(EEA_PDATA, pData);  // EEPROM.put(EEA_PDATA, pData);

  // 1. Обчисліть хеш-суму для заводських даних
  uint16_t checksum = calculateChecksum(pData);

  // 2. Запишіть обчислену хеш-суму в EEPROM
  EEPROM.put(EEA_CHECKSUM, checksum);

  // Унікальний ідентифікатор — це простий спосіб переконатися, що в EEPROM існує дійсний набір даних
  // (є набагато кращі методи, але ми не маємо вільного місця для коду).
  EEPROM.put(EEA_ID, EE_UNIQUEID);

#if defined _DEVELOPMENT_ || defined _BOOTSYS_

  if (full) Serial.println(F("Повне скидання EEPROM з контрольною сумою"));
  else Serial.println(F("Скидання EEPROM з контрольною сумою"));

#endif /* _DEVELOPMENT_ || _BOOTSYS_*/
}

void playBeep(uint16_t frequency, uint16_t duration) {
  tone(PIN_BUZZ, frequency, duration);  // Генерує звук
  delay(duration / 2);                  // Очікування завершення
  noTone(PIN_BUZZ);                     // Вимикає тон
}

void reboot() {
  wdt_enable(WDTO_15MS);
  while (1)
    ;
}

// Логування інформації про імпульс
void logPulseInfo() {
  // #ifdef _DEVELOPMENT_
  //   Serial.print(F("Read: tCT Bit: "));
  //   Serial.print(INA.getShuntVoltageConversionTime());
  //   Serial.print(F("  | Pulse Time: "));
  //   Serial.println(pData.pulseTime);
  // #endif /* _DEVELOPMENT_ */
}

void loadEEPROM() {
  // Перевірте цілісність EEPROM за допомогою хеш-суми.
  uint16_t storedChecksum;
  uint16_t calculatedChecksum;
  uint32_t uniqueID;

  // 1. Зчитайте збережену хеш-суму з EEPROM
  EEPROM.get(EEA_CHECKSUM, storedChecksum);

  // 2. Зчитайте структуру даних програми з EEPROM
  EEPROM.get(EEA_PDATA, pData);

  EEPROM.get(EEA_ID, uniqueID);

  // 3. Обчисліть хеш-суму для щойно зчитаних даних
  calculatedChecksum = calculateChecksum(pData);

  // 4. Порівняйте обчислену хеш-суму зі збереженою та магічне число.
  if (calculatedChecksum != storedChecksum || uniqueID != EE_UNIQUEID) {
#ifdef _DEVELOPMENT_
    Serial.print(F("EEPROM Оновлено! Хеш-сума змінилась (HEX) | 0x"));
    Serial.print(storedChecksum, HEX);  // Вивід в шістнадцятковому форматі з префіксом "0x"
    Serial.print(F(" >>> 0x"));
    Serial.println(calculatedChecksum, HEX);
#endif
    // Хеш-суми не збігаються, EEPROM дані пошкоджені або недійсні
    Serial.println(F("EEPROM Error!  Відновлення EEPROM >>>"));
    resetEEPROM(EE_FULL_RESET);  // Скидання до заводських налаштувань
  } else {
    EEPROM.get(EEA_PDATA, pData);
// Хеш-суми збігаються, дані EEPROM, ймовірно, цілісні
#ifdef _DEVELOPMENT_
    Serial.print(F("EEPROM OK!       Хеш-сума (HEX) = 0x"));
    Serial.println(calculatedChecksum, HEX);  // Вивід в шістнадцятковому форматі з префіксом "0x"
#endif
  }
}

// Оновлюємо дані EEPROM локальною структурою даних програми.
void updateEEPROM() {
  static unsigned long lastEEUpdatetime = 0;
  uint16_t storedChecksum;

  // Не робіть це занадто часто, щоб запобігти передчасному зносу EEPROM.
  if (millis() - lastEEUpdatetime > EEPROM_UPDATE_T) {
    lastEEUpdatetime = millis();

    // 1. Обчисліть хеш-суму для поточних даних програми
    EEPROM.get(EEA_CHECKSUM, storedChecksum);
    uint16_t calculatedChecksum = calculateChecksum(pData);
    if (calculatedChecksum != storedChecksum) {

      // 2. Запишіть поточні дані програми в EEPROM
      EEPROM.put(EEA_PDATA, pData);  // EEPROM.put(EEA_PDATA, pData);

      // 3. Запишіть обчислену хеш-суму в EEPROM
      EEPROM.put(EEA_CHECKSUM, calculatedChecksum);  // EEPROM.put(EEA_CHECKSUM, checksum);

#ifdef _DEVELOPMENT_
      Serial.print(F("EEPROM Оновлено! Хеш-сума змінилась (HEX) | 0x"));
      Serial.print(storedChecksum, HEX);  // Вивід в шістнадцятковому форматі з префіксом "0x"
      Serial.print(F(" >>> 0x"));
      Serial.println(calculatedChecksum, HEX);
#endif

    } else {
#ifdef _DEVELOPMENT_
      Serial.print(F("EEPROM OK!       Хеш-сума не змінилась (HEX) = 0x"));
      Serial.println(calculatedChecksum, HEX);  // Вивід в шістнадцятковому форматі з префіксом "0x"
#endif                                          /* _DEVELOPMENT_ */
    }
  }
}

// Функція для обчислення простої хеш-суми структури progData
uint16_t calculateChecksum(const progData &data) {
  uint16_t checksum = 0;
  const byte *dataPtr = reinterpret_cast<const byte *>(&data);
  size_t dataSize = sizeof(progData);

  for (size_t i = 0; i < dataSize; ++i) {
    checksum += dataPtr[i];  // Проста адитивна хеш-сума
  }
  return checksum;
}
