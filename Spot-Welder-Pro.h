#ifndef _SPOT_WELDER_PRO_H
#define _SPOT_WELDER_PRO_H

// Загальні макроси
#define str(s) #s
#define xstr(s) str(s)

// Більшість з цих макросів визначають рядки, які з'являються на заставці
// (кожен рядок заставки обмежений 21 символом)
#define _DEVICENAME_ "Arduino Spot Welder"
#define _PROGNAME_ "Arduino Spot Welder Control Firmware"
#define _VERSION_MAJOR_ 4
#define _VERSION_MINOR_ 0
#define _REVISION_ 3
#define _COPYRIGHT_ "2024"

/***************************************************************************************************
* Конфігурація користувача                                                                         *
***************************************************************************************************/

#define _DEVELOPMENT_  // Дозволяє друкувати діагностику
// #define _BOOTSYS_  // Примусове завантаження в системне меню для тестування

#define _LANG_EN_  // Language:  _LANG_EN/DE/FR/ES/IT_
// #define _TESTING_  // Дозволяє ігнорувати тривогу про низький заряд батареї
#define _SERIAL_BAUD_ 115200  // Швидкість зв'язку для послідовного налагодження

/***************************************************************************************************
* Визначення виводів та переривань                                                                 *
***************************************************************************************************/

#define ENC_uint16_t 0     // Поворотне переривання для входу CLK (Ph0)
#define PIN_CLK 2          // Вхід CLK датчика повороту (Ph0)
#define PIN_DT 8           // Поворотний енкодер DT вхід (Ph1)
#define PIN_SW 6           // Поворотний енкодер кнопковий перемикач вхід
#define PIN_FOOT_SWITCH 7  // Вхід сигналу ножного перемикача
#define PIN_AUTO_PULSE 3   // Автоімпульсний сенсорний вхід
#define PIN_PULSE 5        // Вихід зварювального імпульсу
#define PIN_BUZZ A1        // Вихід звукового сигналу
#define PIN_TEMP A3        // Вхідний сигнал температури

/***************************************************************************************************
* Макроси                                                                                          *
***************************************************************************************************/
// Значення за замовчуванням для операційних змінних
#define DEF_AUTO_PLSDELAY 20  // Час автоматичної затримки імпульсу за замовчуванням (мс*100)
#define DEF_PULSE_TIME 5      // Час імпульсу за замовчуванням (мс)
#define DEF_SHUNT_VOLTAGE 0
#define DEF_MAX_PULSE_TIME 100    // За замовчуванням максимальний час імпульсу (мс)
#define DEF_SPULSE_TIME 0         // Час короткого імпульсу за замовчуванням (% від часу імпульсу)
#define DEF_NOM_BATT_V 5400       // Номінальна напруга акумулятора за замовчуванням (для тестування)
#define DEF_MAX_BATT_V 5400       // Максимальна напруга батареї за замовчуванням (В*1000)
#define DEF_PULSE_VOLTAGE 4000    // Напруга за замовчуванням під час імпульсу (для тестування) (В*1000)
#define DEF_PULSE_AMPS 5000       // За замовчуванням Ампер під час імпульсу (для тестування) (А*1000)
#define DEF_BATTERY_OFFSET 0      // Зміщення калібрування батареї за замовчуванням (V*1000)
#define DEF_BATT_ALARM 3500       // За замовчуванням низька напруга батареї (В*1000)
#define DEF_HIGH_BATT_ALARM 5500  // За замовчуванням висока напруга акумулятора (В*1000)
#define DEF_HIGH_TEMP_ALARM 65    // Висока температура за замовчуванням | Off in >>> if (TCelsius > DEF_HIGH_TEMP_ALARM) mEvent = EV_NONE;  // EV_TEMP_HIGH;
#define DEF_AUTO_PULSE true       // За замовчуванням Увімкнення автоімпульсу
#define DEF_WELD_SOUND true       // За замовчуванням звук зварювання ввімкнено
#define DEF_OLED_INVERT false     // Орієнтація OLED за замовчуванням

// Обмеження для операційних змінних
#define MIN_PULSE_TIME 1      // Мінімальний час зварювального імпульсу
#define MAX_PULSE_TIME 100    // Абсолютний максимальний час зварювального імпульсу
#define MAX_APULSE_DELAY 50   // Максимальна автоматична затримка імпульсу
#define MIN_APULSE_DELAY 5    // Мінімальна затримка автоматичного імпульсу
#define MAX_SPULSE_TIME 100   // Максимальний час короткого імпульсу
#define MIN_SPULSE_TIME 0     // Мінімальний час короткого імпульсу
#define MAX_BATT_ALARM 5400   // Максимальна тривожна напруга низького рівня заряду акумулятора
#define MIN_BATT_BALARM 3500  // Мінімальна напруга тривоги низького рівня заряду акумулятора

// Макроси синхронізації
#define STANDBY_TIME_OUT 640000L  // Тайм-аут сплячого режиму пристрою (мс)  // 6400000L
#define EEPROM_UPDATE_T 30000     // Час оновлення EEPROM (мс) // 60000 = 60 секунд
#define WP_RETRIGGER_DELAY 50     // Затримка повторного запуску зварювального імпульсу (мс /10)
#define FS_TRIGGER_DELAY 200      // Затримка активації ножного перемикача (мс)
#define RS_DEBOUNCE 20 /*20*/     // Час відскоку поворотного енкодера та перемикача (мс)
#define BV_INTERVAL 2000          // Інтервал вимірювання напруги акумулятора (мс)
#define T_INTERVAL 10000          // Інтервал вимірювання температури (мс)
#define PV_DELAY 1000             // Час, протягом якого потрібно утримувати ножний перемикач,
// перш ніж дані про імпульс з'являться на екрані після імпульсу (мс)

// Макет екрана дисплея
#define CHR_W 6             // Ширина символу [size=1] (пікселів)
#define CHR_H 8             // Висота символу [size=1] (пікселів)
#define LINE_H (CHR_H + 2)  // Висота рядка [size=1] (пікселів)

// Макроси для визначення логічних станів
#define DD_READ true       // Напрямок передачі даних - читання
#define DD_WRITE false     // Напрямок передачі даних - запис
#define Pulse_ON true      // Загальний макрос для стану Pulse ON
#define Pulse_OFF false    // Загальний макрос для стану Pulse OFF
#define B_DN true          // Загальний макрос для стану DOWN
#define B_UP false         // Загальний макрос для стану UP
#define PL_ACTIVE_H false  // Макрос логіки виводу для активного високого рівня
#define PL_ACTIVE_L true   // Макрос логіки виводу для активного низького рівня

// EEPROM macros
#define EEA_ID 0                // Адреса унікального ідентифікатора
#define EEA_PDATA (EEA_ID + 4)  // EEPROM-адреса програмних даних
#define EEA_CHECKSUM (EEA_PDATA + sizeof(progData)) // Адреса для хеш-суми, розміщуємо після структури progData
#define EE_UNIQUEID 0x18fae9c8  // Унікальний ідентифікатор верифікації EEPROM
#define EE_FULL_RESET true      // Параметр скидання для скидання всіх параметрів EEPROM
#define EE_CHECKSUM_SIZE sizeof(uint16_t) // Розмір хеш-суми (uint16_t - 2 байти)

// Маскування макросів під функції - робить код більш читабельним
// Цей макрос зчитує стан кнопкового перемикача на енкодері.
#define btnState() (!digitalRead(PIN_SW))

// Цей макрос керує зварювальним імпульсом.
#define weldPulse(state) digitalWrite(PIN_PULSE, state ? HIGH : LOW)

// Куди подівся цей макрос? Він був у WString.h
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))

/***************************************************************************************************
* OLED Display Configuration                                                                       *
***************************************************************************************************/

#define SCREEN_WIDTH 128  // Ширина OLED-дисплея, в пікселях
#define SCREEN_HEIGHT 64  // Висота OLED-дисплея, в пікселях

#define OLED_RESET 4    // Режим скидання OLED-дисплея
#define OLED_INVERT 2   // Режим орієнтації, визначений OLED, - перевірте документацію до OLED
#define SPLASHTIME 255  // Час затримки заставки на екрані (мс)


/***************************************************************************************************
* Визначення структури, об'єднання та перелічених типів                                            *
***************************************************************************************************/

typedef enum {  // Перерахування типів для формату змінних
  VF_BATTALM,   // Напруга тривоги акумулятора
  VF_TEMPALM,   // Температура Значення тривоги
  VF_BATTV,     // Напруга акумулятора
  VF_BATTA,     // Акумуляторна батарея Ампери
  VF_TEMP,      // Температура
  VF_WELDCNT,   // Кількість зварних швів
  VF_PLSDLY,    // Затримка імпульсу
  VF_SHTPLS,    // Коротка тривалість імпульсу
  VF_DELAY      // Затримка.
} vf_Type;

typedef struct progData {        // Структура робочих даних програми
  uint8_t autoPulseDelay;        // Затримка автоімпульсу (мс/100)
  uint16_t batteryAlarm;         // Низька напруга акумулятора (кількість A/D)
  uint16_t batteryhighAlarm;     // Висока напруга акумулятора (кількість A/D)
  uint8_t TCelsius;              // Температура в градусах Цельсія
  uint8_t maxTCelsius;           // Максимальна температура в градусах Цельсія
  uint16_t weldCount;            // Кількість виконаних зварних швів
  uint16_t pulseTime;            // Час імпульсу (мс)
  uint16_t maxPulseTime;         // Максимально допустимий час імпульсу (мс)
  uint8_t shortPulseTime;        // Короткий час імпульсу (% від часу імпульсу)
  uint16_t batteryOffset;        // Зміщення калібрування напруги акумулятора (з підписом) x10
  uint16_t PulseBatteryVoltage;  // Напруга батареї під час імпульсу x10
  uint16_t PulseAmps;            // Орієнтовна сила струму під час імпульсу x10
  uint16_t PulseShuntVoltage;
  struct progFlags {             // Програмування логічних прапорів
    unsigned en_autoPulse : 1;   // Auto-pulse enable
    unsigned en_Sound : 1;       // Weld Sound enable
    unsigned en_oledInvert : 1;  // Орієнтація OLED - істинна для перевернутого, інакше нормальна
    unsigned unused : 6;         // Невикористані прапори програми
  } pFlags;
} progData;

/***************************************************************************************************
* Прототипи процедур                                                                               *
***************************************************************************************************/

void stateMachine();

void resetEeprom(boolean = false);
void loadEeprom();
void updateEeprom();

void checkForLowVoltageEvent();
void checkForSleepEvent();
void checkForBtnEvent();
void checkTemp();
void foot_switch_error();
void FootSwitch_Alarm();
void Boot_Sound();
void LowBattery_Sound();
void isr();
void splash();
void sendWeldPulse(uint8_t, uint16_t, uint16_t, boolean = PL_ACTIVE_H);
void message(const __FlashStringHelper *, const __FlashStringHelper *,
             const __FlashStringHelper *, uint8_t = 0);
void displayMenuType1(const __FlashStringHelper *, const __FlashStringHelper *,
                      const __FlashStringHelper *, const __FlashStringHelper *,
                      uint8_t SelectedItem);
void displayMenuType2(const __FlashStringHelper *, const char *, const __FlashStringHelper *);
void displayMainScreen();
void displayPulseData();
void displayLowBattery();
void displayHighBattery();
void displayHighTemperature();
void drawStatusLine();
void setTextProp(uint8_t, uint8_t, uint8_t, uint16_t = WHITE, boolean = false);
char *valStr(char *, uint16_t, vf_Type);

void enterMainScreen();
void handleMainScreenCnt();
void enterSystemScreen();
void handleSystemMenu(char *str);
void handleRebootMenu(char *str);
void handleMenuItem(uint8_t menuIndex, uint16_t &setting, uint16_t minValue, uint16_t maxValue, const char *title, const char *unit);
void handleSubMenu2(char *str);

/***************************************************************************************************
* Мовні рядки (дуже проста мовна реалізація - англійська за замовчуванням)                         *
***************************************************************************************************/

// Скопіюйте мовні рядки з else у відповідний пункт, а потім змініть їх відповідно до вашої мови.
// Визначте вашу мову у верхній частині цього заголовного файлу.
// Важливо підтримувати правильне форматування рядків. Кожен рядок має вбудований коментар,
// який визначає його формат.

// Comment legend: field width          21 для дрібного шрифту, 10 для великого шрифту
//                 justification        left, centre, right
//                 padded               Пробіли з обох кінців для заповнення поля

#ifdef _LANG_DE_

#elif defined _LANG_FR_

#elif defined _LANG_ES_

#elif defined _LANG_IT_

#else
//                                           0123456789               // large font
//                                           012345678901234567890    // small font

static const char LS_APULSE[] PROGMEM = "Pulse Set";      // 10 char, centre, padded
static const char LS_BATTALM1[] PROGMEM = "Batt Alarm";   // 10 char, centre, padded
static const char LS_SHORTPLS1[] PROGMEM = "Shrt Pulse";  // 10 char, centre, padded

static const char LS_MANAUTO[] PROGMEM = "  Mode    ";    // 10 char, centre, padded
static const char LS_DELAY[] PROGMEM = "  Delay   ";      // 10 char, centre, padded
static const char LS_WELDSOUND[] PROGMEM = "  Sound   ";  // 10 char, centre, padded
static const char LS_EXIT[] PROGMEM = "  Exit    ";       // 10 char, centre, padded

static const char LS_STANDBY[] PROGMEM = "  STANDBY ";             // 10 char, centre, padded
static const char LS_CLICKBTN[] PROGMEM = " Please Click Button";  // 21 char, left
static const char LS_EXITSTBY[] PROGMEM = "   to Exit Standby";    // 21 char, left

static const char LS_BATTALM[] PROGMEM = "Low Battery Alarm";  // 21 char, left
static const char LS_BATTERY[] PROGMEM = "BATTERY";            // 10 char, left
static const char LS_LOWV[] PROGMEM = "LOW VOLTS";             // 10 char, left
static const char LS_HIGHV[] PROGMEM = "HIGH VOLTS";           // 10 char, left
static const char LS_PULSE[] PROGMEM = "Pulse:";               // 21 char, left

static const char LS_TEMPALM[] PROGMEM = "Temperature Alarm";  // 21 char, left
static const char LS_TEMP[] PROGMEM = "TEMP";                  // 10 char, left
static const char LS_HIGHT[] PROGMEM = "HIGH TEMP";            // 10 char, left
static const char LS_CEL[] PROGMEM = "TEMP IN CELSIUS";        // 21 char, left
static const char LS_COOL[] PROGMEM = "PLEASE COOL DOWN";      // 21 char, left

static const char LS_AUTOPLSON[] PROGMEM = "Weld Pulse Activation";  // 21 char, left
static const char LS_AUTO[] PROGMEM = "Auto Pulse";                  // 10 char, left
static const char LS_MANUAL[] PROGMEM = "Manual";                    // 10 char, left
static const char LS_AUTO_BAR[] PROGMEM = "Auto";                    // 10 char, left
static const char LS_MANUAL_BAR[] PROGMEM = "Manual";                // 10 char, left

static const char LS_AUTOPLSDLY[] PROGMEM = "Auto Pulse Delay";    // 21 char, left
static const char LS_SHORTPLS[] PROGMEM = "Short Pulse Duration";  // 21 char, left
static const char LS_WPDRN[] PROGMEM = "Weld Pulse Duration";      // 21 char, left

static const char LS_WELDSOUNDM[] PROGMEM = "Weld Pulse Sound";  // 21 char, left
static const char LS_SOUNDON[] PROGMEM = "ON";                   // 10 char, left
static const char LS_SOUNDOFF[] PROGMEM = "OFF";                 // 10 char, lef

static const char LS_BATTMSG[] PROGMEM = " Battery";         // 10 char, centre
static const char LS_MAXPMSG[] PROGMEM = "   Duration Set";  // 21 char, centre

static const char LS_PCOF[] PROGMEM = "% of Pulse Time";  // 21 char, left
static const char LS_SECONDS[] PROGMEM = "Seconds";       // 21 char, left
static const char LS_VOLTAGE[] PROGMEM = "Volts";         // 21 char, left
static const char LS_MS[] PROGMEM = "ms";                 // 2  char, left
static const char LS_VUNITS[] PROGMEM = "V";              // 1  char, left
static const char LS_AUNITS[] PROGMEM = "A";              // 1  char, left
static const char LS_WELDS[] PROGMEM = "W";               // 1  char, left
static const char LS_TUNITS[] PROGMEM = "C";              // 2  char, left


static const char LS_REBOOTFR[] PROGMEM = "   with Full Reset";   // 21 char, centre
static const char LS_REBOOTNR[] PROGMEM = "   without Reset";     // 21 char, centre
static const char LS_REBOOTSR[] PROGMEM = "   with Safe Reset";   // 21 char, centre
static const char LS_RECALMSG[] PROGMEM = "   re-calibrated";     // 21 char, centre
static const char LS_WAITMSG[] PROGMEM = " PLEASE REMOVE POWER";  // 21 char, centre

static const char LS_SYSMENU[] PROGMEM = "System Menu";  // 21 char, left
static const char LS_SETTINGS[] PROGMEM = " Settings ";  // 10 char, centre, padded
static const char LS_DISPLAY[] PROGMEM = "  Display ";   // 10 char, centre, padded
static const char LS_BOOT[] PROGMEM = "   Boot   ";      // 10 char, centre, padded

static const char LS_SETTMENU[] PROGMEM = "System Settings";  // 21 char, left
static const char LS_MAXPULSE[] PROGMEM = "Max Pulse ";       // 10 char, centre, padded

static const char LS_BOOTMENU[] PROGMEM = "Reboot Spot Welder";  // 21 char, left
static const char LS_REBOOT[] PROGMEM = "  Reboot  ";            // 10 char, centre, padded
static const char LS_SAFERST[] PROGMEM = "Safe Reset";           // 10 char, centre, padded
static const char LS_FULLRST[] PROGMEM = "Full Reset";           // 10 char, centre, padded

static const char LS_INVERTMENU[] PROGMEM = "Screen Orientation";  // 21 char, left
static const char LS_SCRNORM[] PROGMEM = "NORMAL";                 // 10 char, left
static const char LS_SCRINV[] PROGMEM = "INVERTED";                // 10 char, left

static const char LS_MAXPLSMENU[] PROGMEM = "Set Max Weld Pulse";  // 21 char, left

static const char LS_BATCALMENU[] PROGMEM = "Calibrate Battery";  // 21 char, left
static const char LS_BATCALMSG[] PROGMEM = "Set Measured Volts";  // 21 char, left
static const char LS_MSGHDR[] PROGMEM = "System Message";         // 21 char, left

#endif

#endif  // _SPOT_WELDER_PRO_H

// EOF Spot_Welder_Pro.h
