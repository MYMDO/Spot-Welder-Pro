#ifndef _SPOT_WELDER_PRO_H
#define _SPOT_WELDER_PRO_H

// General macros
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
* User Configuration                                                                               *
***************************************************************************************************/

// #define _DEVELOPMENT_ /**< Дозволяє друкувати діагностику */
// #define _BOOTSYS_                    /**< Примусове завантаження в системне меню для тестування */

#define _LANG_EN_            /**< Language:  _LANG_EN/DE/FR/ES/IT_ */
// #define _TESTING_            /**< Дозволяє ігнорувати тривогу про низький заряд батареї */
#define _SERIAL_BAUD_ 115200 /**< Швидкість зв'язку для послідовного налагодження */

/***************************************************************************************************
* Pin and interrupt definitions                                                                    *
***************************************************************************************************/

#define ENC_uint16_t 0    /**< Rotary interrupt for CLK input (Ph0) */
#define PIN_CLK 2         /**< Rotary encoder CLK input (Ph0) */
#define PIN_DT 8          /**< Rotary encoder DT input (Ph1) */
#define PIN_SW 6          /**< Rotary encoder push button switch input */
#define PIN_FOOT_SWITCH 7 /**< Foot switch sense input */
#define PIN_AUTO_PULSE 3  /**< Auto-pulse sense input */
#define PIN_PULSE 5       /**< Weld pulse output */
#define PIN_BUZZ A1       /**< Buzzer output */
#define PIN_TEMP A3       /**< Buzzer output */

/***************************************************************************************************
* Macros                                                                                           *
***************************************************************************************************/
// Значення за замовчуванням для операційних змінних
#define DEF_AUTO_PLSDELAY 20      /**< Default auto pulse delay time (ms*100) */
#define DEF_PULSE_TIME 5          /**< Default pulse time (ms) */
#define DEF_SHUNT_VOLTAGE 0
#define DEF_MAX_PULSE_TIME 100    /**< Default maximum pulse time (ms) */
#define DEF_SPULSE_TIME 0         /**< Default short pulse time (% of pulse time) */
#define DEF_NOM_BATT_V 5400       /**< Default nominal battery voltage (for testing) */
#define DEF_MAX_BATT_V 5400       /**< Default maximum battery voltage (V*1000) */
#define DEF_PULSE_VOLTAGE 4000    /**< Default voltage during pulse (for testing) (V*1000) */
#define DEF_PULSE_AMPS 5000       /**< Default Amps during pulse (for testing) (A*1000) */
#define DEF_BATTERY_OFFSET 0      /**< Default battery calibration offset (V*1000) */
#define DEF_BATT_ALARM 3500       /**< Default low battery voltage (V*1000) */
#define DEF_HIGH_BATT_ALARM 5500  /**< Default high battery voltage (V*1000) */
#define DEF_HIGH_TEMP_ALARM 65    /**< Default high temperature | Off in >>> if (TCelsius > DEF_HIGH_TEMP_ALARM) mEvent = EV_NONE;  // EV_TEMP_HIGH; */
#define DEF_AUTO_PULSE true       /**< Default Auto-pulse enable */
#define DEF_WELD_SOUND true       /**< Default Weld Sound enable */
#define DEF_OLED_INVERT false     /**< Default OLED orientation */

// Обмеження для операційних змінних
#define MIN_PULSE_TIME 1      /**< Minimum weld pulse time */
#define MAX_PULSE_TIME 100    /**< Absolute maximum weld pulse time */
#define MAX_APULSE_DELAY 50   /**< Maximum auto pulse delay */
#define MIN_APULSE_DELAY 5    /**< Minimum auto pulse delay */
#define MAX_SPULSE_TIME 100   /**< Maximum short pulse time */
#define MIN_SPULSE_TIME 0     /**< Minimum short pulse time */
#define MAX_BATT_ALARM 5400   /**< Maximum low battery alarm voltage */
#define MIN_BATT_BALARM 3500  /**< Minimum low battery alarm voltage */

// Макроси синхронізації
#define STANDBY_TIME_OUT 640000L /**< Device sleep timeout (ms) */  // 6400000L
#define EEPROM_UPDATE_T 60000                                        /**< EEPROM update time (ms) */
#define WP_RETRIGGER_DELAY 50 /**< Weld pulse re-trigger delay (ms /10) */
#define FS_TRIGGER_DELAY 200  /**< Foot switch activation delay (ms) */
#define RS_DEBOUNCE 20 /*20*/ /**< Rotary encoder & switch debounce time (ms) */
#define BV_INTERVAL 2000      /**< Battery voltage measurement interval (ms) */
#define T_INTERVAL 10000      /**< temperature measurement interval (ms) */
#define PV_DELAY 1000         /**< Time the foot switch has to be hold before the pulse data is shown after a pulse (ms) */

// Макет екрана дисплея
#define CHR_W 6            /**< Width of character [size=1] (pixels) */
#define CHR_H 8            /**< Height of character [size=1] (pixels) */
#define LINE_H (CHR_H + 2) /**< Height of line [size=1] (pixels) */

// Макроси для визначення логічних станів
#define DD_READ true      /**< Data transfer direction - read */
#define DD_WRITE false    /**< Data transfer direction - write */
#define Pulse_ON true         /**< General macro for ON state */
#define Pulse_OFF false       /**< General macro for OFF state */
#define B_DN true         /**< General macro for DOWN state */
#define B_UP false        /**< General macro for UP state */
#define PL_ACTIVE_H false /**< Pin logic macro for Active High */
#define PL_ACTIVE_L true  /**< Pin logic macro for Active Low */

// EEPROM macros
#define EEA_ID 0               /**< Address of unique ID */
#define EEA_PDATA (EEA_ID + 4) /**< Eeprom address of program data */
#define EE_UNIQUEID 0x18fae9c8 /**< Unique Eeprom verification ID */
#define EE_FULL_RESET true     /**< Reset parameter to reset all Eeprom parameters */

// Маскування макросів під функції - робить код більш читабельним
/** Цей макрос зчитує стан кнопкового перемикача на енкодері. */
#define btnState() (!digitalRead(PIN_SW))

/** Цей макрос керує зварювальним імпульсом. */
#define weldPulse(state) digitalWrite(PIN_PULSE, state ? HIGH : LOW)

/** Куди подівся цей макрос? Він був у WString.h */
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper*>(pstr_pointer))

/***************************************************************************************************
* OLED Display Configuration                                                                       *
***************************************************************************************************/

#define SCREEN_WIDTH 128  // Ширина OLED-дисплея, в пікселях
#define SCREEN_HEIGHT 64  // Висота OLED-дисплея, в пікселях

#define OLED_RESET 4   /**< OLED mode */
#define OLED_INVERT 2  /**< OLED defined orientation mode - check OLED doc'n */
#define SPLASHTIME 255 /**< Splash screen time (ms) */


/***************************************************************************************************
* Structure, union, and enumerated type definitions                                                *
***************************************************************************************************/

typedef enum {             /**< Type enumerations for format of variables */
               VF_BATTALM, /**< Battery alarm voltage */
               VF_TEMPALM, /**< Temperature Alarm value */
               VF_BATTV,   /**< Battery voltage */
               VF_BATTA,   /**< Battery Amps */
               VF_TEMP,    /**< Temperature */
               VF_WELDCNT, /**< Weld count */
               VF_PLSDLY,  /**< Pulse delay */
               VF_SHTPLS,  /**< Short pulse duration */
               VF_DELAY    /**< Delay */
} vf_Type;

typedef struct progData {       /**< Program operating data structure */
  uint8_t autoPulseDelay;       /**< Auto-pulse delay (ms/100) */
  uint16_t batteryAlarm;         /**< Low battery voltage (A/D count) */
  uint16_t batteryhighAlarm;     /**< High battery voltage (A/D count) */
  uint8_t TCelsius;             /**< Temperature in Celsius */
  uint8_t maxTCelsius;          /**< maximum Temperature in Celsius */
  uint16_t weldCount;           /**< Count of welds performed */
  uint16_t pulseTime;           /**< Pulse time (ms) */
  uint16_t maxPulseTime;        /**< Maximum allowed pulse time (ms) */
  uint8_t shortPulseTime;       /**< Short pulse time (% of pulse time) */
  uint16_t batteryOffset;         /**< Battery voltage calibration offset (signed) x10 */
  uint16_t PulseBatteryVoltage; /**< Battery voltage during pulse x10 */
  uint16_t PulseAmps;           /**< esimated Amps during pulse x10 */
  uint16_t PulseShuntVoltage;
  struct progFlags {            /**< Program logical flags */
    unsigned en_autoPulse : 1;  /**< Auto-pulse enable */
    unsigned en_Sound : 1;      /**< Weld Sound enable */
    unsigned en_oledInvert : 1; /**< OLED orientation - true for inverted else normal */
    unsigned unused : 6;        /**< Unused program flags */
  } pFlags;
} progData;

/***************************************************************************************************
* Procedure prototypes                                                                             *
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
void message(const __FlashStringHelper*, const __FlashStringHelper*,
             const __FlashStringHelper*, uint8_t = 0);
void displayMenuType1(const __FlashStringHelper*, const __FlashStringHelper*,
                      const __FlashStringHelper*, const __FlashStringHelper*,
                      uint8_t SelectedItem);
void displayMenuType2(const __FlashStringHelper*, const char*, const __FlashStringHelper*);
void displayMainScreen();
void displayPulseData();
void displayLowBattery();
void displayHighBattery();
void displayHighTemperature();
void drawStatusLine();
void setTextProp(uint8_t, uint8_t, uint8_t, uint16_t = WHITE, boolean = false);
char* valStr(char*, uint16_t, vf_Type);

void enterMainScreen();
void handleMainScreenCnt();
void enterSystemScreen();
void handleSystemMenu(char *str);
void handleRebootMenu(char *str);
void handleMenuItem(uint8_t menuIndex, uint16_t &setting, uint16_t minValue, uint16_t maxValue, const char* title, const char* unit);
void handleSubMenu2(char *str);

/***************************************************************************************************
* Language strings (very simple language implementation - English is the default)                  *
***************************************************************************************************/

// Copy the language strings from the else clause into your language specific clause, then alter
// the strings to suit your language. Define your language at the top of this header file. It is
// important to maintain the correct formatting of the strings. Each string has an inline comment
// defining its format.

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
