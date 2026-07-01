/***************************************************************************************************
* Стан пристрою — FSM та обробники станів/меню                                                     *
***************************************************************************************************/

// Глобальна структура для стану меню
struct MenuState {
  uint8_t selectedMenu = 0;
  uint8_t selectedMainMenu = 0;
  uint8_t selectedSubMenu = 0;
  bool btn = false;
};

MenuState menuState;  // Глобальна змінна для стану меню
bool autoPulseTriggered = false; // Захист від помилкового авто-імпульсу

void stateMachine() {
  char str[8];

  uint8_t currentEvent = atomicReadEvent();

  if (currentEvent == EV_BOOTDN) {
    mState = ST_SYSTEM_SCREEN;
    atomicSetEvent(EV_NONE);

  } else {
    checkForBtnEvent();
    checkForSleepEvent();
    checkForLowVoltageEvent();
    checkTemp();

    currentEvent = atomicReadEvent();

    switch (currentEvent) {
      case EV_STBY_TIMEOUT:
        mState = ST_STANDBY;
        atomicSetEvent(EV_NONE);
        break;
      case EV_TEMP_HIGH:
        if (!sysMenu) mState = ST_TEMP_HIGH;
        atomicSetEvent(EV_NONE);
        break;
      case EV_BTNUP:
        if (mState != ST_REBOOT_MENU) atomicSetEvent(EV_NONE);
        break;
    }
  }

  switch (mState) {
    case ST_STANDBY: handleStandbyState(); break;
    case ST_TEMP_HIGH: handleTempHighState(); break;
    case ST_MAIN_SCREEN: enterMainScreen(); break;
    case ST_MAIN_SCREEN_CNT: handleMainScreenCnt(); break;
    case ST_MENU_SCREEN: handleMenuScreen(str); break;
    case ST_SUB_MENU_1: handleSubMenu1(str); break;
    case ST_SUB_MENU_2: handleSubMenu2(str); break;
    case ST_SYSTEM_SCREEN: enterSystemScreen(); break;
    case ST_SYSTEM_MENU: handleSystemMenu(str); break;
    case ST_REBOOT_MENU: handleRebootMenu(str); break;
    case ST_MAXWELD_SCREEN: handleMaxWeldScreen(str); break;
    case ST_INVERT_SCREEN: handleInvertScreen(); break;
    default: break;
  }
}

void handleStandbyState() {
  if (atomicReadEvent() == EV_BTNDN) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  message(FPSTR(LS_STANDBY), FPSTR(LS_CLICKBTN), FPSTR(LS_EXITSTBY));
  atomicSetEvent(EV_NONE);
}

void handleTempHighState() {
  if (atomicReadEvent() == EV_BTNDN) {
    if (ntcError) {
      ntcBypassed = true;
    }
    mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
  }
  if (ntcError) {
    displayNtcError();
  } else {
    displayHighTemperature();
  }
  atomicSetEvent(EV_NONE);
}

void enterMainScreen() {
  mState = ST_MAIN_SCREEN_CNT;
  sysMenu = false;
  menuState.selectedMenu = 0;
  autoPulseTriggered = digitalRead(PIN_AUTO_PULSE) ? true : false;
  displayMainScreen();
}

void handleMainScreenCnt() {
  if ((lowVoltageAlarmActive && !lowVoltageBypassed) || (highVoltageAlarmActive && !highVoltageBypassed) || ina226Error) {
    autoPulseTriggered = false;
    displayMainScreen();

    uint8_t ev = atomicReadEvent();
    if (ev == EV_BTNDN) {
      if (lowVoltageAlarmActive && !lowVoltageBypassed) {
        lowVoltageBypassed = true;
      } else if (highVoltageAlarmActive && !highVoltageBypassed) {
        highVoltageBypassed = true;
      }
      atomicSetEvent(EV_NONE);
      displayMainScreen();
    }
    return;
  }

  if (digitalRead(PIN_AUTO_PULSE) && pData.pFlags.en_autoPulse) {
    if (!autoPulseTriggered) {
      autoPulseTriggered = true;
      displayMainScreen(1);
      if (pData.pFlags.en_Sound) {
        playBeep(1000, 100);
      }
      sendWeldPulse(PIN_AUTO_PULSE, pData.autoPulseDelay * 100, WP_RETRIGGER_DELAY);
      displayMainScreen();
    }
  } else if (!digitalRead(PIN_FOOT_SWITCH)) {
    if (digitalRead(PIN_AUTO_PULSE)) {
      sendWeldPulse(PIN_FOOT_SWITCH, FS_TRIGGER_DELAY, WP_RETRIGGER_DELAY, PL_ACTIVE_L);
    } else {
      displayNoContactWarning();
      if (pData.pFlags.en_Sound) {
        playBeep(300, 200); // Low warning tone
      }
      while (!digitalRead(PIN_FOOT_SWITCH)) {
        wdt_reset();
      }
      displayMainScreen();
    }
  } else {
    autoPulseTriggered = false;
  }

  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    mState = ST_MENU_SCREEN;
    menuState.selectedMenu = 0;
    atomicSetEvent(EV_NONE);
    displayMenuType1(nullptr, FPSTR(LS_APULSE), FPSTR(LS_BATTALM1), FPSTR(LS_SHORTPLS1), 0);
    return;
  }

  if (ev == EV_ENCUP || ev == EV_ENCDN) {
    handleEncoderEvent(ev);
    atomicSetEvent(EV_NONE);
  }

  displayMainScreen();
}

void handleEncoderEvent(uint8_t ev) {
  if (ev == EV_ENCUP) {
    pData.pulseTime = (pData.pulseTime < pData.maxPulseTime) ? pData.pulseTime + 1 : pData.maxPulseTime;
  } else {
    pData.pulseTime = (pData.pulseTime > MIN_PULSE_TIME) ? pData.pulseTime - 1 : MIN_PULSE_TIME;
  }
}

void handleMenuScreen(char *str) {
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    mState = ST_SUB_MENU_1;
    menuState.selectedMainMenu = menuState.selectedMenu;

    if (menuState.selectedMainMenu == 0)
      displayMenuType1(FPSTR(LS_APULSE), FPSTR(LS_MANAUTO),
                       FPSTR(LS_DELAY), FPSTR(LS_WELDSOUND), menuState.selectedMenu);
    else if (menuState.selectedMainMenu == 1)
      displayMenuType1(FPSTR(LS_BATTALM), FPSTR(LS_LOWALRM),
                       FPSTR(LS_HIGHALRM), FPSTR(LS_EXIT), 0);
    else if (menuState.selectedMainMenu == 2) displayMenuType2(FPSTR(LS_SHORTPLS),
                                                               valStr(str, pData.shortPulseTime, VF_SHTPLS), FPSTR(LS_PCOF));

    atomicSetEvent(EV_NONE);
    menuState.selectedMenu = 0;

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    if (ev == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
    else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

    atomicSetEvent(EV_NONE);
    displayMenuType1(NULL, FPSTR(LS_APULSE), FPSTR(LS_BATTALM1),
                     FPSTR(LS_SHORTPLS1), menuState.selectedMenu);
  }
}

void handleSubMenu1(char *str) {
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    if (menuState.selectedMainMenu == 0) {
      mState = ST_SUB_MENU_2;
      menuState.selectedSubMenu = menuState.selectedMenu;

      if (menuState.selectedSubMenu == 0) displayMenuType2(FPSTR(LS_AUTOPLSON), NULL,
                                                           pData.pFlags.en_autoPulse ? FPSTR(LS_AUTO) : FPSTR(LS_MANUAL));
      else if (menuState.selectedSubMenu == 1) displayMenuType2(FPSTR(LS_AUTOPLSDLY),
                                                                valStr(str, pData.autoPulseDelay, VF_DELAY), FPSTR(LS_SECONDS));
      else if (menuState.selectedSubMenu == 2) displayMenuType2(FPSTR(LS_WELDSOUNDM), NULL,
                                                                pData.pFlags.en_Sound ? FPSTR(LS_SOUNDON) : FPSTR(LS_SOUNDOFF));
      else if (menuState.selectedSubMenu == 3) displayMenuType2(FPSTR(LS_NOMVOLTMENU),
                                                                valStr(str, pData.nominalVoltage, VF_BATTALM), FPSTR(LS_VOLTAGE));

    } else if (menuState.selectedMainMenu == 1) {
      if (menuState.selectedMenu < 2) {
        mState = ST_SUB_MENU_2;
        menuState.selectedSubMenu = menuState.selectedMenu;
        if (menuState.selectedSubMenu == 0) {
          displayMenuType2(FPSTR(LS_LOWALRMMENU),
                           valStr(str, pData.batteryAlarm, VF_BATTALM), FPSTR(LS_VOLTAGE));
        } else {
          displayMenuType2(FPSTR(LS_HIGHALRMMENU),
                           valStr(str, pData.batteryhighAlarm, VF_BATTALM), FPSTR(LS_VOLTAGE));
        }
      } else {
        updateEEPROM(true);
        mState = ST_MAIN_SCREEN;
      }

    } else {
      updateEEPROM(true);
      mState = ST_MAIN_SCREEN;
    }

    atomicSetEvent(EV_NONE);
    menuState.selectedMenu = 0;

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    if (menuState.selectedMainMenu == 0) {
      if (ev == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 3 : menuState.selectedMenu - 1;
      else menuState.selectedMenu = menuState.selectedMenu == 3 ? 0 : menuState.selectedMenu + 1;

      if (menuState.selectedMenu < 3) {
        displayMenuType1(FPSTR(LS_APULSE), FPSTR(LS_MANAUTO),
                         FPSTR(LS_DELAY), FPSTR(LS_WELDSOUND), menuState.selectedMenu);
      } else {
        displayMenuType1(FPSTR(LS_APULSE), FPSTR(LS_DELAY),
                         FPSTR(LS_WELDSOUND), FPSTR(LS_NOMVOLT), 2);
      }

    } else if (menuState.selectedMainMenu == 1) {
      if (ev == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
      else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

      displayMenuType1(FPSTR(LS_BATTALM), FPSTR(LS_LOWALRM),
                       FPSTR(LS_HIGHALRM), FPSTR(LS_EXIT), menuState.selectedMenu);

    } else if (menuState.selectedMainMenu == 2) {
      if (ev == EV_ENCDN) pData.shortPulseTime = pData.shortPulseTime > MIN_SPULSE_TIME ? pData.shortPulseTime - 1 : MIN_SPULSE_TIME;
      else pData.shortPulseTime = pData.shortPulseTime < MAX_SPULSE_TIME ? pData.shortPulseTime + 1 : MAX_SPULSE_TIME;

      displayMenuType2(FPSTR(LS_SHORTPLS),
                       valStr(str, pData.shortPulseTime, VF_SHTPLS), FPSTR(LS_PCOF));
    }
    atomicSetEvent(EV_NONE);
  }
}

void handleSubMenu2(char *str) {
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    updateEEPROM(true);
    mState = ST_MAIN_SCREEN;
    atomicSetEvent(EV_NONE);
    menuState.selectedMenu = 0;

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    if (menuState.selectedMainMenu == 0) {
      if (menuState.selectedSubMenu == 0) {
        pData.pFlags.en_autoPulse ^= 1;
        displayMenuType2(FPSTR(LS_AUTOPLSON), NULL, pData.pFlags.en_autoPulse ? FPSTR(LS_AUTO) : FPSTR(LS_MANUAL));
      } else if (menuState.selectedSubMenu == 1) {
        if (ev == EV_ENCDN) pData.autoPulseDelay = pData.autoPulseDelay > MIN_APULSE_DELAY ? pData.autoPulseDelay - 1 : MIN_APULSE_DELAY;
        else pData.autoPulseDelay = pData.autoPulseDelay < MAX_APULSE_DELAY ? pData.autoPulseDelay + 1 : MAX_APULSE_DELAY;

        displayMenuType2(FPSTR(LS_AUTOPLSDLY),
                         valStr(str, pData.autoPulseDelay, VF_DELAY), FPSTR(LS_SECONDS));
      } else if (menuState.selectedSubMenu == 2) {
        pData.pFlags.en_Sound ^= 1;
        displayMenuType2(FPSTR(LS_WELDSOUNDM), NULL, pData.pFlags.en_Sound ? FPSTR(LS_SOUNDON) : FPSTR(LS_SOUNDOFF));
      } else if (menuState.selectedSubMenu == 3) {
        if (ev == EV_ENCDN) pData.nominalVoltage = pData.nominalVoltage > 30 ? pData.nominalVoltage - 1 : 30;
        else pData.nominalVoltage = pData.nominalVoltage < 150 ? pData.nominalVoltage + 1 : 150;

        displayMenuType2(FPSTR(LS_NOMVOLTMENU),
                         valStr(str, pData.nominalVoltage, VF_BATTALM), FPSTR(LS_VOLTAGE));
      }
    } else if (menuState.selectedMainMenu == 1) {
      if (menuState.selectedSubMenu == 0) {
        uint16_t maxLowAlarm = pData.batteryhighAlarm > 0 ? pData.batteryhighAlarm - 1 : MIN_BATT_BALARM / 100;
        if (ev == EV_ENCDN) {
          pData.batteryAlarm = pData.batteryAlarm > MIN_BATT_BALARM / 100 ? pData.batteryAlarm - 1 : MIN_BATT_BALARM / 100;
        } else {
          pData.batteryAlarm = pData.batteryAlarm < maxLowAlarm ? pData.batteryAlarm + 1 : maxLowAlarm;
        }
        displayMenuType2(FPSTR(LS_LOWALRMMENU),
                         valStr(str, pData.batteryAlarm, VF_BATTALM), FPSTR(LS_VOLTAGE));
      } else if (menuState.selectedSubMenu == 1) {
        uint16_t minHighAlarm = pData.batteryAlarm + 1;
        if (ev == EV_ENCDN) {
          pData.batteryhighAlarm = pData.batteryhighAlarm > minHighAlarm ? pData.batteryhighAlarm - 1 : minHighAlarm;
        } else {
          pData.batteryhighAlarm = pData.batteryhighAlarm < MAX_BATT_ALARM / 100 ? pData.batteryhighAlarm + 1 : MAX_BATT_ALARM / 100;
        }
        displayMenuType2(FPSTR(LS_HIGHALRMMENU),
                         valStr(str, pData.batteryhighAlarm, VF_BATTALM), FPSTR(LS_VOLTAGE));
      }
    }
    atomicSetEvent(EV_NONE);
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
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    atomicSetEvent(EV_NONE);

    if (menuState.selectedMenu == 0) {
      displayMenuType2(FPSTR(LS_MAXPLSMENU),
                       valStr(str, pData.maxPulseTime, VF_PLSDLY), FPSTR(LS_MS));
      mState = ST_MAXWELD_SCREEN;
      atomicSetEvent(EV_NONE);

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

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    if (ev == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
    else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

    atomicSetEvent(EV_NONE);
    displayMenuType1(FPSTR(LS_SYSMENU), FPSTR(LS_MAXPULSE),
                     FPSTR(LS_DISPLAY), FPSTR(LS_BOOT), menuState.selectedMenu);
  }
}

void handleRebootMenu(char */*str*/) {
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) menuState.btn = true;
  else if ((ev == EV_BTNUP) && menuState.btn) {
    if (menuState.selectedMenu == 1)
      resetEEPROM(false);
    else if (menuState.selectedMenu == 2)
      resetEEPROM(true);

    message(FPSTR(LS_REBOOT), menuState.selectedMenu == 1 ? FPSTR(LS_REBOOTSR) : menuState.selectedMenu == 2 ? FPSTR(LS_REBOOTFR)
                                                                                                             : FPSTR(LS_REBOOTNR),
            FPSTR(LS_WAITMSG), 2);

    delay(1000);
    reboot();

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    if (ev == EV_ENCDN) menuState.selectedMenu = menuState.selectedMenu == 0 ? 2 : menuState.selectedMenu - 1;
    else menuState.selectedMenu = menuState.selectedMenu == 2 ? 0 : menuState.selectedMenu + 1;

    displayMenuType1(FPSTR(LS_BOOTMENU), FPSTR(LS_REBOOT),
                     FPSTR(LS_SAFERST), FPSTR(LS_FULLRST), menuState.selectedMenu);
  }
  atomicSetEvent(EV_NONE);
}

void handleMaxWeldScreen(char *str) {
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    updateEEPROM(true);
    message(FPSTR(LS_MAXPULSE), FPSTR(LS_MAXPMSG), FPSTR(LS_WAITMSG), 2);
    mState = ST_SYSTEM_SCREEN;
    menuState.selectedMenu = 0;

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    if (ev == EV_ENCDN) pData.maxPulseTime = pData.maxPulseTime > MIN_PULSE_TIME ? pData.maxPulseTime - 1 : MIN_PULSE_TIME;
    else pData.maxPulseTime = pData.maxPulseTime < MAX_PULSE_TIME ? pData.maxPulseTime + 1 : MAX_PULSE_TIME;

    pData.maxPulseTime = pData.maxPulseTime < MIN_PULSE_TIME ? MIN_PULSE_TIME : pData.maxPulseTime > MAX_PULSE_TIME ? MAX_PULSE_TIME
                                                                                                                    : pData.maxPulseTime;
    displayMenuType2(FPSTR(LS_MAXPLSMENU),
                     valStr(str, pData.maxPulseTime, VF_PLSDLY), FPSTR(LS_MS));
  }
  atomicSetEvent(EV_NONE);
}

void handleInvertScreen() {
  uint8_t ev = atomicReadEvent();
  if (ev == EV_BTNDN) {
    updateEEPROM(true);
    mState = ST_SYSTEM_SCREEN;
    atomicSetEvent(EV_NONE);
    menuState.selectedMenu = 0;

  } else if (ev == EV_ENCUP || ev == EV_ENCDN) {
    pData.pFlags.en_oledInvert = !pData.pFlags.en_oledInvert;
    display.setRotation(pData.pFlags.en_oledInvert ? 2 : 0);
    displayMenuType2(FPSTR(LS_INVERTMENU), NULL, pData.pFlags.en_oledInvert ? FPSTR(LS_SCRINV) : FPSTR(LS_SCRNORM));
  }
  atomicSetEvent(EV_NONE);
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

uint16_t drawProgress(struct progress *o, bool clear) {
  const uint16_t steps = 126;
  const uint16_t height = 8;
  const uint16_t x = SSD1306_LCDWIDTH - steps - 1;
  const uint16_t y = 55;

  if (clear) {
    display.fillRect(x - 1, y - 1, steps + 2, height + 2, BLACK);
    return 0;
  }

  if (!(o->opt & PGR_INIT)) {
    display.drawRect(x - 1, y - 1, steps + 2, height + 2, WHITE);
    display.fillRect(x, y, steps, height, (o->opt & PGR_ON) ? BLACK : WHITE);
    display.display();
    o->step = 1;
    o->millis = millis();
    o->opt |= PGR_INIT;
    return 0;
  }

  uint16_t elapsed = millis() - o->millis;
  uint16_t interval = o->time / steps;
  uint16_t expectedStep = (interval > 0) ? (elapsed / interval) : steps;

  if (expectedStep > o->step) {
    o->step = expectedStep;
    display.fillRect(x, y, o->step, height, (o->opt & PGR_ON) ? WHITE : BLACK);
    display.display();
  }

  return (o->step >= steps) ? 1 : 0;
}

void sendWeldPulse(uint8_t sensePin, uint16_t delayEngage, uint16_t delayRelease, bool senseActiveLevel) {
  struct progress wait;
  bool activePinState = (senseActiveLevel == PL_ACTIVE_H);
  unsigned long shortPulseDelay = max(1UL, (pData.pulseTime * pData.shortPulseTime) / 100);

#ifdef _DEVELOPMENT_
  Serial.println(F("Авто-імпульс активовано!"));
#endif

  if (sensePin == PIN_AUTO_PULSE) {
    wait.opt = PGR_ON;
    wait.time = delayEngage;
    while (!drawProgress(&wait, false)) {
      wdt_reset();
      if (digitalRead(PIN_AUTO_PULSE) != activePinState) {
        drawProgress(&wait, true);
        return;
      }
    }
  } else {
    uint16_t d = delayEngage;
    while (d >= 50) { wdt_reset(); delay(50); d -= 50; }
    if (d > 0) { wdt_reset(); delay(d); }
  }

  if (pData.pFlags.en_Sound) playBeep(1500, 100);

  if (pData.shortPulseTime > 0 && pData.pulseTime > 3) {
    wdt_reset();
    weldPulse(Pulse_ON);
    delay(shortPulseDelay);
    weldPulse(Pulse_OFF);
    wdt_reset();
    delay(shortPulseDelay);
  }

  // --- Динамічна компенсація часу імпульсу ---
  uint32_t vnom_scaled = pData.nominalVoltage;
  uint32_t vnom2 = vnom_scaled * vnom_scaled;
  uint32_t vact_scaled = (batteryVoltage + 50) / 100;
  if (vact_scaled < 10) vact_scaled = DEF_NOM_BATT_V / 100;
  uint32_t vact2 = vact_scaled * vact_scaled;
  uint32_t compTime32 = (uint32_t)pData.pulseTime * vnom2 / vact2;

  uint16_t minTime = (pData.pulseTime * 7 + 5) / 10;
  uint16_t maxTime = (pData.pulseTime * 14 + 5) / 10;
  uint16_t compensatedPulseTime = (uint16_t)compTime32;
  if (compensatedPulseTime < minTime) compensatedPulseTime = minTime;
  if (compensatedPulseTime > maxTime) compensatedPulseTime = maxTime;

#ifdef _DEVELOPMENT_
  Serial.print(F("Base time: ")); Serial.print(pData.pulseTime);
  Serial.print(F("ms, V: ")); Serial.print(batteryVoltage / 1000); Serial.print('.'); Serial.print((batteryVoltage % 1000) / 100);
  Serial.print(F("V, Compensated time: ")); Serial.println(compensatedPulseTime);
#endif

  Wire.setClock(800000);
  uint16_t NominalGauss = analogRead(A0);
  delay(ADC_SETTLE_DELAY_MS);
  wdt_reset();

  uint16_t busVoltageBefore = INA.getBusVoltage_mV(); // Зчитуємо напругу спокою безпосередньо перед імпульсом
  INA.setAverage(0); // Встановлюємо 1 замір (без усереднення), щоб миттєво зафіксувати просідання напруги

  uint32_t pulseStart_us = micros();
  uint32_t pulseDuration_us = (uint32_t)compensatedPulseTime * 1000UL;
  uint32_t halfPulseDuration_us = pulseDuration_us / 2;

  weldPulse(Pulse_ON);
  while ((micros() - pulseStart_us) < halfPulseDuration_us) { }
  ADCSRA |= (1 << ADSC); // Асинхронно запускаємо АЦП зчитування струму A0
  while ((micros() - pulseStart_us) < pulseDuration_us) { }
  weldPulse(Pulse_OFF);

  wdt_reset();

  while (ADCSRA & (1 << ADSC)) { } // Очікуємо закінчення фонового перетворення АЦП
  uint16_t PulseGauss = ADC;
  uint16_t busVoltageDuring = INA.getBusVoltage_mV();
  INA.setAverage(INA_AVERAGING_MODE); // Повертаємо стандартне усереднення (128 замірів)

  pData.PulseBatteryVoltage = busVoltageDuring;
  pData.PulseAmps = (PulseGauss > NominalGauss) ? (PulseGauss - NominalGauss) * 10 : 0;

  // Розраховуємо опір контуру в мОм (x10, тобто 20 = 2.0 мОм)
  if (pData.PulseAmps > 0 && busVoltageBefore > busVoltageDuring) {
    pData.PulseResistance = (uint32_t)100 * (busVoltageBefore - busVoltageDuring) / pData.PulseAmps;
  } else {
    pData.PulseResistance = 0;
  }

#ifdef _DEVELOPMENT_
  Serial.print(F("V_before: ")); Serial.print(busVoltageBefore);
  Serial.print(F(" mV, V_during: ")); Serial.print(busVoltageDuring);
  Serial.print(F(" mV, Amps: ")); Serial.print(pData.PulseAmps);
  Serial.print(F(", R: ")); Serial.print(pData.PulseResistance / 10); Serial.print('.'); Serial.print(pData.PulseResistance % 10); Serial.println(F(" mR"));
#endif

  // Очікування деактивації датчика (відпускання кнопки/педалі)
  while (digitalRead(sensePin) == activePinState) {
    wdt_reset(); // Запобігаємо WDT reset під час утримання
    weldPulse(Pulse_OFF);
  }

  wait.opt = PGR_OFF;
  wait.time = delayRelease;
  while (!drawProgress(&wait, false)) { wdt_reset(); }

  pData.weldCount++;
  atomicSetLastActiveTime(millis());
}

void checkForLowVoltageEvent() {
  static unsigned long lastBVTime = 1;
  if (millis() - lastBVTime > BV_INTERVAL) {
    lastBVTime = millis();
    batteryVoltage = INA.getBusVoltage_mV();

#ifdef _TESTING_
    batteryVoltage = DEF_NOM_BATT_V;
#endif

    if (batteryVoltage == 0) {
      ina226Error = true;
      highVoltageAlarmActive = false;
      lowVoltageAlarmActive = false;
    } else {
      ina226Error = false;

      uint16_t highAlarmVoltage_mV = (uint16_t)pData.batteryhighAlarm * 100;
      if (batteryVoltage > highAlarmVoltage_mV) {
        if (!highVoltageAlarmActive) {
          highVoltageAlarmActive = true;
          if (pData.pFlags.en_Sound) playHighVoltageAlarmSound();
        }
      } else {
        highVoltageAlarmActive = false;
        highVoltageBypassed = false;
      }

      uint16_t alarmVoltage_mV = (uint16_t)pData.batteryAlarm * 100;
      if (batteryVoltage < alarmVoltage_mV) {
        if (!lowVoltageAlarmActive) {
          lowVoltageAlarmActive = true;
          if (pData.pFlags.en_Sound) playLowVoltageAlarmSound();
        }
      } else {
        lowVoltageAlarmActive = false;
        lowVoltageBypassed = false;
      }
    }
  }
}

const uint16_t ntc_adc_table[] PROGMEM = {
  786, 736, 683, 626, 569, 512, 457, 405, 357, 313, 273, 238, 207, 180, 156, 136, 118, 103, 90, 79, 69
};

uint8_t readTemperatureCelsius(int adc) {
  if (adc >= (int)pgm_read_word(&ntc_adc_table[0])) return 0;
  if (adc <= (int)pgm_read_word(&ntc_adc_table[20])) return 100;
  for (uint8_t i = 0; i < 20; i++) {
    uint16_t y0 = pgm_read_word(&ntc_adc_table[i]);
    uint16_t y1 = pgm_read_word(&ntc_adc_table[i + 1]);
    if (adc <= (int)y0 && adc > (int)y1) {
      uint8_t t0 = i * 5;
      return t0 + (uint8_t)(5UL * (y0 - adc) / (y0 - y1));
    }
  }
  return 100;
}

void checkTemp() {
  static unsigned long lastTTime = 0;
  if (millis() - lastTTime > T_INTERVAL) {
    lastTTime = millis();
    int bitwertNTC = analogRead(PIN_TEMP);
    if (bitwertNTC > 900 || bitwertNTC < 15) {
      ntcError = true;
      TCelsius = 99;
      if (!ntcBypassed) {
        atomicSetEvent(EV_TEMP_HIGH);
      }
    } else {
      ntcError = false;
      ntcBypassed = false;
      TCelsius = readTemperatureCelsius(bitwertNTC);
      if (TCelsius > DEF_HIGH_TEMP_ALARM) {
        atomicSetEvent(EV_TEMP_HIGH);
      }
    }
  }
}

void checkForSleepEvent() {
  if (atomicReadLastActiveTime() + STANDBY_TIME_OUT < millis()) {
    atomicSetEvent(EV_STBY_TIMEOUT);
  }
}

void checkForBtnEvent() {
  static unsigned long lastBtnTime = 0;
  static bool lastBtnState = B_UP;
  bool thisBtnState = btnState();

  if (millis() - lastBtnTime > RS_DEBOUNCE) {
    if (thisBtnState != lastBtnState) {
      atomicSetEvent(thisBtnState == B_UP ? EV_BTNUP : EV_BTNDN);
      unsigned long now = millis();
      atomicSetLastActiveTime(now);
      lastBtnTime = now;
      lastBtnState = thisBtnState;
    }
  }
}
