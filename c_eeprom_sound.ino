/***************************************************************************************************
* Утиліта перетворення функцій                                                                     *
***************************************************************************************************/
char *valStr(char *str, uint16_t val, vf_Type fType) {
  switch (fType) {
    case VF_BATTALM: sprintf_P(str, PSTR("%u.%u"), val / 10, val % 10); break;
    case VF_TEMPALM: sprintf_P(str, PSTR("%4u"), val); break;
    case VF_BATTV: sprintf_P(str, PSTR("%2u.%02u"), val / 100, val % 100); break;
    case VF_BATTA: sprintf_P(str, PSTR("%5u"), val); break;
    case VF_WELDCNT: sprintf_P(str, PSTR("%5u"), val); break;
    case VF_TEMP: sprintf_P(str, PSTR("%5u"), val); break;
    case VF_PLSDLY: sprintf_P(str, PSTR("%3u"), val); break;
    case VF_SHTPLS: sprintf_P(str, PSTR("%3u"), val); break;
    case VF_DELAY: sprintf_P(str, PSTR("%u.%u"), val / 10, val % 10); break;
    default: str[0] = '\0'; break;
  }
  return str;
}

/***************************************************************************************************
* Утиліти EEPROM функцій                                                                           *
***************************************************************************************************/
void resetEEPROM(bool full) {
  pData.autoPulseDelay = DEF_AUTO_PLSDELAY;
  pData.PulseBatteryVoltage = DEF_PULSE_VOLTAGE;
  pData.PulseAmps = DEF_PULSE_AMPS;
  pData.PulseShuntVoltage = DEF_SHUNT_VOLTAGE;
  pData.batteryAlarm = DEF_BATT_ALARM / 100;
  pData.batteryhighAlarm = DEF_HIGH_BATT_ALARM / 100;
  pData.nominalVoltage = 55;
  pData.weldCount = full == EE_FULL_RESET ? 0 : pData.weldCount;
  pData.pulseTime = DEF_PULSE_TIME;
  pData.maxPulseTime = DEF_MAX_PULSE_TIME;
  pData.shortPulseTime = DEF_SPULSE_TIME;
  pData.pFlags.en_autoPulse = DEF_AUTO_PULSE;
  pData.pFlags.en_Sound = DEF_WELD_SOUND;
  pData.pFlags.en_oledInvert = full ? DEF_OLED_INVERT : pData.pFlags.en_oledInvert;

  EEPROM.put(EEA_PDATA, pData);

  uint16_t checksum = calculateChecksum(pData);
  EEPROM.put(EEA_CHECKSUM, checksum);
  EEPROM.put(EEA_ID, EE_UNIQUEID);

#if defined _DEVELOPMENT_ || defined _BOOTSYS_
  if (full) Serial.println(F("Повне скидання EEPROM з контрольною сумою"));
  else Serial.println(F("Скидання EEPROM з контрольною сумою"));
#endif /* _DEVELOPMENT_ || _BOOTSYS_*/
}

void loadEEPROM() {
  uint16_t storedChecksum;
  uint16_t calculatedChecksum;
  uint32_t uniqueID;

  EEPROM.get(EEA_CHECKSUM, storedChecksum);
  EEPROM.get(EEA_PDATA, pData);
  EEPROM.get(EEA_ID, uniqueID);

  calculatedChecksum = calculateChecksum(pData);

  if (calculatedChecksum != storedChecksum || uniqueID != EE_UNIQUEID) {
#ifdef _DEVELOPMENT_
    Serial.print(F("EEPROM Оновлено! Хеш-сума змінилась (HEX) | 0x"));
    Serial.print(storedChecksum, HEX);
    Serial.print(F(" >>> 0x"));
    Serial.println(calculatedChecksum, HEX);
    Serial.println(F("EEPROM Error!  Відновлення EEPROM >>>"));
#endif
    resetEEPROM(EE_FULL_RESET);
  }

  if (pData.nominalVoltage < 30 || pData.nominalVoltage > 150) {
    pData.nominalVoltage = 55;
  }

  if (pData.batteryAlarm < MIN_BATT_BALARM / 100 || pData.batteryAlarm > MAX_BATT_ALARM / 100) {
    pData.batteryAlarm = DEF_BATT_ALARM / 100;
  }
  if (pData.batteryhighAlarm < MIN_BATT_BALARM / 100 || pData.batteryhighAlarm > MAX_BATT_ALARM / 100) {
    pData.batteryhighAlarm = DEF_HIGH_BATT_ALARM / 100;
  }
  if (pData.batteryAlarm >= pData.batteryhighAlarm) {
    pData.batteryAlarm = DEF_BATT_ALARM / 100;
    pData.batteryhighAlarm = DEF_HIGH_BATT_ALARM / 100;
  }
}

void updateEEPROM() {
  static unsigned long lastEEUpdatetime = 0;
  uint16_t storedChecksum;

  if (millis() - lastEEUpdatetime > EEPROM_UPDATE_T) {
    lastEEUpdatetime = millis();

    EEPROM.get(EEA_CHECKSUM, storedChecksum);
    uint16_t calculatedChecksum = calculateChecksum(pData);
    if (calculatedChecksum != storedChecksum) {
      EEPROM.put(EEA_PDATA, pData);
      EEPROM.put(EEA_CHECKSUM, calculatedChecksum);

#ifdef _DEVELOPMENT_
      Serial.print(F("EEPROM Оновлено! Хеш-сума змінилась (HEX) | 0x"));
      Serial.print(storedChecksum, HEX);
      Serial.print(F(" >>> 0x"));
      Serial.println(calculatedChecksum, HEX);
#endif

    } else {
#ifdef _DEVELOPMENT_
      Serial.print(F("EEPROM OK!       Хеш-сума не змінилась (HEX) = 0x"));
      Serial.println(calculatedChecksum, HEX);
#endif /* _DEVELOPMENT_ */
    }
  }
}

// CRC-16/MODBUS
uint16_t calculateChecksum(const progData &data) {
  uint16_t crc = 0xFFFF;
  const byte *dataPtr = reinterpret_cast<const byte *>(&data);
  size_t dataSize = sizeof(progData);

  for (size_t i = 0; i < dataSize; ++i) {
    crc ^= dataPtr[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/***************************************************************************************************
* Процедури звуку зумера                                                                           *
***************************************************************************************************/

void Boot_Sound() {
  playBeep(1000, 200);
  playBeep(2000, 200);
  playBeep(3000, 200);
  playBeep(2000, 200);
}

void playHighVoltageAlarmSound() {
  playBeep(2000, 150);
  delay(50);
  playBeep(2500, 150);
  delay(50);
  playBeep(3000, 200);
}

void playLowVoltageAlarmSound() {
  playBeep(1500, 150);
  delay(50);
  playBeep(1000, 150);
  delay(50);
  playBeep(500, 200);
}

void playBeep(uint16_t frequency, uint16_t duration) {
  wdt_reset();
  tone(PIN_BUZZ, frequency, duration);
  delay(duration / 2);
  noTone(PIN_BUZZ);
}

void reboot() {
  wdt_enable(WDTO_15MS);
  while (1)
    ;
}
