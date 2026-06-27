/***************************************************************************************************
* Процедури керування меню та дисплеєм                                                             *
***************************************************************************************************/
void displayMenuType1(const __FlashStringHelper *title, const __FlashStringHelper *line1,
                      const __FlashStringHelper *line2, const __FlashStringHelper *line3, uint8_t SelectedItem) {
  display.clearDisplay();
  if (title == NULL) drawStatusLine();
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

void displayMenuType2(const __FlashStringHelper *title, const char *value, const __FlashStringHelper *units) {
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
void setTextProp(uint8_t size, uint8_t xpos, uint8_t ypos, uint16_t color, bool invert) {
  display.setTextSize(size);
  display.setCursor(xpos, ypos);
  if (invert) display.setTextColor(BLACK, color);
  else display.setTextColor(color);
}

void drawStatusLine() {
  char str[16];
  drawText(1, 0, 1, (pData.pFlags.en_autoPulse ? FPSTR(LS_AUTO_BAR) : FPSTR(LS_MANUAL_BAR)));
  uint16_t batteryAmphere = abs(INA.getCurrent_mA());
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 14, 1, valStr(str, batteryAmphere / 10, VF_BATTV), FPSTR(LS_AUNITS));
  display.drawLine(0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE);
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 6, 1, valStr(str, batteryVoltage / 10, VF_BATTV), FPSTR(LS_VUNITS));
}

void displayHighVoltageWarning() {
  char str[16];
  display.fillRect(0, 15, SCREEN_WIDTH, 34, BLACK);
  setTextProp(2, (SSD1306_LCDWIDTH - (10 * CHR_W * 2)) / 2 + 12, 20, WHITE);
  display.print(FPSTR(LS_HIGHV));
  setTextProp(1, (SSD1306_LCDWIDTH - (12 * CHR_W)) / 2, 38, WHITE);
  display.print(F("Current: "));
  display.print(valStr(str, batteryVoltage / 10, VF_BATTV));
  display.print(FPSTR(LS_VUNITS));
}

void displayLowVoltageWarning() {
  char str[16];
  display.fillRect(0, 15, SCREEN_WIDTH, 34, BLACK);
  setTextProp(2, (SSD1306_LCDWIDTH - (9 * CHR_W * 2)) / 2 + 12, 20, WHITE);
  display.print(FPSTR(LS_LOWV));
  setTextProp(1, (SSD1306_LCDWIDTH - (12 * CHR_W)) / 2, 38, WHITE);
  display.print(F("Current: "));
  display.print(valStr(str, batteryVoltage / 10, VF_BATTV));
  display.print(FPSTR(LS_VUNITS));
}

void displayMainScreen(bool signaled) {
  char str[16];
  display.clearDisplay();
  drawStatusLine();

  setTextProp(4, 0, 16 + CHR_H / 2);
  display.print(valStr(str, pData.pulseTime, VF_PLSDLY));
  setTextProp(2, 12 * CHR_W, 32, WHITE, signaled);
  display.print(FPSTR(LS_MS));

  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 6, 20, valStr(str, pData.weldCount, VF_WELDCNT), FPSTR(LS_WELDS));

  char pulseAmpsStr[8];
  drawText(1, 0, 56, FPSTR(LS_PULSE));
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 14, 56, valStr(pulseAmpsStr, pData.PulseAmps, VF_BATTA), FPSTR(LS_AUNITS));
  drawValueWithUnits(1, SSD1306_LCDWIDTH - CHR_W * 6, 56, valStr(str, pData.PulseBatteryVoltage / 10, VF_BATTV), FPSTR(LS_VUNITS));

  if (highVoltageAlarmActive) {
    displayHighVoltageWarning();
  } else if (lowVoltageAlarmActive) {
    displayLowVoltageWarning();
  }

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

void displayBatteryStatus(const __FlashStringHelper *statusText, const char */*value*/) {
  char str[8];
  display.clearDisplay();
  drawStatusLine();
  drawText(2, (SSD1306_LCDWIDTH - (sizeof(LS_BATTERY) - 1) * CHR_W * 2) / 2, 16, FPSTR(LS_BATTERY));
  uint8_t statusLen = strlen_P((PGM_P)statusText);
  drawText(2, (SSD1306_LCDWIDTH - statusLen * CHR_W * 2) / 2, 16 + 2 * LINE_H, statusText);
  drawValueWithUnits(1, (SSD1306_LCDWIDTH - 1 * CHR_W) / 2, 16 + 4 * LINE_H, valStr(str, batteryVoltage, VF_BATTV), FPSTR(LS_VUNITS));
  display.display();
}

void displayTemperatureStatus(const __FlashStringHelper *statusText, const __FlashStringHelper *adviceText) {
  char str[8];
  display.clearDisplay();
  drawStatusLine();
  drawValueWithUnits(2, (SSD1306_LCDWIDTH - (sizeof(LS_HIGHT) - 1) * CHR_W * 2) / 2, 16, valStr(str, TCelsius, VF_TEMP), FPSTR(LS_TUNITS));
  uint8_t statusTempLen = strlen_P((PGM_P)statusText);
  drawText(2, (SSD1306_LCDWIDTH - statusTempLen * CHR_W * 2) / 2, 16 + 2 * LINE_H, statusText);
  uint8_t adviceLen = strlen_P((PGM_P)adviceText);
  drawText(1, (SSD1306_LCDWIDTH - adviceLen * CHR_W) / 2, 16 + 4 * LINE_H, adviceText);
  display.display();
}

void displayLowBattery() {
  char str[16];
  displayBatteryStatus(FPSTR(LS_LOWV), valStr(str, batteryVoltage, VF_BATTV));
}

void displayHighTemperature() {
  displayTemperatureStatus(FPSTR(LS_HIGHT), FPSTR(LS_COOL));
}

void message(const __FlashStringHelper *line1, const __FlashStringHelper *line2,
             const __FlashStringHelper *line3, uint8_t displayTime) {
  display.clearDisplay();
  drawText(1, (SSD1306_LCDWIDTH - (sizeof(LS_MSGHDR) - 1) * CHR_W) / 2, 1, FPSTR(LS_MSGHDR));
  display.drawLine(0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE);
  drawText(2, 1, 16, line1);
  drawText(1, 1, 16 + 2 * LINE_H, line2);
  drawText(1, 1, 16 + 3 * LINE_H, line3);
  display.display();
  if (displayTime) delay(displayTime * 1000);
}

void splash() {
  display.clearDisplay();
  display.display();
  drawText(1, 1, 1, F(_DEVICENAME_));
  drawText(1, 1, 16, F("Ver " xstr(_VERSION_MAJOR_) "." xstr(_VERSION_MINOR_) "." xstr(_REVISION_) " " __DATE__));
  drawText(1, 1, 16 + 2 * LINE_H, F("Copyright (c) " _COPYRIGHT_));
  display.display();

  uint16_t timer = 0;
  while (btnState() != B_DN && timer < SPLASHTIME) {
    wdt_reset();
    delay(10);
    timer += 10;
  }
  while (btnState() == B_DN) wdt_reset();

  display.clearDisplay();
  display.display();
}

void foot_switch_error() {
  display.clearDisplay();
  display.display();
  drawText(1, 1, 1, F("FOOT SWITCH ERROR!"));
  drawText(1, 1, 16, F("Please:"));
  drawText(1, 1, 16 + LINE_H, F("- turn off welder"));
  drawText(1, 1, 16 + 2 * LINE_H, F("- remove foot switch"));
  drawText(1, 1, 16 + 3 * LINE_H, F("- correct the wiring"));
  display.display();

  unsigned long fsTimeout = millis();
  while (!digitalRead(PIN_FOOT_SWITCH)) {
    wdt_reset();
    if (millis() - fsTimeout > 30000UL) {
      break;
    }
  }
}
