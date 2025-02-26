/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 
 Ця програма є вільним програмним забезпеченням; ви можете розповсюджувати її та/або 
 змінювати відповідно до умов Стандартної публічної ліцензії GNU версії 2, 
 опублікованої Фондом вільного програмного забезпечення.
 */

/**
 * @file printf.h
 *
 * Налаштування, необхідне для спрямування stdout до бібліотеки Arduino Serial, яка увімкне 'printf'
 * 
 */

#ifndef __PRINTF_H__
#define __PRINTF_H__

#ifdef ARDUINO

int serial_putc(char c, FILE*) {
  Serial.write(c);

  return c;
}

void printf_begin(void) {
  fdevopen(&serial_putc, 0);
}

#else
#error Цей приклад призначений лише для використання на Arduino.
#endif  // ARDUINO

#endif  // __PRINTF_H__
