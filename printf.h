/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * @file printf.h
 *
 * Setup necessary to direct stdout to the Arduino Serial library, which
 * enables 'printf'
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
#error This example is only for use on Arduino.
#endif  // ARDUINO

#endif  // __PRINTF_H__
