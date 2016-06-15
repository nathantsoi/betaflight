/* @file max7456.h
 * @brief max7456 spi driver
 *
 * @author Nathan Tsoi nathan@vertile.com
 *
 * Copyright (C) 2016 Nathan Tsoi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef MAX_OSD

#include "bus_spi.h"
#include "system.h"

#include "max7456_symbols.h"

/** 30 chars per line, that's just how it is */
#define CHARS_PER_LINE 30

/** autodetect pal or ntsc when set to 1, skip autodetection when set to any other value */
#define MAX_AUTODETECT_VIDEO_FORMAT 0

/** wait for the video camera to startup */
#define MAX_AUTODETECT_WAIT 0

/** PAL or NTSC, value is number of chars total */
#define VIDEO_MODE_PIXELS_NTSC 480
#define VIDEO_MODE_PIXELS_PAL 390

/** Define this to upload the font when the flight controller is reset **/
// TODO: warn or error if this is defined, but not DEBUG
//#define MAX_UPLOAD_FONT_ON_RESET

/** NVM ram size for one font char, actual character bytes **/
#define MAX_NVM_FONT_CHAR_SIZE 54
/** NVM ram field size for one font char, last 10 bytes dont matter **/
#define MAX_NVM_FONT_CHAR_FIELD_SIZE 64
/** Number of ram fields available for characters **/
#define MAX_NVM_FONT_CHAR_COUNT 255

/** an NVM_RAM-sized array **/
typedef struct maxFontChar_s {
  uint8_t data[MAX_NVM_FONT_CHAR_SIZE];
} maxFontChar_t;

void max7456Init(void);
// Write a single character
void max7456WriteChar(const char c, uint16_t address);
// Copy string from ram into screen buffer
void max7456WriteString(const char *string, uint16_t address);
// Write the screen buffer to the device
void max7456DrawScreen(void);
// Write one font character to NVM ram on the MAX7456
void max7456WriteFontChar(uint8_t char_address, maxFontChar_t *font);

// Write the default font to the MAX7456' NVM ram, for use in TEST ONLY! The font is big
#ifdef MAX_UPLOAD_FONT_ON_RESET
void max7456UpdateFont(void);
#endif

// Sadly, the easiest way to draw the artificial horizon is to expose this buffer
extern char max7456_screen_buffer[VIDEO_MODE_PIXELS_NTSC];

#endif //MAX_OSD
