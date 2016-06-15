/* @file max7456.cpp
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

#include "drivers/max7456.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

// DO NOT include font file in production
#ifdef MAX_UPLOAD_FONT_ON_RESET
#include "max7456_font_bold.h"
// or
//#include "max7456_font.h"
#endif

#ifdef MAX_OSD

#define MAX_CS_ENABLE  IOLo(maxCsPin)
#define MAX_CS_DISABLE IOHi(maxCsPin)

#define SPI_WRITE(val) spiTransferByte(MAX_OSD_SPI_INSTANCE, val)
#define SPI_WRITE_REG(addr, val) SPI_WRITE(addr); SPI_WRITE(val)

// IO
static IO_t maxCsPin = IO_NONE;

// PAL or NTSC
static uint16_t _video_mode = VIDEO_MODE_PIXELS_NTSC;

// Main screen ram for MAX7456
char max7456_screen_buffer[VIDEO_MODE_PIXELS_NTSC];

// boolean flag to redraw screen;
static bool _screen_changed = true;

// Copy char into the screen buffer
void max7456WriteChar(const char c, uint16_t address)
{
  max7456_screen_buffer[address] = c;
  _screen_changed = true;
}

// Copy string from ram into screen buffer
void max7456WriteString(const char *string, uint16_t address)
{
  uint16_t xx;
  for(xx=0;string[xx]!=0;)
  {
    max7456_screen_buffer[address++] = string[xx++];
  }
  _screen_changed = true;
}

void max7456HardwareInit(void)
{
  static bool hardwareInitialised = false;

  if (hardwareInitialised) {
    return;
  }

#ifdef MAX_OSD_SPI_CS_PIN
    maxCsPin = IOGetByTag(IO_TAG(MAX_OSD_SPI_CS_PIN));
    IOInit(maxCsPin, OWNER_MAX_OSD, RESOURCE_SPI);
    IOConfigGPIO(maxCsPin, SPI_IO_CS_CFG);
#endif // SDCARD_SPI_CS_PIN

  spiSetDivisor(MAX_OSD_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

  MAX_CS_DISABLE;

  hardwareInitialised = true;
}

/**
 * Init should be called after the SPI and GPIO have been initialized, before the main loop starts
 */
void max7456Init(void)
{
  max7456HardwareInit();
  uint16_t x;
  // clear the buffer
  for(x=0; x<VIDEO_MODE_PIXELS_NTSC; x++) {
    max7456_screen_buffer[x] = 0;
  }

  MAX_CS_ENABLE;
  // soft reset
  SPI_WRITE_REG(MAX7456VM0_REG, MAX7456_RESET);
  delay(100);

#if MAX_AUTODETECT_VIDEO_FORMAT==1
  uint8_t srdata = 0;
  #if MAX_AUTODETECT_WAIT==1
    while ((0b00000011 & srdata) == 0){
      SPI_WRITE(MAX7456ADD_STAT);
      srdata = SPI_WRITE(MAX7456END_string);
      delay(100);
    }
  #else
    SPI_WRITE(MAX7456ADD_STAT);
    srdata = SPI_WRITE(MAX7456END_string);
  #endif //MAX_AUTODETECT_WAIT
  if ((0b00000001 & srdata) == 0b00000001) {
    _video_mode = VIDEO_MODE_PIXELS_PAL;
  }
  else if((0b00000010 & srdata) == 0b00000010) {
    _video_mode = VIDEO_MODE_PIXELS_NTSC;
  }
#endif //AUTOCAM

//#ifdef MAX_FASTPIXEL
//  // force fast pixel timing
//  SPI_WRITE_REG(MAX7456ADD_OSDM, 0x00);
//  // SPI_WRITE_REG(MAX7456ADD_OSDM, 0xEC);
//  // uint8_t srdata = SPI_WRITE(0xFF); //get data byte
//  // srdata = srdata & 0xEF;
//  // SPI_WRITE_REG(0x6c, srdata);
//  delay(100);
//#endif

  // set all rows to same charactor black/white level
  for(x = 0; x < _video_mode/CHARS_PER_LINE; x++) {
    SPI_WRITE_REG(MAX7456ADD_RB0+x, MAX7456_BWBRIGHTNESS);
  }

  // make sure the Max7456 is enabled
  SPI_WRITE(MAX7456VM0_REG);

  if (_video_mode == VIDEO_MODE_PIXELS_PAL) {
    SPI_WRITE(MAX7456OSD_ENABLE|MAX7456VIDEO_MODE_PAL);
  }
  else {
    SPI_WRITE(MAX7456OSD_ENABLE|MAX7456VIDEO_MODE_NTSC);
  }
  MAX_CS_DISABLE;
  delay(100);
}

void max7456DrawDebugScreen()
{
  uint16_t xx;

  MAX_CS_ENABLE;
  for(xx=0;xx<_video_mode;++xx){
    SPI_WRITE_REG(MAX7456ADD_DMAH, xx>>8);
    SPI_WRITE_REG(MAX7456ADD_DMAL, xx);
    SPI_WRITE_REG(MAX7456ADD_DMDI, xx);
  }
  MAX_CS_DISABLE;
}

void max7456DrawScreen()
{
  uint16_t xx;

  if (!_screen_changed) {
    return;
  }
  MAX_CS_ENABLE;
  for(xx=0;xx<_video_mode;++xx){
    SPI_WRITE_REG(MAX7456ADD_DMAH, xx>>8);
    SPI_WRITE_REG(MAX7456ADD_DMAL, xx);
    SPI_WRITE_REG(MAX7456ADD_DMDI, max7456_screen_buffer[xx]);
    max7456_screen_buffer[xx] = ' ';
  }
  MAX_CS_DISABLE;
  _screen_changed = false;
}

void max7456WriteFontChar(uint8_t char_address, maxFontChar_t *font)
{
  MAX_CS_ENABLE;
  SPI_WRITE(MAX7456VM0_REG);
  SPI_WRITE(_video_mode == VIDEO_MODE_PIXELS_PAL ? 0x40 : 0x0);

  SPI_WRITE(MAX7456ADD_CMAH); // set start address high
  SPI_WRITE(char_address);

  for(uint8_t x = 0; x < sizeof(font->data); x++) // write out 54 bytes of character to shadow ram
  {
    SPI_WRITE(MAX7456ADD_CMAL); // set start address low
    SPI_WRITE(x);
    SPI_WRITE(MAX7456ADD_CMDI);
    SPI_WRITE(font->data[x]);
    LED0_TOGGLE;
  }

  // transfer 54 bytes from shadow ram to NVM
  SPI_WRITE(MAX7456ADD_CMM);
  SPI_WRITE(MAX7456WRITE_NVR);

  // wait until bit 5 in the status register returns to 0 (12ms)
  while ((SPI_WRITE(MAX7456ADD_STAT) & MAX7456STATUS_REG_NVR_BUSY) != 0x00);

  SPI_WRITE(MAX7456VM0_REG);
  SPI_WRITE(_video_mode == VIDEO_MODE_PIXELS_PAL ? 0x4c : 0x0c);

  MAX_CS_DISABLE;
}

// TEST only, dont compile this in! It's huge!
#ifdef MAX_UPLOAD_FONT_ON_RESET
void max7456UpdateFont()
{
  maxFontChar_t font;
  for(uint8_t x = 0; x < MAX_NVM_FONT_CHAR_COUNT; x++){
    for(uint8_t i = 0; i < MAX_NVM_FONT_CHAR_SIZE; i++){
      font.data[i] = (uint8_t) (FONTDATA[(MAX_NVM_FONT_CHAR_FIELD_SIZE*x)+i]);
    }
    max7456WriteFontChar(x, &font);
    delay(20); // Shouldn't be needed due to status reg wait.
  }
}
#endif //MAX_UPLOAD_FONT_ON_RESET

#endif //MAX_OSD
