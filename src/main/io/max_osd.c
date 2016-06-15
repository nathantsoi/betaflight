/* @file max_osd.cpp
 * @brief max7456 based osd
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

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include <platform.h>

#ifdef MAX_OSD

#include "version.h"

#include "build_config.h"

#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "common/printf.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/typeconversion.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "io/rc_controls.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#ifdef GPS
#include "io/gps.h"
#include "flight/navigation.h"
#endif

#include "config/runtime_config.h"

#include "config/config.h"

#include "io/max_osd.h"
#include "drivers/max7456_symbols.h"
#include "io/max_osd_messages.h"

// externs
//From mw.c:
extern uint32_t currentTime;

//From rx.c:
extern uint16_t rssi;


// Init (constructor)
static void MaxOsdInit(void);

// Different pages to show
static void PickPage(void);
static void WelcomePage(void);
static void ArmedPage(void);
static void DisarmedPage(void);
static void ConfigPage(void);

// Drawing helpers
static char *itoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos);
static char *itoaUnPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos);
static char *formatTime(uint32_t val, char *str, uint8_t hhmmss);

// Drawing
static void DrawTime(void);
static void DrawMode(void);
static void DrawVoltage(void);
static void DrawAmperage(void);
static void DrawAltitude(void);
static void DrawRSSI(void);
static void DrawHorizon(void);

// Helpers
static void DisplayFor(uint32_t seconds);
static bool ShouldUpdateNow(void);
static uint16_t GetPosition(uint8_t position);
static bool FieldIsVisible(uint8_t position);

// Members
// buffer that we'll use to write to the max4567 driver
static char _line_buffer[LINE];
// an array of all the field display settings
// display info mask is 0xC000 and position info is in 0x01FF
//static uint16_t _screen_position[MAX_OSD_DISPLAY_FIELDS];
// how long to wait until redrawing the screen, used by DisplayFor
static uint32_t _show_screen_until = 0;

void initMaxOSD(maxOsdConfig_t *initialMaxOsdConfig)
{
  maxOsdConfig = initialMaxOsdConfig;
  max7456Init();
  MaxOsdInit();
  WelcomePage();
  max7456DrawScreen();
}

void taskUpdateMaxOSD()
{
  if (!ShouldUpdateNow()) {
    return;
  }
  PickPage();
  max7456DrawScreen();
}

void MaxOsdInit()
{
  uint16_t xx;
  for(xx=0;xx<LINE;xx++) {
    _line_buffer[xx] = ' ';
  }
  //TODO: handle PAL. ps. this just seems wrong, maybe x<LINE06?
  //where en is a position setting index
  //if(maxOsdConfig->videoType == PAL){
  //  uint16_t x = _screen_position[en] & MAX_OSD_POS_MASK;
  //  if (x>LINE06) _screen_position[en] = _screen_position[en] + LINE;
  //  if (x>LINE09) _screen_position[en] = _screen_position[en] + LINE;
  //}
}

/** PAGES **/
void PickPage()
{
  if (1)//(ARMING_FLAG(ARMED))
  {
    ArmedPage();
  }
  else
  {
    DisarmedPage();
    // TODO: Config page
  }
}


void WelcomePage()
{
  max7456WriteString(WELCOME_MSG, LINE04+6);
  DisplayFor(5);
}

void ArmedPage()
{
  DrawTime();
  DrawMode();
  DrawVoltage();
  DrawAmperage();
  DrawAltitude();
  DrawRSSI();
  DrawHorizon();
  //_max.WriteString(_screen_buffer, GetPosition(motorArmedPosition));
  //tfp_sprintf(lineBuffer, "Volts: %d.%1d Cells: %d", vbat / 10, vbat % 10, batteryCellCount);
}

void DisarmedPage()
{
  DrawTime();
  //DrawMode();
  //DrawVoltage();
  //DrawAmperage();
  //DrawAltitude();
  //DrawRSSI();
  //_max.WriteString(_screen_buffer, GetPosition(motorArmedPosition));
  //tfp_sprintf(lineBuffer, "Volts: %d.%1d Cells: %d", vbat / 10, vbat % 10, batteryCellCount);
}

void ConfigPage(void)
{
}

/** END PAGES **/

/** DRAWING HELPERS **/
char *itoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)
{
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = '.';
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *itoaUnPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)
{
// Val to convert
// Return String
// Length
// Decimal position
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = '.';
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *formatTime(uint32_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}
/** END DRAWING HELPERS **/

/** DRAWING **/
void DrawTime()
{
  // TODO: if showing time
  uint16_t position = GetPosition(MAX_OSD_FIELD_TIME);
  bool armed = ARMING_FLAG(ARMED);
  uint32_t onSeconds = currentTime / 1000000;
  uint32_t displaytime;
  if (armed) {
    // TODO:
    //if(((flyTime/60)>=Settings[S_FLYTIME_ALARM])&&(timer.Blink2hz))
    //  return;

    //if(flyTime < 3600) {
    //  screenBuffer[0] = SYM_FLY_M;
    //  displaytime = flyTime;
    //}
    //else {
    //  screenBuffer[0] = SYM_FLY_H;
    //  displaytime = flyTime/60;
    //}
  }
  else {
    if(onSeconds < 3600) {
      _line_buffer[0] = SYM_ON_M;
      displaytime = onSeconds;
    }
    else {
      _line_buffer[0] = SYM_ON_H;
      displaytime = onSeconds/60;
    }
  }
  formatTime(displaytime, _line_buffer+1, 0);
  max7456WriteString(_line_buffer, position);
}

void DrawMode()
{
  if (FieldIsVisible(MAX_OSD_FIELD_MODE)) {
    uint8_t mode_position = GetPosition(MAX_OSD_FIELD_MODE);
    char modechars[2] = {SYM_ACRO, SYM_ACRO1};
    if (IS_RC_MODE_ACTIVE(ANGLE_MODE)) {
      modechars[0] = SYM_STABLE;
      modechars[1] = SYM_STABLE1;
    }
    else if (IS_RC_MODE_ACTIVE(HORIZON_MODE)) {
      modechars[0] = SYM_HORIZON;
      modechars[1] = SYM_HORIZON1;
    }
    max7456WriteChar(modechars[0], mode_position++);
    max7456WriteChar(modechars[1], mode_position++);
    if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
      max7456WriteChar(SYM_AIR, mode_position++);
      max7456WriteChar(SYM_AIR1, mode_position++);
    }
    // TODO: super acro mode
  }
  // TODO: GPS modes
}

void DrawVoltage()
{
  if (!FieldIsVisible(MAX_OSD_FIELD_VOLTAGE)) {
    return;
  }
  uint16_t position = GetPosition(MAX_OSD_FIELD_VOLTAGE);
  // vbat is from battery.c, 0.1V steps, we want it in volts
  // 99.9 volts max
  _line_buffer[0] = SYM_VOLT;
  itoaPadded(vbat*10, _line_buffer+1, 4, 3);
  max7456WriteString(_line_buffer, position);
}

void DrawAmperage()
{
  uint16_t position;
  if (FieldIsVisible(MAX_OSD_FIELD_AMPERAGE)) {
    position = GetPosition(MAX_OSD_FIELD_AMPERAGE);
    _line_buffer[0] = SYM_AMP;
    itoaPadded(amperage, _line_buffer+1, 4, 3);
    max7456WriteString(_line_buffer, position);
  }
  if (FieldIsVisible(MAX_OSD_FIELD_POWER_METER)) {
    uint16_t position = GetPosition(MAX_OSD_FIELD_POWER_METER);
    _line_buffer[0] = SYM_MAH;
    itoa(mAhDrawn,_line_buffer+1,10);
    max7456WriteString(_line_buffer, position);
  }
}

void DrawAltitude()
{
  //altitudeHoldGetEstimatedAltitude();
}

void DrawRSSI()
{
}

void DrawHorizon()
{
  if (!FieldIsVisible(MAX_OSD_FIELD_HORIZON)) {
    return;
  }
  uint16_t position = GetPosition(MAX_OSD_FIELD_HORIZON);

  int rollAngle = attitude.values.roll;
  int pitchAngle = attitude.values.pitch;

  if(pitchAngle>AHIPITCHMAX) pitchAngle=AHIPITCHMAX;
  if(pitchAngle<-AHIPITCHMAX) pitchAngle=-AHIPITCHMAX;
  if(rollAngle>AHIROLLMAX) rollAngle=AHIROLLMAX;
  if(rollAngle<-AHIROLLMAX) rollAngle=-AHIROLLMAX;

  for(uint8_t X=0; X<=8; X++) {
    if (X==4) X=5;
    int Y = (rollAngle * (4-X)) / 64;
    Y -= pitchAngle / 8;
    Y += 41;
    if(Y >= 0 && Y <= 81) {
      uint16_t pos = position -7 + LINE*(Y/9) + 3 - 4*LINE + X;
      max7456_screen_buffer[pos] = SYM_AH_BAR9_0+(Y%9);
    }
  }

  if (!FieldIsVisible(MAX_OSD_FIELD_MAP_MODE)) {
    max7456_screen_buffer[position-1] = SYM_AH_CENTER_LINE;
    max7456_screen_buffer[position+1] = SYM_AH_CENTER_LINE_RIGHT;
    max7456_screen_buffer[position] =   SYM_AH_CENTER;
  }
  if (FieldIsVisible(MAX_OSD_FIELD_HORIZON_SIDEBARS)) {
    // Draw AH sides
    int8_t hudwidth =  AHISIDEBARWIDTHPOSITION;
    int8_t hudheight = AHISIDEBARHEIGHTPOSITION;
    for(int8_t X=-hudheight; X<=hudheight; X++) {
      max7456_screen_buffer[position-hudwidth+(X*LINE)] = SYM_AH_DECORATION;
      max7456_screen_buffer[position+hudwidth+(X*LINE)] = SYM_AH_DECORATION;
    }
    // AH level indicators
    max7456_screen_buffer[position-hudwidth+1] =  SYM_AH_LEFT;
    max7456_screen_buffer[position+hudwidth-1] =  SYM_AH_RIGHT;
  }
}

/** END DRAWING **/

/** HELPERS **/
void DisplayFor(uint32_t seconds)
{
  _show_screen_until = millis() + seconds * 1000;
}

bool ShouldUpdateNow(void)
{
  uint32_t now = millis();
  bool updateNow = (int32_t)(now - _show_screen_until) >= 0;
  return updateNow;
}

uint16_t GetPosition(uint8_t position)
{
  uint16_t val = maxOsdConfig->fieldPosition[position];
  uint16_t ret = val & MAX_OSD_POS_MASK;
  return ret;
}

bool FieldIsVisible(uint8_t position)
{
  uint16_t val = maxOsdConfig->fieldPosition[position];
  uint16_t ret = (val & MAX_OSD_DISPLAY_MASK);
  return (ret == MAX_OSD_DISPLAY_ALWAYS);
}

/** END HELPERS **/

#endif //MAX_OSD

