/* @file max_osd.h
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

#pragma once

#include "drivers/max7456.h"

#ifdef MAX_OSD

#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees
#define AHISIDEBARWIDTHPOSITION 7
#define AHISIDEBARHEIGHTPOSITION 3

/** CONFIG **/
enum MAX_OSD_FIELDS {
  MAX_OSD_FIELD_GPS_NUM_SAT,
  MAX_OSD_FIELD_GPS_TIME,
  MAX_OSD_FIELD_GPS_DIRECTION_HOME,
  MAX_OSD_FIELD_GPS_DISTANCE_HOME,
  MAX_OSD_FIELD_GPS_SPEED,
  MAX_OSD_FIELD_GPS_ANGLE_HOME,
  MAX_OSD_FIELD_GPS_ALTITUDE,
  MAX_OSD_FIELD_GPS_HEADING,
  MAX_OSD_FIELD_GPS_HEADING_GRAPH,
  MAX_OSD_FIELD_GPS_LAT,
  MAX_OSD_FIELD_GPS_LON,
  MAX_OSD_FIELD_MAP_MODE,
  MAX_OSD_FIELD_MAP_CENTER,
  MAX_OSD_FIELD_BARO,
  MAX_OSD_FIELD_CLIMB,
  MAX_OSD_FIELD_PITCH_ANGLE,
  MAX_OSD_FIELD_ROLL_ANGLE,
  MAX_OSD_FIELD_SENSOR,
  MAX_OSD_FIELD_THROTTLE,
  MAX_OSD_FIELD_MOTOR_ARMED,
  MAX_OSD_FIELD_RSSI,
  MAX_OSD_FIELD_TEMP,
  MAX_OSD_FIELD_VOLTAGE,
  MAX_OSD_FIELD_AMPERAGE,
  MAX_OSD_FIELD_MODE,
  MAX_OSD_FIELD_POWER_METER,
  MAX_OSD_FIELD_WATT,
  MAX_OSD_FIELD_TIME,
  MAX_OSD_FIELD_HORIZON,
  MAX_OSD_FIELD_HORIZON_SIDEBARS,
  MAX_OSD_FIELD_GIMBAL,
  MAX_OSD_FIELD_SPORT,
  MAX_OSD_FIELD_CALLSIGN,
  MAX_OSD_FIELD_DEBUG,
  // Count of fields
  MAX_OSD_DISPLAY_FIELDS
};

enum MAX_VIDEO_TYPES { NTSC, PAL };

typedef struct maxOsdConfig_s {
  uint8_t videoType;
  uint16_t fieldPosition[MAX_OSD_DISPLAY_FIELDS];
} maxOsdConfig_t;

maxOsdConfig_t *maxOsdConfig;

#define MAX_OSD_POS_MASK        0x01FF
#define MAX_OSD_DISPLAY_MASK    0xC000
#define MAX_OSD_DISPLAY_ALWAYS  0xC000
#define MAX_OSD_DISPLAY_NEVER   0x0000


/** END CONFIG **/

#define LINE      30
#define LINE01    0
#define LINE02    30
#define LINE03    60
#define LINE04    90
#define LINE05    120
#define LINE06    150
#define LINE07    180
#define LINE08    210
#define LINE09    240
#define LINE10    270
#define LINE11    300
#define LINE12    330
#define LINE13    360
#define LINE14    390
#define LINE15    420
#define LINE16    450

void initMaxOSD(maxOsdConfig_t *initialMaxOsdConfig);
void taskUpdateMaxOSD(void);

#endif //MAX_OSD
