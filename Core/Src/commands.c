/*
 * commands.c
 *
 *  Created on: Dec 4, 2019
 *      Author: jerasmus
 */
#include <stdio.h>
#include <stdlib.h>
#include "Utils/cli.h"
#include "Utils/commands.h"

#include "main.h"

extern RTC_HandleTypeDef hrtc;
extern void debug_gps(uint8_t argc, char **argv);

void show_rtc(uint8_t argc, char **argv)
{
  RTC_TimeTypeDef rtc_time = {0};
  RTC_DateTypeDef rtc_date = {0};

  if(argc > 5)
  {
	  rtc_date.Year = atoi(argv[1]) - 2000;
	  rtc_date.Month = atoi(argv[2]);
	  rtc_date.Date = atoi(argv[3]);
	  HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	  rtc_time.Hours = atoi(argv[4]);
	  rtc_time.Minutes = atoi(argv[5]);
	  rtc_time.Seconds = 0;
	  HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
  }

  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
  printf("Time: %04u-%02u-%02u %02u:%02u:%02u\n", rtc_date.Year + 2000, rtc_date.Month, rtc_date.Date, rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);
}

const sTermEntry_t rtcEntry =
{ "rtc", "Show RTC time", show_rtc };


const sTermEntry_t gpsEntry =
{ "gps", "Toggle GPS IGT pin", debug_gps };

const sTermEntry_t *cli_entries[] =
{
      &hEntry,
      &helpEntry,
      &rebootEntry,
      &bootEntry,
      &rtcEntry,
      &gpsEntry,
      0
};
