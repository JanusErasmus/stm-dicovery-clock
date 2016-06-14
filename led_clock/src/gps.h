#ifndef _GPS_H
#define _GPS_H
#include <stm32f10x.h>

void initGPS();

bool gps_getTime(uint8_t *hour, uint8_t *min);

#endif //_TERMINAL_H

