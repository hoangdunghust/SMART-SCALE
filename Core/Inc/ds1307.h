#ifndef __DS1307_H
#define __DS1307_H

#include "main.h"

#define DS1307_ADDR (0x68 << 1)

typedef struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
}DS1307_Time;

void DS1307_GetTime(DS1307_Time *time);
void DS1307_SetTime(DS1307_Time *time);

#endif
