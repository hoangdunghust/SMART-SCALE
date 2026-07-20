#ifndef USER_H
#define USER_H


#include "stdint.h"


#define MAX_USER 4

#define USER_START_ADDR 0x00



typedef struct
{
    uint8_t day;
    uint8_t month;
    uint8_t year;

    uint8_t hour;
    uint8_t minute;
    uint8_t second;

}TimeStamp_t;



typedef struct
{
    float weight;

    TimeStamp_t ts;

}History_t;



typedef struct
{

    uint8_t uid[4];


    History_t history[3];


    uint8_t count;


}UserProfile_t;



#define USER_ADDR(index) \
(USER_START_ADDR + index*sizeof(UserProfile_t))



#endif
