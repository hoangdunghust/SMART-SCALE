#ifndef HISTORY_H
#define HISTORY_H


#define MAX_HISTORY 3


#define START_ADDR 0xB0

#define COUNT_ADDR 0xD4



typedef struct
{

float weight;

TimeStamp_t ts;


}LogEntry_t;



#endif
