#ifndef HISTORY_H
#define HISTORY_H


#define MAX_HISTORY 5


#define START_ADDR 0x1000

#define COUNT_ADDR 0x2000



typedef struct
{

float weight;

TimeStamp_t ts;


}LogEntry_t;



#endif
