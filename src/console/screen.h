#ifndef _SCREEN_H_
#define _SCREEN_H_

#include "include.h"

void *screen_task(void *arg);
void start_screen(void);

class screen
{
public:
    int lineMoveUp = {0};
    int printline(uint64_t timestamp, float data1, float data2, float data3, const char *string1, const char *string2, const char *string3);
    int endscreen(void);
    int startscreen(void);
    int outputscreen(void);
};
// extern class screen Screen;

#endif