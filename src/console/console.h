#ifndef _CONSOLE_
#define _CONSOLE_
// #include "screen.h"
#include "include.h"
#define MAX_SIZE 1024


#define CONSOLE_ENABLE 1
#define PI 3.1415926535

void *console_task(void *arg);
void start_console(void);
class console_class_typedef
{
public:
    pthread_t console_thread;
    void run(void);
    int exec_command(char* user_input);
};
static void print_prompt(void){
    char cwd[MAX_SIZE];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        printf("vehicle > ");
    }
}

static int read_input(char* str){
    char buf[MAX_SIZE];
    char *ret;
    memset(buf, 0, sizeof(buf));
    // printf("buf is %s\n",buf);
    ret = fgets(buf, MAX_SIZE, stdin); 
    if (strlen(buf) != 1) { 
        if(strlen(buf) != 0)
        {
            strcpy(str, buf); 
            return 1;
        }else{// Len buf == 0
            // printf("vehicle > ");
            return 0;
        } 
    } else {// Len buf == 1
        printf("vehicle > ");
        return 0; 
    }
}

static float my_radians(float deg)
{
    return deg / 180 * PI;
}
static float my_degree(float rad)
{
    return rad * 180 / PI;
}

#endif