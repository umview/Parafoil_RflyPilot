#include "mytask.h"

void my_task_typedef::my_task_init(void)
{
    cnt = 0;
}

void my_task_typedef::my_task_run(void)
{
    cnt++;
    printf("timestamp %f, %d\n", get_time_now() / 1e6, cnt);
}