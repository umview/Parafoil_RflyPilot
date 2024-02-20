#include "mytask_thread.h"

class my_task_typedef mytask;

void * thread_mytask(void * ptr)
{
    mytask.my_task_init();

    for(;;)
    {
        mytask.my_task_run();
        sleep(1);
    }
}


void start_mytask(void)
{
  bool ret = create_thread("mytask", thread_mytask, NULL);

}