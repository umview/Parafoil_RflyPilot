#include "console_thread.h"

class console_class_typedef console;
void *console_task(void *arg)
{
    //get_hight_Pri(1);
    console.run();
    return 0;
}

void start_console(void)
{
    printf("start console");    
    int ret = pthread_create(&console.console_thread, NULL, console_task, NULL);
    if (ret != 0)
    {
        std::cout << "pthread_create error: error_code =" << ret << std::endl;
    }
}
