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

  bool ret = create_thread("console", console_task, NULL);

}
