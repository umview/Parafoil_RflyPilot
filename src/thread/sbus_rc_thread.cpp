#include "sbus_rc_thread.h"
void *sbus_thread(void *ptr)
{
    sbus_api.init((char *)ptr);
    sbus_api.loop(true); //true for demo; false for normal running
    // NOTIC : to run the demo well, short TX-RX is needed
    return NULL;
}
void start_sbus(const char *sbus_serial)
{
  int rc;
  pthread_t thr;
  if(rc = pthread_create(&thr, NULL, sbus_thread, (void *)sbus_serial))
  {
    printf(" thread cretated failed %d \n", rc);
  }
  printf("sbus thread cretated\n"); 
}