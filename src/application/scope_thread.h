#ifndef _SCOPE_THREAD_
#define _SCOPE_THREAD_
#include "include.h"
#include "msg_def.h"
void * thread_scope(void * ptr);
void start_scope(const char *addr);
extern class scope_class scope;
#endif