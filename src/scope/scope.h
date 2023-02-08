#ifndef _SCOPE_
#define _SCOPE_
#include "include.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>


class scope_class
{
public:
	int sockfd;
	char buf[32];
	struct sockaddr_in serveraddr;	
	void init(const char *addr, uint16_t port);
	void udp_send(uint8_t *data, int len);
};
#endif