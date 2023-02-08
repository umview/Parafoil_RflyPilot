#include "scope.h"


void scope_class::init(const char *addr, uint16_t port)
{
	socklen_t len = sizeof(struct sockaddr_in);

	memset(&serveraddr, 0, len);

	//填充upd 服务器自身的信息
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(port);
	serveraddr.sin_addr.s_addr = inet_addr(addr);
//192.168.199.152  BUAA PC
//192.168.43.233 Android E603
//192.168.199.166 BUAA Desktop
	//注意此处是udp协议,用SOCK_DGRAM
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	if (sockfd < 0)
	{
		perror("fail to socket");
		//return -1;
	}	

	//for logger

    // tlog_close(log);
    // tlog_exit();

}
void scope_class::udp_send(uint8_t *data, int len)
{

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	sendto(sockfd, data, len, 0, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
	//recvfrom(sockfd, buf, 0, 0, NULL, NULL);
	//printf("%s\n", buf);

	close(sockfd);
}
