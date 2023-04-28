#include "scope.h"


void scope_class::init(const char *addr, uint16_t port)
{

  /* 建立udp socket */
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sockfd < 0)
  {
    perror("socket");
    exit(1);
  }

  /* 设置address */

  memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = INADDR_BROADCAST;//inet_addr(addr);
  serveraddr.sin_port = htons(port);
  ret = setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &serveraddr, sizeof(serveraddr));

	if (sockfd < 0)
	{
		perror("fail to socket!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		//return -1;
	}	



}
void scope_class::udp_send(uint8_t *data, int len)
{

	ret = sendto(sockfd, data, len, 0, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
	if(ret == -1)
	{
			printf("udp ret %d\n", ret);

	}
	//close(sockfd);
}
/********************************************************************************************/
void udp_recv_class::init(const char *addr, uint16_t port)
{
  /* 建立udp socket */
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sockfd < 0)
  {
    perror("socket");
    exit(1);
  }

  /* 设置address */

  memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr(addr);
  serveraddr.sin_port = htons(port);
  int ret = bind(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr));




}
void udp_recv_class::udp_recv(uint8_t *data, int length)
{


    static struct sockaddr_in cliaddr;
    static socklen_t  len;
    memset(data, 0, length);
    int rlen = recvfrom(sockfd, data, length, 0, (struct sockaddr*)&cliaddr, &len);
        // printf("client info, ip: %s, port: %d\n",
        //        inet_ntop(AF_INET, &cliaddr.sin_addr.s_addr, ipbuf, sizeof(ipbuf)),
        //        ntohs(cliaddr.sin_port));
        // printf("recvfrom client say: %s\n", buf);

}