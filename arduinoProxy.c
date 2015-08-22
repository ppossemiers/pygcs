#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 5556

void send2Drone(char *cmd)
{
   struct sockaddr_in receiver_addr;
   int sock_fd;
   sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   receiver_addr.sin_family = AF_INET;
	  if( inet_aton( "192.168.1.1",  &receiver_addr.sin_addr )== 0) {
		  printf("Init failed\n");
	   close(sock_fd);
		  return;
	  }
   receiver_addr.sin_port = htons( PORT );
   cmd[strlen(cmd) - 1] = '\r';
   sendto(sock_fd, cmd, strlen(cmd), 0,(struct sockaddr*)&receiver_addr,sizeof(receiver_addr));
   close(sock_fd);
}

int main () {

 while(1) {
  char cmd[50];
  fgets(cmd, 50, stdin);
  if (cmd[0] == 'A' && cmd[1] == 'T') {
	  send2Drone(cmd);
  }
 }
}
