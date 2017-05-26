/*#######################################
#  Application for Async notifications #
# This app subscribes to async notifi- #
# cations from lis3dh misc device.     #
#  Author : Melwyn Lobo                #
#  Copyright: LX Group                 #
#######################################
****************************************/

#include<stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

int fd = 0;

void input_handler(int dummy)
{
	char buf[128];
	int len;
	int i = 0;
	len = read(fd, buf, 16);
	buf[len] = '\n';
	while(buf[i] != '\n')
		printf("%c", buf[i++]);
	printf("\n");
	return;
}
int main(int argc, char *argv[])
{
	int oflags;
	fd = open("/dev/lis3dh", O_RDWR);
	if(fd < 0) {
		printf("Devices file not present. Insert module\n");
		return 0;
	}
	signal(SIGIO, input_handler);
	fcntl(fd, F_SETOWN, getpid());
	oflags = fcntl(fd, F_GETFL);
	fcntl(fd, F_SETFL, oflags | FASYNC);

	while(1) ;
}
