#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <sys/select.h>
#include <stdarg.h>
#include <termios.h>
#include <ctype.h>
#include <sys/time.h>
#include <time.h>
#include "util.h"

void set_nonblocking(int fd)
{
	unsigned v = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, v | O_NONBLOCK);
}

int open_serial(const char *device, unsigned speed)
{
	int fd;
        struct termios t;

	fd = open(device, O_RDWR | O_NONBLOCK);
	if (fd == -1) {
		return -1;
	}

        if (tcgetattr(fd, &t)) {
		printf("unable to get termios attributes: %s\n", strerror(errno));
		close(fd);
		return -1;
	}
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL;
        cfsetspeed(&t, speed);
        
        if (tcsetattr(fd, TCSANOW, &t)) {
		printf("unable to set termios attributes: %s\n", strerror(errno));
		close(fd);
		return -1;
	}

	set_nonblocking(fd);

	return fd;
}

int open_socket_in(const char *listen_addr, int port)
{
        struct sockaddr_in addr;
	int s;
	struct hostent *host;

        if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("Socket");
		exit(1);
        }

	host = (struct hostent *)gethostbyname(listen_addr);
	
        memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	memcpy(&addr.sin_addr, host->h_addr, host->h_length);

	if (bind(s,(struct sockaddr *)&addr,sizeof(addr)) == -1) {
		printf("port %d - failed to bind: %s\n", port, strerror(errno));
		exit(1);
        }

	set_nonblocking(s);
	return s;	
}


int open_socket_out(const char *server, int port)
{
        struct sockaddr_in addr;
	int s;
	struct hostent *host;

        if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("Socket");
		exit(1);
        }

	host = (struct hostent *)gethostbyname(server);
	
        memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
	memcpy(&addr.sin_addr, host->h_addr, host->h_length);

	if (connect(s,(struct sockaddr *)&addr, sizeof(addr)) != 0) {
		perror("connect");
		exit(1);
	}
	set_nonblocking(s);
	return s;	
}

float constrain(float v, float minv, float maxv)
{
	if (v > maxv) return maxv;
	if (v < minv) return minv;
	return v;
}


uint64_t get_usec(void)
{
	struct timeval tv;
        gettimeofday(&tv, NULL);
        return (1000*1000*(uint64_t)tv.tv_sec) + tv.tv_usec;
}

void swap32(void *p, int n)
{
	uint32_t *f = p;
	while (n--) {
		*f = htonl(*f);
		f++;
	}
}


void swap64(void *p, int n)
{
	union temp64 {
                uint64_t ll;
                uint32_t l[2];
	} t, *f = p;

	while (n--) {
		t.l[0] = htonl(f->l[1]);
		t.l[1] = htonl(f->l[0]);

		f->ll = t.ll;
		f++;
	}
}
