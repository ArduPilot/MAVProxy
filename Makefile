CC=gcc
CFLAGS=-g -std=c99 -D_XOPEN_SOURCE=600 -Wall

all: mavproxy

mavproxy: mavproxy.c
	$(CC) $(CFLAGS) -o $@ mavproxy.c util.c -Wall -I../mavlink/include -lm

clean:
	rm -f mavproxy *~

