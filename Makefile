CC=gcc
CFLAGS=-g -std=c99 -D_XOPEN_SOURCE=600 -Wall -I../mavlink/include
LIBS=-lm -lreadline

all: mavproxy

SRC=mavproxy.c util.c editfile.c
OBJ=$(SRC:.c=.o)


mavproxy: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LIBS)

clean:
	rm -f mavproxy *~ *.o
