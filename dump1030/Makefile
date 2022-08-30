CFLAGS?=-O2 -g -Wall -W $(shell pkg-config --cflags librtlsdr)
LDLIBS+=$(shell pkg-config --libs librtlsdr) -lpthread -lm -lstdc++
CC?=gcc
PROGNAME=dump1030

all: dump1030

%.o: %.c
	$(CC) $(CFLAGS) -c $<

dump1030: dump1030.o
	$(CC) -g -o dump1030 dump1030.o $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o dump1030
