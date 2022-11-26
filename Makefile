all: relais

clean:
	rm -f relais *~

CFLAGS = -g -O -pthread
LDFLAGS = -g -pthread
