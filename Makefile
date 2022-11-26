all: relais

clean:
	rm -f relais *~

CFLAGS = -g -O -pthread -Wall -Wextra
LDFLAGS = -g -pthread
