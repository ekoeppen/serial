CFLAGS = -g -O2 -Wall -Wno-unused-result -std=c99 -D_DEFAULT_SOURCE
SRCS = main.c
PROG = serial

all: serial

clean:
	rm -f $(PROG)

$(PROG): $(SRCS) Makefile
	$(CC) $(CFLAGS) -o $@ $(SRCS)
