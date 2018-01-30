CC=gcc
CFLAGS=-Wall -O2

pmbus_peek: pmbus_peek.c
	$(CC) $(CFLAGS) -o pmbus_peek pmbus_peek.c

clean:
	rm -f pmbus_peek
