CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -Wno-format-truncation
LDLIBS  = -lasound -lm
PREFIX  ?= /usr/local

autosampler: autosampler.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

install: autosampler
	install -D -m 755 autosampler $(DESTDIR)$(PREFIX)/bin/autosampler

clean:
	rm -f autosampler

.PHONY: install clean
