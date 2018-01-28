# For multiple programs using a single source file each,
# we can just define 'progs' and create custom targets.
PROGS	=	pkt-husky
LIBNETMAP =

CLEANFILES = $(PROGS) *.o

CC = /work/orangePi/OrangePi_H5SDK/toolchain/gcc-linaro-aarch/bin/aarch64-linux-gnu-gcc
SRCDIR ?= ../..
#VPATH = $(SRCDIR)/apps/pkt-gen

NO_MAN=
CFLAGS = -O2 -pipe
#CFLAGS += -Werror -Wall -Wunused-function
CFLAGS += -Werror 
CFLAGS += -I $(SRCDIR)/sys -I $(SRCDIR)/apps/include
CFLAGS += -Wextra

LDLIBS += -lpthread -lm
ifeq ($(shell uname),Linux)
	LDLIBS += -lrt	# on linux
endif

ifdef WITH_PCAP
LDLIBS += -lpcap
else
CFLAGS += -DNO_PCAP
endif

PREFIX ?= /usr/local

all: $(PROGS)

clean:
	-@rm -rf $(CLEANFILES)

.PHONY: install
install: $(PROGS:%=install-%)

install-%:
	install -D $* $(DESTDIR)/$(PREFIX)/bin/$*

pkt-husky: husky_demo.c
	$(CC) $(CFLAGS) -DBUSYWAIT $^ -o $@ $(LDLIBS)
