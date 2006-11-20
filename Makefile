# cmd

#include ../vars.mk
BINDIR = /home/anita/bin	# install destination for most programs

CC = gcc
CFLAGS = -g
LIBS = -L/usr/local/lib -lncurses

TARGETS = cmd batchcmd

all:	$(TARGETS)

cmd:	newcmd.c newcmdfunc.h
	$(CC) $(CFLAGS) -o cmd newcmd.c $(LIBS)

batchcmd:	batchcmd.c
	$(CC) $(CFLAGS) -o batchcmd batchcmd.c

newcmdfunc.h:	newcmdlist.h
	awk -f newmkcmdfunc.awk newcmdlist.h > newcmdfunc.h

install:	all
	cp $(TARGETS) $(BINDIR)

clean:
	rm -f newcmdfunc.h cmd batchcmd

