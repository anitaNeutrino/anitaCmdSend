# cmd

#include ../vars.mk
BINDIR = /home/anita/bin	# install destination for most programs

CC = gcc
CFLAGS = -g
LIBS = -L/usr/local/lib -lncurses -lm

TARGETS = cmdTui batchcmd

all:	$(TARGETS)

cmdTui:	newcmdfunc.h newcmd.o cmdUtilDef.o 
	$(CC) $(CFLAGS) newcmd.o cmdUtilDef.o $(LIBS) -o cmdTui 

batchcmd:	batchcmd.c
	$(CC) $(CFLAGS) -o batchcmd batchcmd.c

newcmdfunc.h:	newcmdlist.h
	awk -f newmkcmdfunc.awk newcmdlist.h > newcmdfunc.h

install:	all
	cp $(TARGETS) $(BINDIR)

clean:
	rm -f newcmdfunc.h cmd batchcmd *.o

