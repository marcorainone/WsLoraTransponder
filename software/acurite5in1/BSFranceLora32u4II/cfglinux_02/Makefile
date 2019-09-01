#
# bsfcfg Makefile
# serial configurator for Acurite transponder with BsFrance Lora32u4II 
#

CC      = gcc
CFLAGS  = -g -I../src
AR	    = ar
ARFLAGS = rcv
RM      = rm -f

default: all

all: bsfcfg

bsfcfg: bsfcfg.c
	$(CC) $(CFLAGS) -o bsfcfg bsfcfg.c dictionary.c iniparser.c -I./ 

clean veryclean:
	$(RM) bsfcfg bsf32u4.ini



