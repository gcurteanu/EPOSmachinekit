#! gmake

#
# Copyright (C) 2015 Francois Beaulier
#
# This file is part of canfestival, a library implementing the canopen
# stack
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

CC = gcc
CXX = g++
LD = g++
#OPT_CFLAGS = -O2 -Wall -D__DEBUG__
OPT_CFLAGS = -O2
CFLAGS = $(OPT_CFLAGS) $(PROG_CFLAGS)
PROG_CFLAGS =  -DUSE_XENO -I/usr/include/xenomai -D_GNU_SOURCE -D_REENTRANT -D__XENO__
EXE_CFLAGS =  -lnative -L/usr/lib -lpthread_rt -lxenomai -lpthread -lrt -lrtdm -ldl -llinuxcnchal
OS_NAME = Linux
ARCH_NAME = x86
TARGET = unix
CAN_DRIVER = can_socket
TIMERS_DRIVER = timers_xeno
ENABLE_SHARED = 

INCLUDES += -I/usr/local/include/canfestival -I.

ifeq ($(ENABLE_SHARED),1)
LIBS = -L/usr/local/lib -lcanfestival -lcanfestival_$(TARGET)
else
LIBS = -L/usr/local/lib -lcanfestival -lcanfestival_$(TARGET)
endif

OBJS_MASTER = master.o EPOScontrol.o ds302.o dcf.o epos.o

#obj-m += canmaster
#canmaster-objs := $(OBJS_MASTER)
#include /usr/share/linuxcnc/Makefile.modinc

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

all: master modules

master: $(OBJS_MASTER)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(OBJS_MASTER) $(LIBS) $(EXE_CFLAGS)

clean:
	rm -f $(OBJS_MASTER) master canmanager.so

BUILD_VERBOSE = 1

obj-m += canmanager.o
canmanager-objs := canmanager.o EPOScontrol.o dcf.o epos.o ds302.o /usr/local/lib/libcanfestival.a /usr/local/lib/libcanfestival_unix.a

canmanager.c: canmanager.comp
	comp canmanager.comp

include Makefile.modinc
