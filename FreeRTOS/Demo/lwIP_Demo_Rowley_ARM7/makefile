#	FreeRTOS.org V4.1.3 - Copyright (C) 2003-2006 Richard Barry.
#
#	This file is part of the FreeRTOS.org distribution.
#
#	FreeRTOS.org is free software; you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation; either version 2 of the License, or
#	(at your option) any later version.
#
#	FreeRTOS.org is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU General Public License for more details.
#
#	You should have received a copy of the GNU General Public License
#	along with FreeRTOS.org; if not, write to the Free Software
#	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
#	A special exception to the GPL can be applied should you wish to distribute
#	a combined work that includes FreeRTOS.org, without being obliged to provide
#	the source code for any proprietary components.  See the licensing section 
#	of http://www.FreeRTOS.org for full details of how and when the exception
#	can be applied.
#
#	***************************************************************************
#	See http://www.FreeRTOS.org for documentation, latest information, license 
#	and contact details.  Please ensure to read the configuration and relevant 
#	port sections of the online documentation.
#	***************************************************************************

CC=arm-elf-gcc
OBJCOPY=arm-elf-objcopy
ARCH=arm-elf-ar
CRT0=boot.s
DEBUG=-g
OPTIM=-O0
LDSCRIPT=atmel-rom.ld

#
# CFLAGS common to both the THUMB and ARM mode builds
#

CFLAGS= \
-I.  \
-I./EMAC  \
-I../Common/include  \
-I./USB  \
-I./lwip-1.1.0/src/include  \
-I./lwip-1.1.0/contrib/port/FreeRTOS/AT91SAM7X  \
-I../../Source/include  \
-I../../Source/portable/GCC/ARM7_AT91SAM7S  \
-I./lwip-1.1.0/src/include/ipv4 \
-Wall  \
-Wextra  \
-Wstrict-prototypes  \
-Wmissing-prototypes  \
-Wmissing-declarations  \
-Wno-strict-aliasing  \
-D SAM7_GCC  \
-D THUMB_INTERWORK \
-mthumb-interwork \
-mcpu=arm7tdmi  \
-T$(LDSCRIPT) \
$(DEBUG)  \
$(OPTIM)

THUMB_FLAGS=-mthumb
LINKER_FLAGS=-Xlinker -ortosdemo.elf -Xlinker -M -Xlinker -Map=rtosdemo.map

#
# Source files that can be built to THUMB mode.
#
FREERTOS_THUMB_SRC= \
  ../../Source/tasks.c \
  ../../Source/queue.c \
  ../../Source/list.c \
  ../../Source/portable/GCC/ARM7_AT91SAM7S/port.c

DEMO_APP_THMUB_SRC= \
  ../../Source/portable/MemMang/heap_2.c \
  ParTest/ParTest.c \
  main.c \
  ../Common/Minimal/flash.c \
  ../Common/Minimal/BlockQ.c \
  ../Common/Minimal/integer.c \
  ../Common/Minimal/PollQ.c \
  ../Common/Minimal/semtest.c \
  BasicWEB.c \
  USB/USB-CDC.c 

LWIP_THUMB_SRC= \
  lwip-1.1.0/src/core/tcp_out.c \
  lwip-1.1.0/src/core/inet.c \
  lwip-1.1.0/src/core/mem.c \
  lwip-1.1.0/src/core/memp.c \
  lwip-1.1.0/src/core/netif.c \
  lwip-1.1.0/src/core/pbuf.c \
  lwip-1.1.0/src/core/raw.c \
  lwip-1.1.0/src/core/stats.c \
  lwip-1.1.0/src/core/sys.c \
  lwip-1.1.0/src/core/tcp.c \
  lwip-1.1.0/src/core/tcp_in.c \
  lwip-1.1.0/src/core/ipv4/ip.c \
  lwip-1.1.0/src/core/ipv4/ip_addr.c \
  lwip-1.1.0/src/core/ipv4/icmp.c \
  lwip-1.1.0/src/api/tcpip.c \
  lwip-1.1.0/src/api/api_msg.c \
  lwip-1.1.0/src/api/err.c \
  lwip-1.1.0/src/api/api_lib.c \
  lwip-1.1.0/src/netif/etharp.c \
  lwip-1.1.0/contrib/port/FreeRTOS/AT91SAM7X/sys_arch.c \
  lwip-1.1.0/src/netif/ethernetif.c \
  EMAC/SAM7_EMAC.c \
  lwip-1.1.0/src/core/udp.c \
  lwip-1.1.0/src/core/ipv4/ip_frag.c

#
# Source files that must be built to ARM mode.
#
ARM_SRC= \
  ../../Source/portable/GCC/ARM7_AT91SAM7S/portISR.c \
  EMAC/SAM7_EMAC_ISR.c \
  USB/USBIsr.c \
  Cstartup_SAM7.c  


#
# Define all object files.
#
ARM_OBJ = $(ARM_SRC:.c=.o)
FREERTOS_THUMB_OBJ = $(FREERTOS_THUMB_SRC:.c=.o)
DEMO_APP_THMUB_OBJ = $(DEMO_APP_THMUB_SRC:.c=.o)
LWIP_THUMB_OBJ = $(LWIP_THUMB_SRC:.c=.o)

rtosdemo.bin : rtosdemo.elf
	$(OBJCOPY) rtosdemo.elf -O binary rtosdemo.bin

rtosdemo.hex : rtosdemo.elf
	$(OBJCOPY) rtosdemo.elf -O ihex rtosdemo.hex

rtosdemo.elf : $(ARM_OBJ) $(DEMO_APP_THMUB_OBJ) $(LWIP_THUMB_OBJ) $(FREERTOS_THUMB_OBJ) $(CRT0) Makefile FreeRTOSConfig.h
	$(CC) $(CFLAGS) $(ARM_OBJ) $(DEMO_APP_THMUB_OBJ) $(LWIP_THUMB_OBJ) $(FREERTOS_THUMB_OBJ) -nostartfiles $(CRT0) $(LINKER_FLAGS)

$(DEMO_APP_THMUB_OBJ)  : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(THUMB_FLAGS) $(CFLAGS) $< -o $@

$(LWIP_THUMB_OBJ)  : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(THUMB_FLAGS) $(CFLAGS) $< -o $@

$(FREERTOS_THUMB_OBJ)  : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(THUMB_FLAGS) $(CFLAGS) $< -o $@

$(ARM_OBJ) : %.o : %.c $(LDSCRIPT) Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

clean :
	touch Makefile

