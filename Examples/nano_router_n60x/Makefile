# Usage:
#
# make binary = Make binary file.
#
# make eeprom = Make eeprom file.
#
# make clean = Clean object files.
#
# make clean-deps = Clean object files and dependencies.
#
# make program = Download the hex file to the device

APP=nano_router_n60x

#Directories
SOURCE_DIR = ../../FreeRTOS/Source
PLATFORM_DIR=../../Platform/nano
PORT_DIR = $(PLATFORM_DIR)
COMMON_DIR =../../Common

# Application options
include app.rules
# Platform options
include $(PLATFORM_DIR)/platform.rules

# Target file name (without extension).
TARGET = $(APP)

# Optimization level
OPT =
#OPT = s

# C sources
SRC +=
# $(PLATFORM_DIR)/uart.c 

# Additional build options
#CFLAGS += -DRF_ACK_HW
CFLAGS += -DNRP_UART_DMA -DNRP_UART_DMA_TX
# -DNRP_UART_DMA_RX
# Assembler source files
ASRC += 


# Extra include directories
EXTRAINCDIRS = 

CFLAGS +=
	
program: binary
	../../Tools/nano_programmer/nano_programmer -d $(PGM_PORT) --program $(TARGET).$(FORMAT)

include $(PLATFORM_DIR)/platform-compile.rules
