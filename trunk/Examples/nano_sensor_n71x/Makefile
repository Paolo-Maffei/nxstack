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

APP=nano_sensor_n71x

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
SRC += $(PLATFORM_DIR)/adc.c 
# $(PLATFORM_DIR)/uart.c 

# Additional build options
#CFLAGS += -DCC2430_EM

# Assembler source files
ASRC += 


# Extra include directories
EXTRAINCDIRS = 

CFLAGS +=
	
program: binary
	../../Tools/nano_programmer/nano_programmer -d $(PGM_PORT) --program $(TARGET).$(FORMAT)

include $(PLATFORM_DIR)/platform-compile.rules
