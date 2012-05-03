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
# make bsl = Download the hex file to the device, using bootstrap loader

APP=micro_gateway

#Directories
SOURCE_DIR = ../../FreeRTOS/Source
PLATFORM_DIR=../../Platform/micro
PORT_DIR = $(PLATFORM_DIR)
COMMON_DIR =../../Common

# Application options
include app.rules
# Platform options
include $(PLATFORM_DIR)/platform.rules

# Target file name (without extension).
TARGET = $(APP)

# Optimization level
OPT = s

# List additional C source files here. (C dependencies are automatically generated.)

SRC += 
# List additional build options here
CFLAGS += 

# List Assembler source files here.
ASRC += 


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = 


# Optional compiler flags.

#DEBUG_LEVEL=-g

CFLAGS += -O$(OPT) $(DEBUG_LEVEL)


# ---------------------------------------------------------------------------


# Define all object files.
#OBJS = $(notdir $(SRC) ) $(notdir $(ASRC) )
OBJS = $(addprefix .objs/build/root/$(APP)/,$(SRC:.c=.o) ) $(addprefix .objs/build/root/$(APP)/,$(ASRC:.S=.o) )
DEPS = $(addprefix .deps/deps/root/$(APP)/,$(SRC:.c=.d) ) 
# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = $(CFLAGS) -I.
ALL_ASFLAGS = $(ASFLAGS)  -I. -x assembler-with-cpp 

# Build targets:
binary: link $(TARGET).$(FORMAT)
	@echo ""
	@echo "--- Build complete ---"

config_start: 
	@echo "--- Config ---"

config: config_start module_id.h modules_conf.h
	@echo "--- Config done ---"

depend_start: 
	@echo "--- Depend ---"
	
.deps/depend:
	@mkdir -p .deps
	@touch .deps/depend

dep: config depend_start $(DEPS) .deps/depend 
	@cat $(DEPS) > .deps/depend
	@echo "--- Depend done ---"

obj: dep $(OBJS) 
	@echo ""
	@echo "--- Build done ---"

$(TARGET).elf: $(OBJS)
	$(CC) $(ALL_CFLAGS) $(OBJS) --output $@ $(LDFLAGS)
	
link: obj $(TARGET).elf
	@echo ""
	@echo "--- Link done ---"

eeprom: link $(TARGET)_eeprom.$(FORMAT)
	@echo ""
	@echo "--- EEPROM file generated ---"

%_eeprom.$(FORMAT): %.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $(@:.=_eeprom.)

%.$(FORMAT): %.elf
	$(OBJCOPY) -O $(FORMAT)  -R .eeprom  $< $@

clean:
	rm -f $(OBJS) *.elf *.ihex *.srec modules_conf.h module_id.h
	rm -rf .objs .deps


clean-deps: 
	rm -f $(DEPS) .deps/depend
	
.deps/deps/root/$(APP):
	mkdir -p .deps/deps/root/$(APP)

.objs/build/root/$(APP):
	mkdir -p .objs/build/root/$(APP)
	
# Compile: create object files from C source.
.objs/build/root/$(APP)/%.o: %.c
	@echo
	@echo "Compile:" $<
	@mkdir -p $(dir  $(addprefix .objs/build/root/$(APP)/, $(<:.c=.o)) ) 
	$(CC) -c $(ALL_CFLAGS) $< -o $(addprefix .objs/build/root/$(APP)/, $(<:.c=.o)) 

# Create asm source from C.
.objs/build/root/$(APP)/%.s : %.c
	@echo
	@echo "Compile to asm:" $<
	$(CC) -S $(ALL_CFLAGS) $< -o $(addprefix .objs/build/root/$(APP)/, $(<:.c=.s)) 

# Create object files from asm.
.objs/build/root/$(APP)/%.o : %.S
	@echo
	@echo "Assemble:" $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $(addprefix .objs/build/root/$(APP)/, $(<:.S=.o)) 

	
# Create depend files.
.deps/deps/root/$(APP)/%.d: %.c
	@echo "Depend:" $<
	@mkdir -p $(dir  $(addprefix .deps/deps/root/$(APP)/, $(<:.c=.d)) ) 
	@$(CC) -MM $(ALL_CFLAGS) -o .deps/tmp $<
	@echo -n .objs/build/root/$(APP)/$(dir $<) > $@
	@cat .deps/tmp >> $@
#$(addprefix .deps/deps/root/$(APP)/, $(<:.c=.d)) 
	
# Create symbol table.
%.sym: %.elf
	@echo
	@echo "Symbols:" $@
	$(NM) -n $< > $@

# Create listing file.
%.lss: %.elf
	@echo
	@echo "Listing:" $@
	$(OBJDUMP) -h -S $< > $@
	
include .deps/depend
