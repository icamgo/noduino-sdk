#
# Copyright (c) 2019 MaiKe Labs
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
#  1. Set the line containing "MCU" to match your board's processor.
#     Older one's are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000.
#
#  2. Type "make" to compile/verify your program.
#
#  3. Type "make flash" to upload your program into your board
#

TARGET = quark.ino

SDK_BASE		?= ../..

ARDUINO_CORES	:= $(SDK_BASE)/arch/avr/cores
LIB_DIR			:= $(SDK_BASE)/arch/avr/libs

TOOLCHAIN_DIR = $(SDK_BASE)/toolchain/
UNAME_S := $(shell uname -s)

ifeq ($(OS),Windows_NT)
PORT = COM7
else
	ifeq ($(UNAME_S),Linux)
		PORT ?= /dev/ttyUSB0
	endif
	ifeq ($(UNAME_S),Darwin)
		PORT ?= /dev/cu.SLAB_USBtoUART
	endif
endif

UPLOAD_RATE = 57600
AVRDUDE_PROGRAMMER = stk500v1
MCU = atmega328p
F_CPU = 16000000

VERSION=187

ifeq ($(OS),Windows_NT)
AVR_TOOLS_PATH 	= $(TOOLCHAIN_DIR)/avr-gcc/bin
AVRDUDE_PATH = $(TOOLCHAIN_DIR)/avrdude
AVRDUDE_CONF_PATH = $(TOOLCHAIN_DIR)/avrdude
else
AVR_TOOLS_PATH 	= $(TOOLCHAIN_DIR)/avr-gcc/bin
AVRDUDE_PATH = $(TOOLCHAIN_DIR)/avrdude
AVRDUDE_CONF_PATH = $(TOOLCHAIN_DIR)/avrdude
endif

#Note that if your program has dependencies other than those
#already listed below, you will need to add them accordingly.
C_MODULES =  										\
$(ARDUINO_CORES)/wiring_pulse.c 						\
$(ARDUINO_CORES)/wiring_analog.c 						\
$(ARDUINO_CORES)/wiring.c 								\
$(ARDUINO_CORES)/hooks.c 								\
$(ARDUINO_CORES)/wiring_digital.c 						\
$(ARDUINO_CORES)/WInterrupts.c 							\
$(LIB_DIR)/Wire/src/utility/twi.c						\
$(ARDUINO_CORES)/wiring_shift.c

CXX_MODULES = 										\
$(ARDUINO_CORES)/HardwareSerial.cpp 					\
$(ARDUINO_CORES)/HardwareSerial0.cpp 					\
$(ARDUINO_CORES)/Print.cpp 								\
$(ARDUINO_CORES)/Stream.cpp 							\
$(ARDUINO_CORES)/WMath.cpp 								\
$(ARDUINO_CORES)/WString.cpp 							\
$(ARDUINO_CORES)/main.cpp								\
$(LIB_DIR)/SPI/src/SPI.cpp								\
$(LIB_DIR)/MCP342x/src/MCP342x.cpp						\
$(LIB_DIR)/Wire/src/Wire.cpp							\
$(LIB_DIR)/LowPower/LowPower.cpp						\
$(LIB_DIR)/SX127x/sx1272.cpp							\
$(LIB_DIR)/SHT2X/sht2x.cpp								\
$(LIB_DIR)/BMP180/bmp180.cpp							\
$(LIB_DIR)/TSL2561/tsl2561.cpp							\
$(LIB_DIR)/MH-Z16/mhz16.cpp								\
./gps.cpp											\
./vbat.cpp												

CXX_APP = main.cpp
MODULES = $(C_MODULES) $(CXX_MODULES)
SRC = $(C_MODULES)
CXXSRC = $(CXX_MODULES) $(CXX_APP)
FORMAT = ihex

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#DEBUG = stabs
DEBUG =

OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)L -DARDUINO=$(VERSION) -DARDUINO_ARCH_AVR
CXXDEFS = -DF_CPU=$(F_CPU)L -DARDUINO=$(VERSION) -DARDUINO_ARCH_AVR

# Place -I options here
CINCS = -I $(ARDUINO_CORES)
CXXINCS = -I $(ARDUINO_CORES)

CINCS += -I  $(LIB_DIR)/SPI/src						\
			-I $(LIB_DIR)/EEPROM/src				\
			-I $(LIB_DIR)/LowPower					\
			-I $(LIB_DIR)/Wire/src					\
			-I $(LIB_DIR)/Wire/src/utility			\
			-I $(LIB_DIR)/SX127x					\
			-I $(LIB_DIR)/SHT2X						\
			-I $(LIB_DIR)/BMP180					\
			-I $(LIB_DIR)/TSL2561					\
			-I $(LIB_DIR)/MH-Z16					\
			-I .									

CXXINCS += -I  $(LIB_DIR)/SPI/src					\
			-I $(LIB_DIR)/EEPROM/src				\
			-I $(LIB_DIR)/LowPower					\
			-I $(LIB_DIR)/Wire/src					\
			-I $(LIB_DIR)/SX127x					\
			-I $(LIB_DIR)/SHT2X						\
			-I $(LIB_DIR)/BMP180					\
			-I $(LIB_DIR)/TSL2561					\
			-I $(LIB_DIR)/MH-Z16					\
			-I .									

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
#CSTANDARD = -std=gnu99

CDEBUG = -g$(DEBUG)
#CWARN = -Wall -Wstrict-prototypes
#CWARN = -Wall   # show all warnings
CWARN = -w #suppress all warnings

####CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CTUNING = -ffunction-sections -fdata-sections
CXXTUNING = -fno-exceptions -ffunction-sections -fdata-sections
CEXTRA = -MMD -DUSB_PID=null -DUSB_VID=null -DARDUINO=187

CFLAGS = $(CDEBUG) -O$(OPT) $(CWARN) $(CTUNING) $(CDEFS) $(CINCS) $(CSTANDARD) $(CEXTRA)
CXXFLAGS = $(CDEBUG) -O$(OPT) $(CWARN) $(CXXTUNING) $(CDEFS) $(CINCS) $(CEXTRA)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs
LDFLAGS = -O$(OPT) -lm -Wl,--gc-sections


# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_MAIN = -U flash:w:build/main.hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:eeprom.bin:r

AVRDUDE_FLAGS = -V -F -C $(TOOLCHAIN_DIR)/etc/avrdude.conf \
-c arduino -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) \
-b $(UPLOAD_RATE) -D

# Program settings
CC = $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
LD = $(AVR_TOOLS_PATH)/avr-gcc
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
AVRDUDE = $(AVRDUDE_PATH)/avrdude
REMOVE = rm -f
MV = mv -f

# Define all object files.
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)
OBJ_MODULES = $(C_MODULES:.c=.o) $(CXX_MODULES:.cpp=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = $(CFLAGS) -mmcu=$(MCU)
ALL_CXXFLAGS = $(CXXFLAGS) -mmcu=$(MCU)
ALL_ASFLAGS = -x assembler-with-cpp $(ASFLAGS) -mmcu=$(MCU)
ALL_LDFLAGS = $(LDFLAGS) -mmcu=$(MCU)

# Default target.
all: build_files rebuild build sizeafter

build: elf hex

build/main.o: 
	test -d build || mkdir build
	echo '#include "Arduino.h"' > build/main.cpp
	cat $(TARGET) >> build/main.cpp
	cat $(ARDUINO_CORES)/main.cpp >> build/main.cpp
	$(CXX) -c $(ALL_CXXFLAGS) build/main.cpp -o build/main.o
	echo $(CXX_MODULES)

rebuild:
	$(REMOVE) build/main.*

elf: build/main.elf
hex: build/main.hex
eep: build/main.eep
lss: build/main.lss
sym: build/main.sym

# Program the device.
upload: build/main.hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_MAIN)

flash: build/main.hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_MAIN)

eeprom: eeprom.bin
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_EEPROM)

	# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) build/main.hex
ELFSIZE = $(SIZE)  build/main.elf
sizebefore:
	@if [ -f build/main.elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter:
	@if [ -f build/main.elf ]; then echo; echo $(MSG_SIZE_AFTER); $(HEXSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000


coff: build/main.elf
	$(COFFCONVERT) -O coff-avr build/main.elf main.cof


extcoff: main.elf
	$(COFFCONVERT) -O coff-ext-avr build/main.elf main.cof


.SUFFIXES: .elf .hex .eep .lss .sym

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	$(OBJCOPY) -O $(FORMAT) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--no-change-warnings \
	--change-section-lma .eeprom=0 $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

# Link: create ELF output file from library.
build/main.elf: build/main.o build/core.a
	$(LD) $(ALL_LDFLAGS) -o $@ build/main.o build/core.a

build/core.a: $(OBJ_MODULES)
	@for i in $(OBJ_MODULES); do echo $(AR) rcs build/core.a $$i; $(AR) rcs build/core.a $$i; done


# Compile: create object files from C++ source files.
.cpp.o:
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@

# Compile: create object files from C source files.
.c.o:
	$(CC) -c $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C source files.
.c.s:
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
.S.o:
	$(CC) -c $(ALL_ASFLAGS) $< -o $@


# Automatic dependencies
%.d: %.c
	$(CC) -M $(ALL_CFLAGS) $< | sed "s;$(notdir $*).o:;$*.o $*.d:;" > $@

%.d: %.cpp
	$(CXX) -M $(ALL_CXXFLAGS) $< | sed "s;$(notdir $*).o:;$*.o $*.d:;" > $@


# Target: clean project.
clean:
	$(REMOVE) build/main.hex build/main.eep build/main.cof build/main.elf \
	build/main.map build/main.sym build/main.o build/main.lss build/core.a \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d) \
	build/main.cpp
	$(REMOVE) -r build

.PHONY:	all rebuild build elf hex eep lss sym program coff extcoff clean build_files sizebefore sizeafter

#include $(SRC:.c=.d)
#include $(CXXSRC:.cpp=.d)
