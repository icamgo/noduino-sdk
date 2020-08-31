#
# Copyright (c) 2019 MaiKe Labs
#
# This makefile allows you to build sketches from the command line
#
#  1. Type "make" to compile/verify your program.
#
#  2. Type "make flash" to upload your program into your board
#

TARGET		= main.cpp

SDK_BASE	?= ../../..
ARDUINO_CORES	:= $(SDK_BASE)/arch/efm32/cores
ARDUINO_BOARD	:= $(SDK_BASE)/arch/efm32/variants/efm32zg110
#ARDUINO_BOARD	:= $(SDK_BASE)/arch/efm32/variants/efm32hg110
LIB_DIR		:= $(SDK_BASE)/arch/efm32/libs

MCU = cortex-m0plus
#F_CPU = 7000000
F_CPU = 14000000

VERSION=187

ifeq ($(OS),Windows_NT)
TOOLCHAIN_DIR = $(SDK_BASE)/toolchain/gcc-arm-none-eabi/bin
OPENOCD = $(SDK_BASE)/toolchain/openocd-nt/bin/openocd
PORT = COM7
else
TOOLCHAIN_DIR = /usr/share/arduino
	ifeq ($(UNAME_S),Linux)
		PORT ?= /dev/ttyUSB0
	endif
	ifeq ($(UNAME_S),Darwin)
		PORT ?= /dev/cu.SLAB_USBtoUART
	endif
endif

OPENOCD_CFG = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; program build/main.bin verify reset; shutdown"
OPENOCD_RESET = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset; shutdown"

CC	:= $(TOOLCHAIN_DIR)/arm-none-eabi-gcc
CXX	:= $(TOOLCHAIN_DIR)/arm-none-eabi-g++
LD	:= $(TOOLCHAIN_DIR)/arm-none-eabi-gcc
OBJCOPY	:= $(TOOLCHAIN_DIR)/arm-none-eabi-objcopy
OBJDUMP	:= $(TOOLCHAIN_DIR)/arm-none-eabi-objdump
AR	:= $(TOOLCHAIN_DIR)/arm-none-eabi-ar
SIZE	:= $(TOOLCHAIN_DIR)/arm-none-eabi-size
NM	:= $(TOOLCHAIN_DIR)/arm-none-eabi-nm

RM = rm -f
MV = mv -f

C_MODULES =  										\
$(ARDUINO_CORES)/hooks.c 								\
$(ARDUINO_CORES)/itoa.c									\
$(ARDUINO_CORES)/syscalls.c 							\
$(ARDUINO_CORES)/wiring_shift.c							\
$(ARDUINO_CORES)/efm32_hal/system.c						\
$(ARDUINO_CORES)/efm32_hal/startup.c					\
$(ARDUINO_CORES)/efm32_hal/emlib_part1.c				\
$(ARDUINO_CORES)/efm32_hal/emlib_part2.c				\
$(ARDUINO_CORES)/efm32/gpiointerrupt.c					\
$(ARDUINO_CORES)/efm32/efm32pwm.c						\
$(ARDUINO_CORES)/efm32/efm32init.c						\
$(ARDUINO_CORES)/efm32/efm32gpio.c						\
$(ARDUINO_CORES)/efm32/core_callback.c					\
$(ARDUINO_CORES)/avr/dtostrf.c							\
$(LIB_DIR)/rtcdrv/rtcdriver.c

#$(ARDUINO_CORES)/efm32/efm32dac.c						\

CXX_MODULES = 										\
$(ARDUINO_CORES)/abi.cpp 								\
$(ARDUINO_CORES)/new.cpp 								\
$(ARDUINO_CORES)/Print.cpp 								\
$(ARDUINO_CORES)/RingBuffer.cpp 						\
$(ARDUINO_CORES)/Stream.cpp 							\
$(ARDUINO_CORES)/WMath.cpp 								\
$(ARDUINO_CORES)/WString.cpp 							\
$(ARDUINO_CORES)/efm32/HardwareSerial.cpp				\
$(ARDUINO_CORES)/efm32/debug.cpp						\
$(ARDUINO_CORES)/efm32/efm32adc.cpp						\
$(ARDUINO_CORES)/main.cpp								\
$(LIB_DIR)/pt1000/pt1000.cpp

CXX_APP = main.cpp
MODULES = $(C_MODULES) $(CXX_MODULES)
SRC = $(C_MODULES)
CXXSRC = $(CXX_MODULES) $(CXX_APP)

CDEFS = -DF_CPU=$(F_CPU)L -DARDUINO=$(VERSION) -DARDUINO_ARCH_ARM
CXXDEFS = -DF_CPU=$(F_CPU)L -DARDUINO=$(VERSION) -DARDUINO_ARCH_ARM

CINCS = -I$(ARDUINO_CORES) -I$(ARDUINO_CORES)/efm32 -I$(ARDUINO_CORES)/../sys -I$(ARDUINO_CORES)/../sys/CMSIS/inc -I$(ARDUINO_CORES)/../sys/emlib/inc -I$(ARDUINO_CORES)/../sys/SiliconLabs/EFM32ZG/Include -I$(ARDUINO_CORES)/../variants/efm32zg110  

CXXINCS = -I$(ARDUINO_CORES) -I$(ARDUINO_CORES)/efm32 -I$(ARDUINO_CORES)/../sys -I$(ARDUINO_CORES)/../sys/CMSIS/inc -I$(ARDUINO_CORES)/../sys/emlib/inc -I$(ARDUINO_CORES)/../sys/SiliconLabs/EFM32ZG/Include -I$(ARDUINO_CORES)/../variants/efm32zg110  

CINCS		+= -I  $(LIB_DIR)/softspi/				\
			-I $(LIB_DIR)/softi2c/					\
			-I $(LIB_DIR)/pt1000/					\
			-I $(LIB_DIR)/pc10/						\
			-I $(LIB_DIR)/sx127x					\
			-I $(LIB_DIR)/U8g2/src					\
			-I $(LIB_DIR)/U8g2/src/clib				\
			-I $(LIB_DIR)/rtcdrv					\
			-I .									
CXXINCS		+= -I  $(LIB_DIR)/softspi/				\
			-I $(LIB_DIR)/softi2c/					\
			-I $(LIB_DIR)/pt1000/					\
			-I $(LIB_DIR)/pc10/						\
			-I $(LIB_DIR)/sx127x					\
			-I $(LIB_DIR)/U8g2/src					\
			-I $(LIB_DIR)/U8g2/src/clib				\
			-I $(LIB_DIR)/rtcdrv					\
			-I .

CFLAGS := -g -Os -w -fno-exceptions -std=gnu11 -ffunction-sections 		\
	-fdata-sections -nostdlib --param max-inline-insns-single=500 -Dprintf=iprintf

CFLAGS += -mcpu=$(MCU) -mfloat-abi=soft -mthumb -MMD

DEFS += -DARDUINO_efm32zg110f32 -DEFM32ZG -DEFM32ZG110F32 			\
	-DARM_MATH_CM0PLUS 													\
	-DF_CPU=$(F_CPU)													\
	-DUSE_HFRCO															\
	-DUSE_LEUART0=1														\
	-DUSE_LFBLFRCO=1													\
	-DEMDRV_RTCDRV_USE_LFRCO											\
	-DRAM_LENGTH=4096 -DFLASH_LENGTH=32768 -DEFM32GENERIC	 			\
	-DARDUINO_ARCH_EFM32 -DMENU_SERIAL=SerialLEUART0 -DARDUINO=$(VERSION) 



CXXFLAGS := -g -Os -w -fno-exceptions -std=gnu++14 -fpermissive			\
	-ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics	\
	--param max-inline-insns-single=500 -fno-rtti -Dprintf=iprintf -MMD 

CXXFLAGS += -mcpu=$(MCU) -mfloat-abi=soft -mthumb -MMD

ASFLAGS = 
LDFLAGS := -mcpu=$(MCU) -mthumb -Os -Wl,--cref					\
	-Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler	\
	-Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--start-group	\
	-u _sbrk -u link -u _close -u _fstat -u _isatty -u _lseek -u _read	\
	-u _write -u _exit -u kill -u _getpid 

LDFLAGS += "-T$(ARDUINO_BOARD)/ld/efm32zg110f32.ld"

#"-Wl,-Map,$(build_dir)/Blink.ino.map"
#"-L$(build_dir)"
#"$(build_dir)/sketch/Blink.ino.cpp.o" "$(build_dir)/core/variant.c.o" "$(build_dir)/core/variant.cpp.o" "$(build_dir)/core/core.a"


OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)
OBJ_MODULES = $(C_MODULES:.c=.o) $(CXX_MODULES:.cpp=.o)

LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

ALL_CFLAGS = $(CFLAGS) $(DEFS) $(CINCS)
ALL_CXXFLAGS = $(CXXFLAGS) $(DEFS) $(CXXINCS)

all: build_files rebuild build sizeafter

build: elf hex bin

build/main.cpp.o: 
	test -d build || mkdir build
	echo '#include "Arduino.h"' > build/main.cpp
	cat $(TARGET) >> build/main.cpp
	cat $(ARDUINO_CORES)/main.cpp >> build/main.cpp
	$(CXX) -c $(ALL_CXXFLAGS) build/main.cpp -o build/main.cpp.o
	$(CC) -c $(ALL_CFLAGS) $(ARDUINO_BOARD)/variant.c -o build/variant.c.o
	$(CXX) -c $(ALL_CXXFLAGS) $(ARDUINO_BOARD)/variant.cpp -o build/variant.cpp.o
	echo $(CXX_MODULES)

rebuild:
	$(RM) build/main.*

elf: build/main.elf
hex: build/main.hex
bin: build/main.bin
eep: build/main.eep
lss: build/main.lss
sym: build/main.sym

build/main.elf: build/main.cpp.o build/core.a
	$(LD) $(LDFLAGS) "-Wl,-Map,build/main.map" -o $@ build/main.cpp.o build/variant.c.o build/variant.cpp.o build/core.a -lstdc++ -lc -Wl,--end-group -lm -lgcc --specs=nano.specs

build/core.a: $(OBJ_MODULES)
	@for i in $(OBJ_MODULES); do echo $(AR) rcs build/core.a $$i; $(AR) rcs build/core.a $$i; done

.cpp.o:
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@

.c.o:
	$(CC) -c $(ALL_CFLAGS) $< -o $@

.c.s:
	$(CC) -S $(ALL_CFLAGS) $< -o $@

.S.o:
	$(CC) -c $(ALL_ASFLAGS) $< -o $@


%.d: %.c
	$(CC) -M $(ALL_CFLAGS) $< | sed "s;$(notdir $*).o:;$*.o $*.d:;" > $@

%.d: %.cpp
	$(CXX) -M $(ALL_CXXFLAGS) $< | sed "s;$(notdir $*).o:;$*.o $*.d:;" > $@

############################################################
HEXSIZE = $(SIZE) --target=ihex build/main.hex
ELFSIZE = $(SIZE)  build/main.elf

sizebefore:
	@if [ -f build/main.elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter:
	@if [ -f build/main.elf ]; then echo; echo $(MSG_SIZE_AFTER); $(HEXSIZE); echo; fi

coff: build/main.elf
	$(COFFCONVERT) -O coff-avr build/main.elf main.cof


.SUFFIXES: .elf .hex .bin .eep .lss .sym

.elf.hex:
	$(OBJCOPY) -O ihex -R .eeprom $< $@

.elf.eep:
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--no-change-warnings \
	--change-section-lma .eeprom=0 $< $@

.elf.bin:
	$(OBJCOPY) -O binary $< $@

.elf.lss:
	$(OBJDUMP) -h -S $< > $@

.elf.sym:
	$(NM) -n $< > $@
############################################################


upload: build/main.bin
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_CFG)

flash: build/main.bin
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_CFG)

reset:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_RESET)

clean:
	$(RM) build/main.hex build/main.eep build/main.cof build/main.elf \
	build/main.map build/main.sym build/main.o build/main.lss build/core.a \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d) \
	build/main.cpp
	$(RM) -r build

.PHONY:	all rebuild build elf hex eep lss sym program coff extcoff clean build_files sizebefore sizeafter
