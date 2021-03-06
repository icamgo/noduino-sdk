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

TARGET		= main.cpp

SDK_BASE	?= ../..
ARDUINO_CORES	:= $(SDK_BASE)/arch/efm32/cores
ARDUINO_BOARD	:= $(SDK_BASE)/arch/efm32/variants/efm32zg110
LIB_DIR		:= $(SDK_BASE)/arch/efm32/libs

DEFS := -DCONFIG_V0 -DUSE_SOFTSPI
#DEFS = -DUSE_SOFTSPI

MCU = cortex-m0plus
F_CPU = 1000000
#F_CPU = 14000000

VERSION=187

TOOLCHAIN_DIR = $(SDK_BASE)/toolchain/gcc-arm-none-eabi/bin
OPENOCD = $(SDK_BASE)/toolchain/openocd-nt/bin/openocd

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

V ?= $(VERBOSE)
ifeq ("$(V)","1")
Q :=
vecho := @true
else
Q := @
vecho := @echo
endif

OPENOCD_FW = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; program $(fw) verify reset; shutdown"

OPENOCD_FLASH = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; program build/main.bin verify reset; shutdown"
OPENOCD_USER = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; wait_halt; flash erase_sector 1 0 0; flash write_bank 1 .u.bin; flash verify_bank 1 .u.bin; reset run; shutdown"

OPENOCD_ERASE_MAIN = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; flash banks; reset init; reset halt; wait_halt; flash erase_sector 0 0 31; reset run; shutdown"
#OPENOCD_ERASE_USER = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; flash banks; reset init; reset halt; wait_halt; flash erase_sector 1 0 0; reset run; shutdown"
OPENOCD_ERASE_USER = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; wait_halt; flash erase_sector 1 0 0; reset run; shutdown"

OPENOCD_DUMP_MAIN = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; flash probe 0; flash read_bank 0 flash.bin; reset; shutdown"
OPENOCD_DUMP_USER = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; dump_image user.bin 0x0FE00000 0x400; reset; shutdown"

OPENOCD_GET_DID = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; dump_image .did.bin 0x0FE00008 0x8; reset; shutdown"
OPENOCD_GET_UUID = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; dump_image .uuid.bin 0x0FE081F0 0x8; reset; shutdown"

OPENOCD_GO = "set CPUTAPID 0x0bc11477; source [find target/efm32.cfg]; init; targets; reset halt; program $(fw) verify reset; init; targets; reset halt; wait_halt; flash erase_sector 1 0 0; flash write_bank 1 .u.bin; flash verify_bank 1 .u.bin; reset run; shutdown"

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
$(LIB_DIR)/softi2c/softi2c.cpp							\
$(LIB_DIR)/softspi/softspi.cpp							\
$(LIB_DIR)/sht2x/sht2x.cpp								\
$(LIB_DIR)/sts3x/sts3x.cpp								\
$(LIB_DIR)/sx127x/sx1272.cpp


CXX_APP = main.cpp
MODULES = $(C_MODULES) $(CXX_MODULES)
SRC = $(C_MODULES)
CXXSRC = $(CXX_MODULES) $(CXX_APP)

CINCS = -I$(ARDUINO_CORES) -I$(ARDUINO_CORES)/efm32 -I$(ARDUINO_CORES)/../sys -I$(ARDUINO_CORES)/../sys/CMSIS/inc -I$(ARDUINO_CORES)/../sys/emlib/inc -I$(ARDUINO_CORES)/../sys/SiliconLabs/EFM32ZG/Include -I$(ARDUINO_CORES)/../variants/efm32zg110 

CXXINCS = -I$(ARDUINO_CORES) -I$(ARDUINO_CORES)/efm32 -I$(ARDUINO_CORES)/../sys -I$(ARDUINO_CORES)/../sys/CMSIS/inc -I$(ARDUINO_CORES)/../sys/emlib/inc -I$(ARDUINO_CORES)/../sys/SiliconLabs/EFM32ZG/Include -I$(ARDUINO_CORES)/../variants/efm32zg110 

CINCS		+= -I  $(LIB_DIR)/softspi/				\
			-I $(LIB_DIR)/softi2c/					\
			-I $(LIB_DIR)/sht2x/					\
			-I $(LIB_DIR)/sts3x/					\
			-I $(LIB_DIR)/U8g2/src					\
			-I $(LIB_DIR)/U8g2/src/clib				\
			-I $(LIB_DIR)/rtcdrv					\
			-I $(LIB_DIR)/sx127x					\
			-I .
CXXINCS		+= -I  $(LIB_DIR)/softspi/				\
			-I $(LIB_DIR)/softi2c/					\
			-I $(LIB_DIR)/sht2x/					\
			-I $(LIB_DIR)/sts3x/					\
			-I $(LIB_DIR)/U8g2/src					\
			-I $(LIB_DIR)/U8g2/src/clib				\
			-I $(LIB_DIR)/rtcdrv					\
			-I $(LIB_DIR)/sx127x					\
			-I .

CFLAGS := -g -Os -w -fno-exceptions -std=gnu11 -ffunction-sections 		\
	-fdata-sections -nostdlib --param max-inline-insns-single=500 -Dprintf=iprintf

CFLAGS += -mcpu=$(MCU) -mfloat-abi=soft -mfp16-format=ieee -mthumb -MMD

DEFS += -DARDUINO_efm32zg110f32 -DEFM32ZG -DEFM32ZG110F32 			\
	-DARM_MATH_CM0PLUS 													\
	-DF_CPU=$(F_CPU)													\
	-DUSE_HFRCO															\
	-DUSE_LEUART0=1														\
	-DUSE_LFBLFRCO=1													\
	-DMENU_SERIAL=SerialLEUART0 										\
	-DEMDRV_RTCDRV_USE_LFRCO											\
	-DRAM_LENGTH=4096 -DFLASH_LENGTH=32768 -DEFM32GENERIC	 			\
	-DARDUINO_ARCH_EFM32 -DARDUINO=$(VERSION)

CXXFLAGS := -g -Os -w -fno-exceptions -std=gnu++14 -fpermissive			\
	-ffunction-sections -fdata-sections -nostdlib -fno-threadsafe-statics	\
	--param max-inline-insns-single=500 -fno-rtti -Dprintf=iprintf -MMD

CXXFLAGS += -mcpu=$(MCU) -mfloat-abi=soft -mfp16-format=ieee -mthumb -MMD

ASFLAGS =
LDFLAGS := -mcpu=$(MCU) -mthumb -Os -Wl,--cref					\
	-Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler	\
	-Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--start-group	\
	-u _sbrk -u link -u _close -u _fstat -u _isatty -u _lseek -u _read	\
	-u _write -u _exit -u kill -u _getpid

LDFLAGS += "-T$(ARDUINO_BOARD)/ld/efm32zg110f32.ld"


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
dev_type	:= 02
year		:= $(shell date +%y)
week		:= $(shell date +%V)

id_base := 1$(year)$(dev_type)$(week)0000

ifeq ($(OS),Windows_NT)
rand		:= $(shell /bin/bash -c "echo $$RANDOM")
else
rand		:= $(shell /bin/bash -c 'echo $$RANDOM')
endif

ifndef id
id = $$(($(id_base)+$(rand)%10000))

id_ok = $$(cat .idused | grep $(id))
endif

hex_id = $$(printf "%016X" $(id) | fold -w2 | tac | tr -d "\n")

DID=$$(xxd -g8 -p .did.bin | fold -w2 | tac | tr -d "\n")
UUID=$$(xxd -g8 -p .uuid.bin | fold -w2 | tac | tr -d "\n")
UBINID=$$(xxd -g8 -s8 -l8 -p .u.bin | fold -w2 | tac | tr -d "\n")

ifndef fw
fw := build/main.bin
endif

# make produce (write firmware main.bin into main flash, set a random devid and log it)
# make produce fw=xxx.bin (write firmware xxx.bin into main flash, set a random devid and log it)
produce: $(fw)
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_FW)
	-$(Q) echo "Generating the random DevID: $(id)"
	@if [ -z $(id_ok) ]; then echo DevID $(id) is OK; echo $(id) >> .idused; \
		echo "Generating the u.bin ..."; cp -f $(ARDUINO_BOARD)/user.bin .u.bin;				  \
		echo $(hex_id) | xxd -r -s 8 -p - .u.bin; else echo $(id) is used, please re-make; exit -1; fi
	@printf "Current DevID in u.bin is: %d\r\n" 0x$(UBINID);
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_USER)
	@rm -f .u.bin

# same as produce
# make go (write firmware main.bin into main flash, set a random devid and log it)
# make go fw=xxx.bin (write firmware xxx.bin into main flash, set a random devid and log it)

go: $(fw) .u.bin
	@printf "Current DevID in u.bin is: %d\r\n" 0x$(UBINID);
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_GO) && rm -f .u.bin

.u.bin:
	-$(Q) echo $(id)
	@if [ -z $(id_ok) ]; then echo DevID $(id) is OK; echo $(id) >> .idused; \
		echo "Generating the u.bin ..."; cp -f $(ARDUINO_BOARD)/user.bin .u.bin;				  \
		echo $(hex_id) | xxd -r -s 8 -p - .u.bin; else echo $(id) is used, please re-make; exit -1; fi

upload: build/main.bin
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_FLASH)

flash: build/main.bin
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_FLASH)

user: .u.bin
	@printf "Current DevID in u.bin is: %d\r\n" 0x$(UBINID);
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_USER)
	@rm -f .u.bin

dumpflash:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_DUMP_MAIN)
	-$(Q) echo "The whole 32KB flash is dumped into flash.bin"

dumpuser:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_DUMP_USER)
	-$(Q) echo "Dump the 1KB user data into user.bin"

erase:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_ERASE_MAIN)
	-$(Q) echo "Erase the 32KB main data"

eraseflash:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_ERASE_MAIN)
	-$(Q) echo "Erase the 32KB main data"

eraseuser:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_ERASE_USER)
	-$(Q) echo "Erase the 1KB user data @0x0FE0_0000"

getid:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_GET_DID)
	@echo ""
	@printf "The DevID is: %d\r\n" 0x$(DID);
	@rm -f .did.bin

# same as getid
getdid:
	@$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_GET_DID)
	@echo ""
	@printf "The DevID is: %d\r\n" 0x$(DID);
	@rm -f .did.bin

getuuid:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_GET_UUID)
	@echo ""
	@printf "The UUID is: %d\r\n" 0x$(UUID);
	@echo $(UUID) | tr [:lower:] [:upper:]
	@rm -f .uuid.bin

# make setid (set random id, do not log it)
# make setid id=xxx (set id=xxx, do not log it)
setid:
	@printf "Current used DevID is: %d (0x%016X)\r\n" $(id) $(id);
	@echo "Generating the u.bin ..."; cp -f $(ARDUINO_BOARD)/user.bin .u.bin;  \
		echo $(hex_id) | xxd -r -s 8 -p - .u.bin;
	@printf "Current DevID in u.bin is: %d\r\n" 0x$(UBINID);
	@$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_USER)
	@rm -f .u.bin

# same as setid
setdid:
	@printf "Current used DevID is: %d (0x%016X)\r\n" $(id) $(id);
	@echo "Generating the u.bin ..."; cp -f $(ARDUINO_BOARD)/user.bin .u.bin;  \
		echo $(hex_id) | xxd -r -s 8 -p - .u.bin;
	@printf "Current DevID in u.bin is: %d\r\n" 0x$(UBINID);
	@$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_USER)
	@rm -f .u.bin

reset:
	$(OPENOCD) -f interface/stlink.cfg -c $(OPENOCD_RESET)

clean:
	$(RM) build/main.hex build/main.eep build/main.cof build/main.elf \
	build/main.map build/main.sym build/main.o build/main.lss build/core.a \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d) \
	build/main.cpp
	$(RM) -r build

.PHONY:	all rebuild build elf hex eep lss sym program coff extcoff clean build_files sizebefore sizeafter
