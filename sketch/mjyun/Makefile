#
# Copyright (c) 2015 MaiKe Labs
#
#

# name for the target project
TARGET		= app

# which modules (subdirectories) of the project to include in compiling
MODULES		= user

BUILD_DIR	= build
FW_BASE		= build

# base directory of the SDK package, absolute
SDK_BASE		?= ../../
ESPSDK_BASE		?= $(SDK_BASE)/sdk
TOOLCHAIN_DIR	?= $(SDK_BASE)/toolchain

# paths from the SDK used in this project
SDK_INCDIR	= include include/driver
SDK_LIBDIR = lib

# various paths from the ESPSDK used in this project
ESPSDK_INCDIR	= include include/json
ESPSDK_LIBDIR	= lib
ESPSDK_LDDIR	= ld

# libraries used in this project, mainly provided by the ESPSDK
LIBS		:= gcc c
LIBS		+= crypto espnow json lwip_536 lwip mesh net80211 phy pp pwm smartconfig ssl upgrade wpa wps main
LIBS		+= driver airkiss mjyun 

# esptool path and port
ifeq ($(OS),Windows_NT)
	ESPTOOL		?= $(TOOLCHAIN_DIR)/bin/esptool.exe
	FW_TOOL		?= $(TOOLCHAIN_DIR)/bin/esptool.exe
	ESPPORT		?= COM3
else
	ESPTOOL		?= $(TOOLCHAIN_DIR)/bin/esptool
	FW_TOOL		?= $(TOOLCHAIN_DIR)/bin/esptool

	UNAME_S		:= $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		ESPPORT		?= /dev/cu.SLAB_USBtoUART
	endif
	ifeq ($(UNAME_S),Linux)
		ESPPORT		?= /dev/ttyUSB0
	endif
endif

# firmware generating tool
FW_TOOL		?= $(TOOLCHAIN_DIR)/bin/esptool

# toolchain
CC		:= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc
AR		:= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-ar
LD		:= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc

# compiler flags using during compilation of source files
CFLAGS		= -g -O2 -Wpointer-arith -Wundef -Werror -Wl,-EL -fno-inline-functions -nostdlib -mlongcalls -mtext-section-literals  -D__ets__ -DICACHE_FLASH

# linker flags used to generate the main object file
LDFLAGS		= -nostdlib -Wl,--no-check-sections -u call_user_start -Wl,-static

# linker script used for the above linkier step
LD_SCRIPT	= eagle.app.v6.4096.ld

# we create two different files for uploading into the flash
# these are the names and options to generate them
FW_FILE_1		= 0x00000
FW_FILE_1_ARGS	= -bo $@ -bm dio -bf 40 -bz 4M -bs .text -bs .data -bs .rodata -bc -ec
FW_FILE_2		= 0x40000
FW_FILE_2_ARGS	= -es .irom0.text $@ -ec

###############################################
#### no user configurable options below here
###############################################
SRC_DIR		:= $(MODULES)
BUILD_BASE	:= $(addprefix $(BUILD_DIR)/,$(MODULES))

SDK_INCDIR		:= $(addprefix -I$(SDK_BASE)/,$(SDK_INCDIR))
SDK_LIBDIR		:= $(addprefix -L$(SDK_BASE)/,$(SDK_LIBDIR))

ESPSDK_INCDIR	:= $(addprefix -I$(ESPSDK_BASE)/,$(ESPSDK_INCDIR))
ESPSDK_LIBDIR	:= $(addprefix -L$(ESPSDK_BASE)/,$(ESPSDK_LIBDIR))
LD_SCRIPT		:= $(addprefix -T$(ESPSDK_BASE)/$(ESPSDK_LDDIR)/,$(LD_SCRIPT))

INCDIR			:= $(addprefix -I,$(SRC_DIR))
MODULE_INCDIR	:= -I include

SRC			:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))
OBJ			:= $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRC))
LIBS		:= $(addprefix -l,$(LIBS))
APP_AR		:= $(addprefix $(BUILD_DIR)/,$(TARGET)_app.a)
TARGET_OUT	:= $(addprefix $(BUILD_DIR)/,$(TARGET).out)

FW_FILE_1	:= $(addprefix $(FW_BASE)/,$(FW_FILE_1).bin)
FW_FILE_2	:= $(addprefix $(FW_BASE)/,$(FW_FILE_2).bin)

V ?= $(VERBOSE)
ifeq ("$(V)","1")
Q :=
vecho := @true
else
Q := @
vecho := @echo
endif

vpath %.c $(SRC_DIR)

define compile-objects
$1/%.o: %.c
	$(vecho) "CC $$<"
	$(Q) $(CC) $(INCDIR) $(MODULE_INCDIR) $(ESPSDK_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@
endef

.PHONY: all checkdirs flash clean

all: checkdirs $(TARGET_OUT) $(FW_FILE_1) $(FW_FILE_2)

$(FW_FILE_1): $(TARGET_OUT)
	$(vecho) "FW $@"
	$(Q) $(FW_TOOL) -eo $(TARGET_OUT) $(FW_FILE_1_ARGS)

$(FW_FILE_2): $(TARGET_OUT)
	$(vecho) "FW $@"
	$(Q) $(FW_TOOL) -eo $(TARGET_OUT) $(FW_FILE_2_ARGS)

$(TARGET_OUT): $(APP_AR)
	$(vecho) "LD $@"
	$(Q) $(LD) $(ESPSDK_LIBDIR) $(SDK_LIBDIR) $(LD_SCRIPT) $(LDFLAGS) -Wl,--start-group $(LIBS) $(APP_AR) -Wl,--end-group -o $@

$(APP_AR): $(OBJ)
	$(vecho) "AR $@"
	$(Q) $(AR) cru $@ $^

checkdirs: $(BUILD_BASE) $(FW_BASE)

$(BUILD_BASE):
	$(Q) mkdir -p $@

firmware:
	$(Q) mkdir -p $@

flash: all $(FW_BASE)/0x00000.bin $(FW_BASE)/0x40000.bin
	-$(ESPTOOL) -cd nodemcu -cb 115200 -cp $(ESPPORT) -ca 0x00000 -cf $(FW_BASE)/0x00000.bin -ca 0x40000 -cf $(FW_BASE)/0x40000.bin

zero:
	-$(ESPTOOL) -cd nodemcu -cb 115200 -cp $(ESPPORT) -ca 0x3FE000 -cf $(ESPSDK_BASE)/bin/blank.bin

clean:
	$(Q) rm -f $(APP_AR)
	$(Q) rm -f $(TARGET_OUT)
	$(Q) rm -rf $(BUILD_DIR)
	$(Q) rm -f $(FW_FILE_1)
	$(Q) rm -f $(FW_FILE_2)
	$(Q) rm -rf $(FW_BASE)

$(foreach bdir,$(BUILD_BASE),$(eval $(call compile-objects,$(bdir))))
