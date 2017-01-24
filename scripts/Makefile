#
# Copyright (c) 2015 MaiKe Labs
#
#

ESPSDK_BASE		?= $(SDK_BASE)/sdk
TOOLCHAIN_DIR	?= $(SDK_BASE)/toolchain

# paths from the SDK used in this project
SDK_INCDIR	+= include include/driver
SDK_LIBDIR = lib
SDK_LDDIR = ld

# various paths from the ESPSDK used in this project
ESPSDK_INCDIR	+= include include/json
ESPSDK_LIBDIR	= lib
ESPSDK_LDDIR	= ld

# libraries used in this project, mainly provided by the ESPSDK
LIBS		+= gcc
LIBS		+= crypto espnow json lwip_536 lwip mesh net80211 phy pp pwm smartconfig mbedtls upgrade wpa wps main
LIBS		+= driver mjyun airkiss

# toolchain
XCC		?= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc
XAR		?= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-ar
XLD		?= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc

# compiler flags using during compilation of source files
CFLAGS		+= -g -O2 -Wpointer-arith -Wundef -Werror -Wl,-EL -fno-inline-functions -nostdlib -mlongcalls -mtext-section-literals  -D__ets__ -DICACHE_FLASH

# linker flags used to generate the main object file
LDFLAGS		+= -nostdlib -Wl,--no-check-sections -u call_user_start -Wl,-static

# linker script used for the above linkier step
LD_SCRIPT	= $(CONFIG_LD_SCRIPT)


# we create two different files for uploading into the flash
# these are the names and options to generate them
FW_FILE_1		= $(CONFIG_IRAM_ADDR)
FW_FILE_1_ARGS	= -bo $@ -bm $(CONFIG_FLASH_MODE) -bf $(CONFIG_FLASH_FREQ)	\
				  -bz $(subst B,,$(CONFIG_FLASH_SIZE)) -bs .text -bs .data -bs .rodata -bc -ec
FW_FILE_2		= $(CONFIG_IROM_ADDR)
FW_FILE_2_ARGS	= -es .irom0.text $@ -ec

# esptool path and port
ifeq ($(OS),Windows_NT)
	ESPTOOL		?= $(TOOLCHAIN_DIR)/bin/esptool.exe
	FW_TOOL		?= $(TOOLCHAIN_DIR)/bin/esptool.exe
	CONFIG_ESPPORT	?= COM3
else
	ESPTOOL		?= $(TOOLCHAIN_DIR)/bin/esptool
	FW_TOOL		?= $(TOOLCHAIN_DIR)/bin/esptool

	UNAME_S		:= $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		CONFIG_ESPPORT	?= /dev/cu.SLAB_USBtoUART
	endif
	ifeq ($(UNAME_S),Linux)
		CONFIG_ESPPORT	?= /dev/ttyUSB0
	endif
endif

# firmware generating tool
FW_TOOL		?= $(TOOLCHAIN_DIR)/bin/esptool
ESPTOOL_PY	?= $(TOOLCHAIN_DIR)/bin/esptool.py

ifeq ($(CONFIG_FLASH_SIZE), 512KB)
CONFIG_RFCAL_ADDR	?= 0x7B000
CONFIG_RFINIT_ADDR	?= 0x7C000
CONFIG_SYSCONF_ADDR	?= 0x7D000
CONFIG_YUNCONF_ADDR	?= 0x74000
CONFIG_LD_SCRIPT	?= $(CONFIG_CPU).512.ld
FLASH_SIZE = 512
endif
ifeq ($(CONFIG_FLASH_SIZE), 1MB)
CONFIG_RFCAL_ADDR	?= 0xFB000
CONFIG_RFINIT_ADDR	?= 0xFC000
CONFIG_SYSCONF_ADDR	?= 0xFD000
CONFIG_YUNCONF_ADDR	?= 0xF4000
CONFIG_LD_SCRIPT ?= $(CONFIG_CPU).1024.ld
FLASH_SIZE = 1024
endif
ifeq ($(CONFIG_FLASH_SIZE), 2MB)
CONFIG_RFCAL_ADDR ?= 0x1FB000
CONFIG_RFINIT_ADDR ?= 0x1FC000
CONFIG_SYSCONF_ADDR ?= 0x1FD000
CONFIG_YUNCONF_ADDR ?= 0x1F4000
CONFIG_LD_SCRIPT ?= $(CONFIG_CPU).2048.ld
FLASH_SIZE = 2048
endif
ifeq ($(CONFIG_FLASH_SIZE), 4MB)
CONFIG_RFCAL_ADDR ?= 0x3FB000
CONFIG_RFINIT_ADDR ?= 0x3FC000
CONFIG_SYSCONF_ADDR ?= 0x3FD000
CONFIG_YUNCONF_ADDR ?= 0x3F4000
CONFIG_LD_SCRIPT ?= $(CONFIG_CPU).4096.ld
FLASH_SIZE = 4096
endif

ifneq ($(ESPBAUD),)
CONFIG_ESPBAUD		= $(ESPBAUD)
endif
CONFIG_ESPBAUD		?= 230400

ifneq ($(ESPPORT),)
CONFIG_ESPPORT		= $(ESPPORT)
endif

##############################################
#### no user configurable options below here
###############################################
SRC_DIR		:= $(MODULES)
BUILD_BASE	:= $(addprefix $(BUILD_DIR)/,$(MODULES))

SDK_INCDIR		:= $(addprefix -I$(SDK_BASE)/,$(SDK_INCDIR))
SDK_LIBDIR		:= $(addprefix -L$(SDK_BASE)/,$(SDK_LIBDIR))

ESPSDK_INCDIR	:= $(addprefix -I$(ESPSDK_BASE)/,$(ESPSDK_INCDIR))
ESPSDK_LIBDIR	:= $(addprefix -L$(ESPSDK_BASE)/,$(ESPSDK_LIBDIR))
LD_SCRIPT		:= $(addprefix -T$(SDK_BASE)/$(SDK_LDDIR)/,$(LD_SCRIPT))

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
$1/%.o: %.c $(SRC_DIR)/compile.h
	$(vecho) "CC $$<"
	$(Q) $(XCC) $(INCDIR) $(MODULE_INCDIR) $(ESPSDK_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@
endef

.PHONY: all checkdirs flash clean $(SRC_DIR)/compile.h

all: checkdirs $(TARGET_OUT) $(FW_FILE_1) $(FW_FILE_2)

$(TARGET_OUT): $(APP_AR)
	$(vecho) "LD $@"
	$(Q) $(XLD) $(ESPSDK_LIBDIR) $(SDK_LIBDIR) $(LD_SCRIPT) $(LDFLAGS) -Wl,--start-group $(LIBS) $(APP_AR) -Wl,--end-group -o $@

$(APP_AR): $(OBJ)
	$(vecho) "AR $@"
	$(Q) $(XAR) cru $@ $^

checkdirs: $(BUILD_BASE) $(FW_BASE)

$(BUILD_BASE):
	$(Q) mkdir -p $@

firmware:
	$(Q) mkdir -p $@

clean:
	$(Q) rm -f $(APP_AR)
	$(Q) rm -f $(TARGET_OUT)
	$(Q) rm -rf $(BUILD_DIR)
	$(Q) rm -f $(FW_FILE_1)
	$(Q) rm -f $(FW_FILE_2)
	$(Q) rm -rf $(FW_BASE)
	$(Q) rm -f $(SRC_DIR)/compile.h

$(SRC_DIR)/compile.h:
	$(Q)$(SDK_BASE)/scripts/mkcompile_h $@ $(XCC) $(CONFIG_CPU) $(CONFIG_VERSION) $(CONFIG_HW_VERSION)

$(foreach bdir,$(BUILD_BASE),$(eval $(call compile-objects,$(bdir))))

$(FW_FILE_1): $(TARGET_OUT)
	$(vecho) "FW $@"
	$(Q) $(FW_TOOL) -eo $(TARGET_OUT) $(FW_FILE_1_ARGS)

$(FW_FILE_2): $(TARGET_OUT)
	$(vecho) "FW $@"
	$(Q) $(FW_TOOL) -eo $(TARGET_OUT) $(FW_FILE_2_ARGS)
 
flash: all
	-$(ESPTOOL) -cd $(CONFIG_BOARD) -cb $(CONFIG_ESPBAUD) -cp $(CONFIG_ESPPORT)				\
				-ca $(CONFIG_RFCAL_ADDR) -cf $(ESPSDK_BASE)/bin/blank.bin					\
				-ca $(CONFIG_RFINIT_ADDR) -cf $(ESPSDK_BASE)/bin/$(CONFIG_RF_INIT_DATA)		\
				-ca $(CONFIG_IRAM_ADDR) -cf $(FW_BASE)/$(CONFIG_IRAM_ADDR).bin	\
				-ca $(CONFIG_IROM_ADDR) -cf $(FW_BASE)/$(CONFIG_IROM_ADDR).bin

rfinit:
	-$(ESPTOOL) -cd $(CONFIG_BOARD) -cb $(CONFIG_ESPBAUD) -cp $(CONFIG_ESPPORT) 			\
				-ca $(CONFIG_RFCAL_ADDR) -cf $(ESPSDK_BASE)/bin/blank.bin				\
				-ca $(CONFIG_RFINIT_ADDR) -cf $(ESPSDK_BASE)/bin/$(CONFIG_RF_INIT_DATA)

sysclean:
	-$(ESPTOOL) -cd $(CONFIG_BOARD) -cb $(CONFIG_ESPBAUD) -cp $(CONFIG_ESPPORT)		\
				-ca $(CONFIG_SYSCONF_ADDR) -cf $(ESPSDK_BASE)/bin/blank.bin		\
				-ca $(shell printf "0x%x" $$(($(CONFIG_SYSCONF_ADDR)+0x1000))) -cf $(ESPSDK_BASE)/bin/blank.bin

yunclean:
	-$(ESPTOOL) -cd $(CONFIG_BOARD) -cb $(CONFIG_ESPBAUD) -cp $(CONFIG_ESPPORT)		\
				-ca $(CONFIG_YUNCONF_ADDR) -cf $(ESPSDK_BASE)/bin/blank.bin		\
				-ca $(shell printf "0x%x" $$(($(CONFIG_YUNCONF_ADDR)+0x1000))) -cf $(ESPSDK_BASE)/bin/blank.bin

zero:
	-$(ESPTOOL) -cd $(CONFIG_BOARD) -cb $(CONFIG_ESPBAUD) -cp $(CONFIG_ESPPORT)		\
				-ca $(CONFIG_SYSCONF_ADDR) -cf $(ESPSDK_BASE)/bin/blank.bin		\
				-ca $(shell printf "0x%x" $$(($(CONFIG_SYSCONF_ADDR)+0x1000))) -cf $(ESPSDK_BASE)/bin/blank.bin	\
				-ca $(CONFIG_YUNCONF_ADDR) -cf $(ESPSDK_BASE)/bin/blank.bin		\
				-ca $(shell printf "0x%x" $$(($(CONFIG_YUNCONF_ADDR)+0x1000))) -cf $(ESPSDK_BASE)/bin/blank.bin

# Reset the device id to $(DID), for testing
# make devid DID=WotD0000000208 PASS=xxxxxxxx

getdevid:
	-$(ESPTOOL_PY) --port $(CONFIG_ESPPORT) read_flash $(CONFIG_YUNCONF_ADDR) 0x1000 mjyun.bin
	-$(Q) echo "The original mjyun data is got into mjyun.bin"

setdevid:
	-$(if $(value DID),, $(error Device ID (DID) is not set!))
	-$(if $(value PASS),, $(error Device Password (PASS) is not set!))
	-$(if $(wildcard mjyun.bin),, $(error please run 'make getdevid' to backup devid first!))
	-$(Q) cp mjyun.bin jackxx.bin
	-echo "The original mjyun data is backuped in mjyun.bin"
	-printf $(DID) | dd conv=notrunc of=jackxx.bin bs=1 seek=35
	-printf $(PASS) | dd conv=notrunc of=jackxx.bin bs=1 seek=68
	-$(ESPTOOL) -cd $(CONFIG_BOARD) -cb $(CONFIG_ESPBAUD) -cp $(CONFIG_ESPPORT)	\
				-ca $(CONFIG_YUNCONF_ADDR) -cf jackxx.bin					\
				-ca $(shell printf "0x%x" $$(($(CONFIG_YUNCONF_ADDR)+0x1000))) -cf jackxx.bin
	-$(Q) rm -f jackxx.bin

# analysis the memory usage of our firmware
mem: all
	-$(Q) $(ESPTOOL_PY) elf2image -o /tmp/ $(TARGET_OUT)
	-$(Q) rm -f /tmp/*.bin

# erase the whole flash!
erase:
	-$(ESPTOOL_PY) --port $(CONFIG_ESPPORT) erase_flash

dump:
	-$(ESPTOOL_PY) --port $(CONFIG_ESPPORT) --baud $(CONFIG_ESPBAUD) read_flash 0x0 $(shell echo $$(($(FLASH_SIZE)*1024))) flash.bin
