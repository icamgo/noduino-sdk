#
# Copyright (c) 2019 MaiKe Labs
#

SDK_BASE		?= ../..
OPENCPU_BASE	?= $(SDK_BASE)/arch/opencpu
TOOLCHAIN_DIR	?= $(SDK_BASE)/toolchain/arm-none-eabi

HWVER := LV 

#LD_SCRIPT	= $(CONFIG_LD_SCRIPT)
LD_SCRIPT	:= mt2625_flash_bring_up.ld

LD_SCRIPT	:= $(addprefix -T$(OPENCPU_BASE)/lib/,$(LD_SCRIPT))

XCC := $(TOOLCHAIN_DIR)/bin/arm-none-eabi-gcc
XAR := $(TOOLCHAIN_DIR)/bin/arm-none-eabi-ar
XLD := $(TOOLCHAIN_DIR)/bin/arm-none-eabi-ld
XOBJCOPY := $(TOOLCHAIN_DIR)/bin/arm-none-eabi-objcopy

CFLAGS := -mlittle-endian -mthumb -mcpu=cortex-m4			\
			-mfpu=fpv4-sp-d16 -mfloat-abi=hard 				\
			-fsingle-precision-constant -Wdouble-promotion 	\
			-ffunction-sections -fdata-sections				\
			-fno-builtin -fno-strict-aliasing -fno-common	\
			-gdwarf-2 -std=gnu99 -w -Os 

LDFLAGS := -mlittle-endian -mthumb -mcpu=cortex-m4			\
			-mfpu=fpv4-sp-d16 -mfloat-abi=hard 				\
			-fsingle-precision-constant -Wdouble-promotion	\
			--specs=nano.specs -lnosys -nostartfiles		\
			-u _printf_float -u _scanf_float				\
			-Wl,-wrap=malloc -Wl,-wrap=calloc				\
			-Wl,-wrap=realloc -Wl,-wrap=free -Wl,-wrap=memcpy		\
			-Wl,-wrap=memset -Wl,-wrap=memmove -Wl,-wrap=memcmp		\
			-Wl,-wrap=strcpy -Wl,-wrap=strcmp -Wl,-wrap=strlen		\
			-Wl,-wrap=fprintf -Wl,-wrap=_printf_i					\
			-Wl,--gc-sections						\
			-u md_commit_id_str										\
			-Wl,--whole-archive					\
			-Wl,--start-group

LIBS	:= LVobj.a								\
			LVcustom_lib_md.a					\
			libhal_protected_CM4_GCC.a			\
			librsdl.a							\
			lib_md.a							\
			libtool_authentication_CM4_GCC.a

LIBS	:= $(addprefix $(OPENCPU_BASE)/lib/,$(LIBS))

LDFLAGS += $(LIBS)
LDFLAGS += -Wl,--end-group -Wl,--no-whole-archive
LDFLAGS += -lm
LDFLAGS += -Wl,$(LD_SCRIPT)

INCDIR	:= inc/os inc/apb inc/lwip inc
INCDIR	:= $(addprefix -I$(OPENCPU_BASE)/,$(INCDIR))
INCDIR	+= -I .

SRC_DIRS := .

OBJS = $(foreach dir,$(SRC_DIRS),$(patsubst %.c,%.o,$(wildcard $(dir)/*.c)))

all: $(SRC_DIRS)/compile.h $(OBJS)
	@echo "linking libs"
	@mkdir -p firmware
	$(XCC) $(LDFLAGS) $(OBJS) -o firmware/nbiot_m2m_demo.elf
	$(XOBJCOPY) -O binary firmware/nbiot_m2m_demo.elf firmware/nbiot_m2m_demo.bin
	cp $(OPENCPU_BASE)/utils/flash_download.cfg firmware/
	cp $(OPENCPU_BASE)/utils/mt2625_bootloader.bin firmware/

%.o: %.c
	@echo 'compile $<...'''
	@$(XCC) $(CFLAGS) $(INCDIR) -c $< -o $@

sinclude .config

$(SRC_DIRS)/compile.h:
	$(SDK_BASE)/scripts/mkcompile_h $@ $(XCC) $(CONFIG_CPU) $(CONFIG_VERSION) $(CONFIG_HW_VERSION)
	
clean:
	@rm -f $(subst /,\,$(OBJS))
	@rm -rf firmware
	@rm -f compile.h
	@echo clean ends
