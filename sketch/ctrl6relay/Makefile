#
# Copyright (c) 2015 MaiKe Labs
#
#

# name for the target project
TARGET		= app

# which modules (subdirectories) of the project to include in compiling
MODULES		= .

BUILD_DIR	= build
FW_BASE		= build

# base directory of the SDK package, absolute
SDK_BASE		?= ../..

# read the configuration file
include $(SDK_BASE)/.config
sinclude .config

# override following variables to use your toolchain
#XCC		:= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc
#XAR		:= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-ar
#XLD		:= $(TOOLCHAIN_DIR)/xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc

# include path used in this project
SDK_INCDIR	+= include/noduino include/lwip

# libraries used in this project
LIBS		+= cloud cirom mirom noduino ssl espudpd echo

# compiler flags using during compilation of source files
CFLAGS		+= -DLWIP_OPEN_SRC

# linker flags used to generate the main object file
LDFLAGS		+=

# include common section of the build system
ifeq ($(CONFIG_OTA), y)
include $(SDK_BASE)/scripts/Makefile.ota
else
include $(SDK_BASE)/scripts/Makefile
endif
