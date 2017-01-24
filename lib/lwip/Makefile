#############################################################
# Required variables for each makefile
# Discard this section from all parent makefiles
# Expected variables (with automatic defaults):
#   CSRCS (all "C" files in the dir)
#   SUBDIRS (all subdirs with a Makefile)
#   GEN_LIBS - list of libs to be generated ()
#   GEN_IMAGES - list of images to be generated ()
#   COMPONENTS_xxx - a list of libs/objs in the form
#     subdir/lib to be extracted and rolled up into
#     a generated lib/image xxx.a ()
#
ifndef PDIR
UP_EXTRACT_DIR = ..
GEN_LIBS = liblwip_osc.a
COMPONENTS_liblwip_osc =	api/liblwipapi.a		\
							app/liblwipapp.a		\
							core/liblwipcore.a		\
							netif/liblwipnetif.a
endif

# noduino sdk base dir
SDK_BASE := $(PDIR)../..
ESPSDK_BASE := $(SDK_BASE)/sdk

#############################################################
# Configuration i.e. compile options etc.
# Target specific stuff (defines etc.) goes in here!
# Generally values applying to a tree are captured in the
#   makefile at its root level - these are then overridden
#   for a subtree within the makefile rooted therein
#
DEFINES += -DICACHE_FLASH -U__STRICT_ANSI__ -DLWIP_OPEN_SRC

#############################################################
# Recursion Magic - Don't touch this!!
#
# Each subtree potentially has an include directory
#   corresponding to the common APIs applicable to modules
#   rooted at that subtree. Accordingly, the INCLUDE PATH
#   of a module can only contain the include directories up
#   its parent path, and not its siblings
#
# Required for each makefile to inherit from the parent
#
INCLUDES := $(INCLUDES) -I $(PDIR)include
INCLUDES += -I$(ESPSDK_BASE)/include -I$(SDK_BASE)/include/lwip -I./ 
PDIR := ../$(PDIR)
sinclude $(ESPSDK_BASE)/Makefile

LIB_INCDIR	:= ../../include/lwip

install: all
	mv .output/lib/*.a ../ 
	rm -rf $(LIB_INCDIR)
	cp -a include $(LIB_INCDIR)
	rm $(LIB_INCDIR)/user_config.h
