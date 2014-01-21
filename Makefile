# For building for the current running version of Linux
TARGET		:= $(shell uname -r)
HOME=$(shell pwd)
# Or for a specific version
#TARGET		:= 2.6.33.5
KERNEL_MODULES	:= /lib/modules/$(TARGET)
# KERNEL_BUILD	:= $(KERNEL_MODULES)/build
KERNEL_BUILD	:= /usr/src/linux-headers-$(TARGET)

#SYSTEM_MAP	:= $(KERNEL_BUILD)/System.map
SYSTEM_MAP	:= /boot/System.map-$(TARGET)

DRIVER := hid-cp2112
DRIVER2 := hid


# Directory below /lib/modules/$(TARGET)/kernel into which to install
# the module:
MOD_SUBDIR = drivers/misc

obj-m	:= hid-cp2112.o hid-core.o i2c-mux-pca954x.o

MAKEFLAGS += --no-print-directory
#LDFLAGS += -lhid

.PHONY: all install modules modules_install clean

all: modules

# Targets for running make directly in the external module directory:

modules clean:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(CURDIR) EXTRA_CFLAGS=-I$(CURDIR)/include $@

install: modules_install

modules_install:
	cp $(DRIVER).ko $(KERNEL_MODULES)/kernel/$(MOD_SUBDIR)
	depmod -a -F $(SYSTEM_MAP) $(TARGET)
