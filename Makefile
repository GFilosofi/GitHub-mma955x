# Author: Emiliano Betti (Epigenesys s.r.l.)

ROOTFS?=/var/lib/nfs-k5-rootfs

MODNAME=mma955x

# If y, forces DEBUG output for this module.
# In n, DEBUG output can still be enabled if CONFIG_COSMED_DEBUG is selected in
# kernel configuration
DEBUG?=n

# Comment this line not to install the header
#INSTALL_HEADERS=$(MODNAME).h

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
	DEBFLAGS = -Wall -Wstrict-prototypes -O -g -DCONFIG_COSMED_DEBUG
# "-O" is needed to expand inlines
else
	DEBFLAGS = -Wall -Wstrict-prototypes -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS) -DDR_NAME=\"$(MODNAME)\"

ifneq ($(KERNELRELEASE),)
  ALL_C_SOURCE:=$(shell cd $(M) && ls *.c)
  ALL_OBJS:=$(ALL_C_SOURCE:.c=.o)
  # Comment the next line if you do not depend on any other module
  # KBUILD_EXTRA_SYMBOLS=$(M)/../ad768x/Module.symvers $(M)/../mpl3115a2/Module.symvers
  obj-m := $(MODNAME).o
  $(MODNAME)-objs := $(ALL_OBJS)
else
  ALL_SOURCE=$(shell echo *.c *.h)
  KERNELDIR ?= ../../kernels/linux
  OBJDIR ?= ../../kernels/linux
  #KERNELDIR ?= /lib/modules/$(shell uname -r)/source
  #OBJDIR ?= /lib/modules/$(shell uname -r)/build
  PWD := $(shell pwd)

default: headers
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-arago-linux-gnueabi- -C $(KERNELDIR) O=$(OBJDIR) M=$(PWD) modules
	if [ -d test ]; then make -C test CROSS_COMPILE=arm-arago-linux-gnueabi- ; fi

.PHONY: clean

tags: $(ALL_SOURCE)
	ctags --language-force=C $(ALL_SOURCE)

TAGS: $(ALL_SOURCE)
	ctags -e --language-force=C $(ALL_SOURCE)

ifneq ($(INSTALL_HEADERS),)
headers: $(INSTALL_HEADERS)
	@sudo mkdir -p $(ROOTFS)/usr/include/cosmed
	@sudo cp -v $(KERNELDIR)/include/cosmed/* $(ROOTFS)/usr/include/cosmed/
	@sudo cp -v $^ $(ROOTFS)/usr/include/cosmed/
else
headers:
	@echo "No headers to install"
endif

install: default
	sudo INSTALL_MOD_PATH=$(ROOTFS) $(MAKE) ARCH=arm CROSS_COMPILE=arm-arago-linux-gnueabi- -C $(KERNELDIR) O=$(OBJDIR) M=$(PWD) modules_install
	if [ -d test ]; then sudo make -C test install MODNAME=$(MODNAME) ROOTFS=$(ROOTFS) ; fi

clean:
	if [ -d test ]; then make -C test clean ; fi
	rm -rf .*.ko.cmd .*.o.cmd *.o *~ *.mod.c Module.symvers modules.order .tmp_versions
	rm -rf tags TAGS

distclean: clean
	if [ -d test ]; then make -C test distclean ; fi
	rm -f $(MODNAME).ko

endif

