##############################################
##
## Compile module
##
##############################################



SRCDIR:=/work/sdn/netmap/LINUX
BUILDDIR:= $(shell pwd)
ROOTDIR := /work/sdn/netmap

# The following commands are needed to build the modules as out-of-tree,
# in fact the kernel sources path must be specified.

# Additional compile flags (e.g. header location)
EXTRA_CFLAGS := -I$(ROOTDIR) -I$(BUILDDIR) -I$(SRCDIR) -I$(SRCDIR)/../sys -I$(SRCDIR)/../sys/dev -DCONFIG_NETMAP -DDEV_NETMAP
EXTRA_CFLAGS += -Wno-unused-but-set-variable


# We use KSRC for the kernel configuration and sources.
# If the sources are elsewhere, then use SRC to point to them.
KSRC = /work/orangePi/OrangePi_H5SDK/kernel
SRC  = /work/orangePi/OrangePi_H5SDK/kernel/
KOPTS =  ARCH=arm64 CROSS_COMPILE=/work/orangePi/OrangePi_H5SDK/toolchain/gcc-linaro-aarch/bin/aarch64-linux-gnu-
MODPATH = 
LIN_VER = 30a41


ifneq ($(KERNELRELEASE),)
    obj-m := ax88179_178a.o
    obj-m += usbnet.o
#module-objs := probe.o
else
	PWD := $(shell pwd)
    
default:
	cp $(ROOTDIR)/Module.symvers $(BUILDDIR)/
	make -C $(KSRC) M=$(PWD) EXTRA_CFLAGS='$(EXTRA_CFLAGS)' $(KOPTS) modules

endif

.PHONY : clean

clean:
	make -C $(KSRC) M=$(PWD) EXTRA_CFLAGS='$(EXTRA_CFLAGS)' $(KOPTS) modules clean
     
     



	
	
	