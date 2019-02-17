obj-m += joycon.o
MY_CFLAGS += -g -DDEBUG 

KVERSION := $(KERNELRELEASE)
ifeq ($(origin KERNELRELEASE), undefined)
KVERSION := $(shell uname -r)
endif
KDIR := /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

test: all
	sync
	-rmmod joycon
	insmod ./joycon.ko

debug: ccflags-y += ${MY_CFLAGS} CC += ${MY_CFLAGS}
debug: all
