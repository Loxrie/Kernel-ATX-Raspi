obj-m := kernel_atx_raspi.o
KVERSION := `uname -r`

ifneq (,$(findstring -v7, $(KVERSION)))
CFLAGS_kernel_atx_raspi.o := -DRPI2
endif

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
