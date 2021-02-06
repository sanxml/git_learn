KERNELDIR := /run/media/sanxml/ubuntu/GitHub/Lichee-Pi/buildroot-licheepi/output/build/linux-5.4.77
CURRENT_PATH := $(shell pwd)
obj-m := chrdevbase.o


build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean