obj-m := mydm9000.o
KDIR := /home/akaedu/CortexA8-s5pv210-20120901/tiny210/build-env/src/linux-2.6.35.7

all:
	make -C $(KDIR)	SUBDIRS=$(PWD) 	modules
	ls -l *.ko

clean:
	-rm *.ko *.o *.order *.mod.c *.symvers
