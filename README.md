hid-cp2112
==========

Silicon Labs HID USB to SMBus master bridge driver. Driver should work only with kernel version 3.15+. Drivers does not create a new kernel I2C interface in older kernels than 3.15.

This driver will add support for Silicon Laboratories CP2112 HID USB-to-SMBus Bridge to
the linux kernel. This IC is used in MLAB USBI2C01A module. 

To build and install this driver, execute the following command sequence:

    $ make
    $ insmod ./hid-cp2112.ko

After test of sucessfull working you can run 

    $ sudo make install

To install module to kernel modules directory. 

If you get an error message such as

    make: *** /usr/src/linux-headers-2.6.32-32-server: No such file or directory.
    Stop.
    make: *** [modules] Error 2

You will have to install the correct version of the linux header files. In
Ubuntu, and with above error message, you can use

    sudo apt-get install linux-headers-$(uname -r)


If you need patch whole kernel source tree, you should use this command. 

    patch -p1 < hid-cp2112.patch
