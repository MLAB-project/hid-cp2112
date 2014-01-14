hid-cp2112
==========

Silicon Labs HID USB to SMBus master bridge driver

This driver adds support for Silicon Laboratories CP2112 HID USB-to-SMBus Bridge to
the linux kernel.

To build and install this driver, execute the following command sequence:

    $ make
    $ sudo make install

If you get an error message such as

    make: *** /usr/src/linux-headers-2.6.32-32-server: No such file or directory.
    Stop.
    make: *** [modules] Error 2

You will have to install the correct version of the linux header files. In
Ubuntu, and with above error message, this would be

    sudo apt-get install linux-headers-2.6.32-32-server
