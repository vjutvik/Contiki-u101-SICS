## What's this?

This is an *unofficial* branch of UPWIS' port of Contiki to STM32L/F. It also features support for various hardware devices found on the company's UPWIS -motes.


## How do I use it with Contiki-main?

Go to the directory of your app / project in your Contiki main branch, then type the following to compile:

make TARGET=u101-stm32l TARGETDIRS=../../../contiki-u101/platform

Please note that you probably have to change the number of ../ and the name of the directories. Furthermore, the first compile will fail due to problems internal to the build system. Simply re-issue the command and the build will succeed.

In order to program the mote you need to install OpenOCD tool. You can get a Debian package from http://dev.upwis.com/u101/openocd_0.6.0dev-1_i386.deb

The Debian package of above (ver. 0.6.0) is known to work.

Executing the following to program:

make OPENOCD_CFG=interface/neodb.cfg OPENOCD="sudo openocd" TARGETDIRS=../../../contiki-u101/platform TARGET=u101-stm32l <name of your binary>.upload

If you encounter chip reset / misc hardware problems during programming, just try again.


## Can I use it as a border router?

Sure you can!

The u101 has one USB type A connector on the CPU board itself. This is the connector used by the SLIP implementation and thus you need to connect this to the host PC if you want to use the u101 as a border router. This means that if you want a serial console on your border router you will need two USB cables (one from the type A connector and one from the debug board's mini-B.

The debug serial port will present itself to a Linux host as a /dev/ttyUSBn device while the "SLIP interfafce" will present itself as a /dev/ttyACMn device.

One way of running a border router is to build the slip-radio example (examples/ipv6/slip-radio) and run it on the u101 while running the native-border-router on the host PC.

./border-router.native -s ttyACM3 aaaa::1/64

It should also be possible to run the "ordinary" border-router example. This has been tested on the u101-stm32f.

## Is there anyone I can contact for help?

For information, please contact Vilhelm Jutvik at ville@sics.se
