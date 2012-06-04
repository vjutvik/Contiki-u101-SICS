## What's this?

This is an *unofficial* branch of UPWIS' port of Contiki to STM32L/F. It also features support for various hardware devices found on the company's UPWIS -motes.


## How do I use it with Contiki-main?

Go to the directory of your app / project in your Contiki main branch, then type the following to compile:

make TARGET=u101-stm32l TARGETDIRS=../../../contiki-u101/platform

Please note that you probably have to change the number of ../ and the name of the directories. Furthermore, the first compile will fail due to problems internal to the build system. Simply re-issue the command and the build will succeed.

In order to program the mote you need to install OpenOCD tool. You can get a Debian package from http://dev.upwis.com/u101/openocd_0.6.0dev-1_i386.deb

The Debian package of above (ver. 0.6.0) is known to work.

Executing the following to program:

make OPENOCD_CFG=interface/neodb.cfg OPENOCD="sudo openocd" TARGETDIRS=../../../contiki-u101/platform TARGET=u101-stm32l <name of your binary>.u

If you encounter chip reset / misc hardware problems during programming, just try again.


## Is there anyone I can contact for help?

For information, please contact Vilhelm Jutvik at ville@sics.se
