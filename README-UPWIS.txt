## What's this?

This is an *unofficial* branch of UPWIS' port of Contiki to STM32L/F. It also features support for various hardware devices found on the company's UPWIS -motes.


## How do I use it with Contiki-main?
Go to the directory of your app / project in your Contiki main branch, then type the following to compile:

make TARGET=u101-stm32l TARGETDIRS=../../../contiki-u101/platform

You can program your UPWIS mote by executing the following:

make OPENOCD_CFG=interface/neodb.cfg OPENOCD="sudo openocd" TARGETDIRS=../../../contiki-u101/platform TARGET=u101-stm32l <name of your binary>.u


## Is there anyone I can contact for help?

For information, please contact Vilhelm Jutvik at ville@sics.se
