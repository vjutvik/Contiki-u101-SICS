#ifndef PLATFORM_CONF_H
#define PLATFORM_CONF_H

#include <stdint.h>
#include "uspi.h"
#include "ui2c.h"
#include "stm32-spi.h"
#include "stm32l-gpio.h"

extern const uspi_master spim1;
extern const uspi_master spim2;

extern const uspi_device rf230_spi;

extern const ui2c_master i2cm1;

extern const ui2c_slave lsm303dlh_i2c;
extern const ui2c_slave stcn75_i2c;

#endif
