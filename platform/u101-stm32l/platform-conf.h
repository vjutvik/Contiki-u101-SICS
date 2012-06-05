#ifndef PLATFORM_CONF_H
#define PLATFORM_CONF_H

#include <stdint.h>
#include "uspi.h"
#include "stm32-spi.h"
#include "stm32l-gpio.h"

extern const uspi_master spim1;
extern const uspi_master spim2;

extern const uspi_device rf230_spi;

#endif
