
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include "stm32-spi.h"
#include "uspi.h"

static STM32_SPI * 
dev2periph(const uspi_device *d) 
{
  STM32_SPI *spi;
  spi = (STM32_SPI *)uspi_master_of(d)->bus;
  return spi;
}

int uspi_arch_master_init(const uspi_master *m)
{
  STM32_SPI *spi;
  spi = (STM32_SPI *)m->bus;
  stm32_spi_init(spi);
  return 0;
}

int uspi_arch_device_init(const uspi_device *d)
{
  STM32_SPI *spi = dev2periph(d);
  stm32_spi_disable(spi);
  stm32_spi_set_rate(spi, d->rate);
  stm32_spi_enable(spi);
  return 0;
}

void uspi_arch_tx(const uspi_device *dev, uint16_t data)
{
  STM32_SPI *spi = dev2periph(dev);
  stm32_spi_tx(spi, data);
}

uint16_t uspi_arch_rx(const uspi_device *dev)
{
  STM32_SPI *spi = dev2periph(dev);
  return stm32_spi_rx(spi);
}

uint16_t uspi_arch_txrx(const uspi_device *dev, uint16_t data)
{
  STM32_SPI *spi = dev2periph(dev);
  return stm32_spi_txrx(spi, data);
}


