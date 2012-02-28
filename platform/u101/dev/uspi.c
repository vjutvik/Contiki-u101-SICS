
#include <stdint.h>
#include "uspi.h"

int uspi_master_init(const uspi_master *m)
{
  printf("USPI master init (bus %lx)\n", (uint32_t)m->bus);
  return uspi_arch_master_init(m);
}

int uspi_device_init(const uspi_device *d)
{
  printf("USPI device init (%s, bus %lx)\n", d->name, (uint32_t)d->master->bus);
  return uspi_arch_device_init(d);
}

void uspi_cs_high(const uspi_device *dev)
{
  dev->cs(1);
}

void uspi_cs_low(const uspi_device *dev)
{
  dev->cs(0);
}

void uspi_tx(const uspi_device *dev, uint16_t data)
{
  uspi_arch_tx(dev, data);
}

uint16_t uspi_rx(const uspi_device *dev)
{
  return uspi_arch_rx(dev);
}

uint16_t uspi_txrx(const uspi_device *dev, uint16_t data)
{
  return uspi_arch_txrx(dev, data);
}

const uspi_master *uspi_master_of(const uspi_device *dev) {
  return dev->master;
}
