#ifndef USPI_H
#define USPI_H

#include <stdint.h>

typedef void (*spi_cs_fn)(int highlow);

typedef struct {
  uint32_t bus;        /**< Identifier, bus or address of this SPI master */

  /* struct spi_device *dev; */
} uspi_master;

typedef struct {
  uspi_master *master; /**< SPI master connected to the device */
  spi_cs_fn cs;        /**< Chip select function */
  uint32_t rate;       /**< Maximum frequency that device accepts in hz */
  uint8_t mode;        /**< Clock polarity and phase */
  void *p;             /**< Pointer to opaque data passed on by spi functions */
  const char *name;
  /* struct spi_device *next; */
} uspi_device;

int uspi_master_init(const uspi_master *m);
int uspi_device_init(const uspi_device *d);
void uspi_cs_high(const uspi_device *dev);
void uspi_cs_low(const uspi_device *dev);
void uspi_tx(const uspi_device *dev, uint16_t data);
uint16_t uspi_rx(const uspi_device *dev);
uint16_t uspi_txrx(const uspi_device *dev, uint16_t data);
const uspi_master *uspi_master_of(const uspi_device *d);

int uspi_arch_master_init(const uspi_master *m);
int uspi_arch_device_init(const uspi_device *d);
void uspi_arch_cs_high(const uspi_device *dev);
void uspi_arch_cs_low(const uspi_device *dev);
void uspi_arch_tx(const uspi_device *dev, uint16_t data);
uint16_t uspi_arch_rx(const uspi_device *dev);
uint16_t uspi_arch_txrx(const uspi_device *dev, uint16_t data);



#endif
