
#include "uspi.h"
#include "stm32-spi.h"

int uspi_arch_master_init(uspi_master *m) {
  return stm32_spi_init(stm32_spi_num2bus(m->bus));
}

/* Does NOT configure I/O pins. */
int uspi_arch_device_init(uspi_master *m, uspi_device *d) {
  return 0;
}

void uspi_arch_cs_high(uspi_device *dev) {
  STM32_SPI *spi;
  spi = stm32_spi_num2bus(dev->master->bus);

  dev->cs(1);
  stm32_spi_disable(spi);

  /* Maybe disable peripheral clock to save power? */
}

void uspi_arch_cs_low(uspi_device *dev) {
  STM32_SPI *spi;
  uint32_t rate;

  spi = stm32_spi_num2bus(dev->master->bus);
  
  stm32_spi_set_rate(spi, dev->rate, pclk);
  stm32_spi_enable(spi);
  dev->cs(USPI_CS_LOW);
}

void uspi_arch_tx(uspi_device *dev, uint16_t data) {
  stm32_spi *spi;
  spi = (stm32_spi *)dev->master->bus;

  while(!(sb->SR & SPI_SR_TXE))
    ;

  spi->DR = data;
}

int uspi_arch_rx(uspi_device *dev) {
  stm32_spi *spi;
  spi = (stm32_spi *)dev->master->bus;

  while(!(sb->SR & SPI_SR_RXNE))
    ;
  return sb->DR;
}

uint16_t uspi_arch_txrx(uspi_device *dev, uint16_t data) {
  stm32_spi *spi;
  spi = (stm32_spi *)dev->master->bus;

  uspi_arch_tx(dev, data);
  return uspi_arch_rx(dev);
}

