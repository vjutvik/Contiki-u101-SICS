#include <stdint.h>
#include <stdio.h>
#include "stm32-i2c.h"
#include "ui2c.h"
#include "stm32l.h"
#include "stm32l-gpio.h"

#if 0
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

int 
ui2c_arch_master_init(const ui2c_master *m)
{
  STM32_I2C *bus;
  bus = (STM32_I2C *)m->bus;

  if (bus == I2C1) {

    /* Configure GPIO for I2C pins */
    stm32l_gpio_conf_af(GPIOB, 6, GPIO_OUTPUT_TYPE_ODRAIN, 
                        GPIO_OSPEED_40MHZ, GPIO_RESISTORS_NONE);
    stm32l_gpio_conf_af(GPIOB, 7, GPIO_OUTPUT_TYPE_ODRAIN, 
                        GPIO_OSPEED_40MHZ, GPIO_RESISTORS_NONE);
    stm32l_gpio_map_af(GPIOB, 6, GPIO_AF_I2C1);
    stm32l_gpio_map_af(GPIOB, 7, GPIO_AF_I2C1);

    /* Enable clock to I2C peripheral */
    stm32_clk_pclk_enable(I2C1);

  } else if (bus == I2C2) {
    PRINTF("I2C bus 2 not yet supported\n");
    return -1;
  } else {
    printf("Invalid I2C bus (%08lx)\n", (uint32_t)bus);
    return -1;
  }

  /* Initiate I2C peripheral */
  stm32_i2c_init(bus);
  printf("UI2C master init (bus %08lx)\n", (uint32_t)bus);
  return 0;
}

int
ui2c_arch_slave_init(const ui2c_slave *d)
{
  printf("UI2C slave init (%s, bus %08lx)\n", d->name, d->master->bus);
  return 0;
}

/* TODO: Move addr to the ui2c_slave struct? */
int
ui2c_arch_write(const ui2c_slave *dev, uint8_t addr, 
                uint8_t *buf, uint8_t len, uint8_t stop)
{
  STM32_I2C *periph;
  int r;
  periph = (STM32_I2C *)dev->master->bus;
  PRINTF("i2c_write() - ");
  r = stm32_i2c_write(periph, addr, len, buf, stop);
  if (0 != r) {
    PRINTF("failed (%d)\n", r);
  } else {
    PRINTF("ok\n");
  }
  return r;
}

/* TODO: Move addr to the ui2c_slave struct? */
int
ui2c_arch_read(const ui2c_slave *dev, uint8_t addr,
               uint8_t *buf, uint8_t len, uint8_t stop)
{
  STM32_I2C *bus;
  int r;
  bus = (STM32_I2C *)dev->master->bus;
  stm32_i2c_set_rate(bus, dev->rate);
  PRINTF("i2c_read() - ");
  r = stm32_i2c_read(bus, addr, len, buf);
  if (0 != r) {
    PRINTF("failed (%d)\n", r);
  } else {
    PRINTF("ok\n");
  }
  return r;
}
