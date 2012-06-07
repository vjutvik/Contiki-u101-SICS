#ifndef UI2C_H
#define UI2C_H

#include <stdint.h>

typedef struct {
  uint32_t bus;        /**< Identifier, bus or address of this I2C master */

  /* struct ui2c_device *dev; */
} ui2c_master;

typedef struct {
  const ui2c_master *master; /**< I2C master connected to the device */
  uint32_t rate;       /**< Maximum frequency that device accepts in hz */
  void *p;             /**< Pointer to opaque data passed on by i2c functions */
  const char *name;
  /* uint8_t addr; */
  /* struct ui2c_device *next; */
} ui2c_slave;

#define ui2c_master_init ui2c_arch_master_init
#define ui2c_slave_init ui2c_arch_slave_init
#define ui2c_write ui2c_arch_write
#define ui2c_read ui2c_arch_read

int ui2c_arch_master_init(const ui2c_master *m);
int ui2c_arch_slave_init(const ui2c_slave *d);
int ui2c_arch_write(const ui2c_slave *dev, uint8_t addr, 
                    uint8_t *buf, uint8_t len, uint8_t stop);
int ui2c_arch_read(const ui2c_slave *dev, uint8_t addr, 
                   uint8_t *buf, uint8_t len, uint8_t stop);


#endif
