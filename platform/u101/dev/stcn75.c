#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <sys/process.h>
#include <sys/autostart.h>
#include <sys/etimer.h>
#include "ui2c.h"
#include "stcn75.h"

const ui2c_slave stcn75_i2c = {
  .master = &i2cm1,
  .rate = 100 * 1000,
  .name = "stcn75"
};

int stcn75_init(void)
{
  uint8_t buf[2];
  int res;

  ui2c_slave_init(&stcn75_i2c);

  /* Probe and configure */
  res = ui2c_read(&stcn75_i2c, 0x90, buf, 2, 1);
  if (0 != res) {
    printf("Failed to read temperature (%d)\n", res);
    return -1;
  }

  return 0;
}


int stcn75_get_temperature_degc(int *temperature)
{
  uint8_t buf[8];
  int res;
  
  /* Read the temperature */
  res = ui2c_read(&stcn75_i2c, 0x90, buf, 2, 1);
  if (0 != res) {
    printf("Failed to read temperature (%d)\n", res);
    return -1;
  }
  
  *temperature = buf[0];
  
  return 0;
}

