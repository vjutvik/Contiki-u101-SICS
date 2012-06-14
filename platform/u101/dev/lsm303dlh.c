#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <sys/process.h>
#include <sys/autostart.h>
#include <sys/etimer.h>
#include "ui2c.h"
#include "platform-conf.h"
#include "lsm303dlh.h"

const ui2c_slave lsm303dlh_i2c = {
  .master = &i2cm1,
  .rate = 100 * 1000,
  .name = "lsm303"
};


/* We don't handle reading multiple bytes yet. */
int lsm303_read(uint8_t addr, uint8_t reg, uint8_t *data)
{
  uint8_t buf[16];
  int res;
  /* printf("lsm303_read\n"); */
  buf[0] = reg;

  res = ui2c_write(&lsm303dlh_i2c, addr, buf, 1, 0);
  if (0 != res) {
    printf("lsm303_read i2c_write failed (%d)\n", res);
    return res;
  } 
  res = ui2c_read(&lsm303dlh_i2c, addr, buf, 1, 1);
  if (0 != res) {
    printf("lsm303_read i2c_read failed (%d)\n", res);
    return res;
  }

  data[0] = buf[0];
  return 0;
}

int lsm303_write(uint8_t addr, uint8_t reg, uint8_t data)
{
  uint8_t buf[16];
  int res;
  /* printf("lsm303_write\n"); */
  buf[0] = reg;
  buf[1] = data;
  res = ui2c_write(&lsm303dlh_i2c, addr, buf, 2, 1);
  return res;
}

int lsm303_get(uint8_t rawacc[6], uint8_t rawmag[6])
{
  int res;
  int i;

  for (i=0; i<6; i++) {
    res = lsm303_read(LSM303_I2CADDR_ACC, 
                      LSM303_ACCDATA_REG+i, 
                      &rawacc[i]);
    if (0 != res) {
      printf("Failed to read at i = %d\n", i);
      break;
    }
  }

  if (i<6) {
    return -1;
  }

  for (i=0; i<6; i++) {
    res = lsm303_read(LSM303_I2CADDR_MAG, 
                      LSM303_MAGDATA_REG+i, 
                      &rawmag[i]);
    if (0 != res) {
      break;
    }
  }

  if (i<6) {
    return -1;
  }


  return 0;
}

void lsm303_cook(uint8_t raw[6], int cooked[3], int shift)
{
  cooked[0] = (int) ((raw[1]) << 8) | raw[0];
  cooked[1] = (int) ((raw[3]) << 8) | raw[2];
  cooked[2] = (int) ((raw[5]) << 8) | raw[4];

  if (cooked[0] & 0x8000) { 
    cooked[0] = cooked[0] | 0xffff0000; 
  }
  if (cooked[1] & 0x8000) { 
    cooked[1] = cooked[1] | 0xffff0000; 
  }
  if (cooked[2] & 0x8000) { 
    cooked[2] = cooked[2] | 0xffff0000; 
  }

  cooked[0] >>= shift;
  cooked[1] >>= shift;
  cooked[2] >>= shift;
}

int lsm303_get_xacc_newton(int *acc)
{
  int res;
  uint8_t rawacc[6];
  uint8_t rawmag[6];
  int lsm303acc[3];

  res = lsm303_get(rawacc, rawmag);
  if (0 != res) {
    printf("Problem getting\n");
    return -1;
  }
  lsm303_cook(rawacc, lsm303acc, 4);

  *acc = lsm303acc[0];
  return 0;
}

int lsm303_sleep(uint32_t i2c_bus)
{
  int res;
  res = lsm303_write(LSM303_I2CADDR_ACC, 0x20, 0x00);
  if (0 != res) {
    printf("lsm303dlh: Failed to write (%d)\n", res);
    return res;
  }
  return 0;
}

int lsm303_init(uint32_t i2c_bus)
{
  int res;
  int r;

  ui2c_arch_slave_init(&lsm303dlh_i2c);

  /* Init accelerometer */
  res = lsm303_write(LSM303_I2CADDR_ACC, 0x20, 0x67);
  if (0 != res) {
    printf("lsm303dlh: Failed to write (%d)\n", res);
    return res;
  }

  /* Init magnetometer */

  /* Continuous */
  res = lsm303_write(LSM303_I2CADDR_MAG, 0x02, 0x00);
  if (0 != res) {
    printf("lsm303dlh_init: Failed to write (%d)\n", res);
  }

  return 0;
}


#if 0
PROCESS(lsm303_process, "LSM303 process");
PROCESS_THREAD(lsm303_process, ev , data)
{
  static struct etimer timer;
  static int n;
  int res;
  int i;
  uint8_t rawacc[6];
  uint8_t rawmag[6];

  PROCESS_BEGIN();

  printf("lsm303 process\n");

  i2c_init(I2C_BUS_1, 0);

  for (i=0; i<0x1000; i++)
    ;

  lsm303_init(0);

  n=0;
	
  while (1) {
    /* Delay */
    etimer_set(&timer, CLOCK_SECOND / 1);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    res = lsm303_get(rawacc, rawmag);
    if (0 != res) {
      printf("Problem getting\n");
      break;
    }

    lsm303_cook(rawacc, lsm303acc, 4);
    lsm303_cook(rawmag, lsm303mag, 4);
#if 0
    if ((n % 10) == 0) {
      printf("(%d\tY,%d\tZ,%d)\n", 
             lsm303acc[0], lsm303acc[1], lsm303acc[2]);
    }
#endif
    //leds_toggle(LEDS_ALL);
    n++;
  }	
  printf("Exiting lsm303 process\n");
  PROCESS_END();
}
#endif
