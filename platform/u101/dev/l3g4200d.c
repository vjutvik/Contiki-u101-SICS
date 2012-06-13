#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <sys/process.h>
#include <sys/autostart.h>
#include <sys/etimer.h>
#include <leds.h>
#include "ui2c.h"
#include "l3g4200d.h"

const ui2c_slave l3g4200d_i2c = {
  .master = &i2cm1,
  .rate = 100 * 1000,
  .name = "l3g4200d"
};


int l3g4200d_read(uint8_t addr, uint8_t reg, uint8_t *data)
{
  uint8_t buf[16];
  int res;

  buf[0] = reg;

  res = ui2c_write(&l3g4200d_i2c, addr, buf, 1, 0);
  if (0 != res) {
    printf("lsm303_read i2c_write failed (%d)\n", res);
    return res;
  } 
  res = ui2c_read(&l3g4200d_i2c, addr, buf, 1, 1);
  if (0 != res) {
    printf("lsm303_read i2c_read failed (%d)\n", res);
    return res;
  }
  
  data[0] = buf[0];
  return 0;
}

int l3g4200d_write(uint8_t addr, uint8_t reg, uint8_t data)
{
  uint8_t buf[16];
  int res;

  buf[0] = reg;
  buf[1] = data;
  res = ui2c_write(&l3g4200d_i2c, addr, buf, 2, 1);
  return res;
}

int l3g4200d_get(uint8_t rawgyr[6])
{
  int res;
  int i;

  for (i=0; i<6; i++) {
    res = l3g4200d_read(L3G4200D_I2CADDR, 
                        OUT_X_L+i, 
                        &rawgyr[i]);
    printf("%d: %d\n", i, rawgyr[i]);
    if (0 != res) {
      printf("Failed to read at i = %d\n", i);
      break;
    }
  }

  return 0;
}

int l3g4200d_init(void)
{

  int res;
  uint8_t tmp;

  res = l3g4200d_read(L3G4200D_I2CADDR, 
                      WHO_AM_I,
                      &tmp);
  if (0 != res) {
    printf("Couldn't identify gyro\n");
    return -1;
  } else {
    if (tmp != 0xd3) {
      printf("This is not the gyro we are looking for\n");
      return -1;
    }
  }

  /* Start */
  res = l3g4200d_write(L3G4200D_I2CADDR, CTRL_REG1, 0x0f);
  if (0 != res) {
    printf("Couldn't start gyro\n");
  }

  /* 250 degrees/s*/
  res = l3g4200d_write(L3G4200D_I2CADDR, CTRL_REG4, 0x00);
  if (0 != res) {
    printf("Couldn't start gyro\n");
  }

  return 0;
}

void l3g4200d_cook(uint8_t raw[6], int cooked[3])
{
  cooked[0] = (int) ((raw[1]) << 8) | raw[0];
  cooked[1] = (int) ((raw[3]) << 8) | raw[2];
  cooked[2] = (int) ((raw[5]) << 8) | raw[4];

}
