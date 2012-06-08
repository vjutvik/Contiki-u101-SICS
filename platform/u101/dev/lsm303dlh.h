#ifndef __LSM303DLH_H__
#define __LSM303DLH_H__

#include "sys/process.h"

#define LSM303_I2CADDR_ACC  0x30
#define LSM303_I2CADDR_MAG  0x3c

#define LSM303_ACCDATA_REG 0x28

#define LSM303_MAGDATA_REG 0x03

extern int lsm303acc[3];
extern int lsm303mag[3];

int lsm303_init(uint32_t i2c_bus);
int lsm303_get(uint8_t rawacc[6], uint8_t rawmag[6]);
void lsm303_cook(uint8_t raw[6], int cooked[3], int shift);
int lsm303_sleep(uint32_t i2c_bus);
int lsm303_get_xacc_newton(int *acc);

PROCESS_NAME(lsm303_process);

#endif
