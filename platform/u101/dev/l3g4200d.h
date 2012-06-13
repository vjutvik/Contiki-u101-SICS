#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#include "sys/process.h"

#define L3G4200D_I2CADDR        0xd0

#define WHO_AM_I                0x0F
#define CTRL_REG1               0x20
#define CTRL_REG2               0x21
#define CTRL_REG3               0x22
#define CTRL_REG4               0x23
#define CTRL_REG5               0x24
#define REFERENCE               0x25
#define OUT_TEMP                0x26
#define STATUS_REG              0x27
#define OUT_X_L                 0x28

int l3g4200d_init(void);
int l3g4200d_on(void);
int l3g4200d_off(void);
int l3g4200d_get(uint8_t rawgyr[6]);
void l3g4200d_cook(uint8_t raw[6], int cooked[3]);

#endif
