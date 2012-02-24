#ifndef STM32_I2C_H
#define STM32_I2C_H

#include <stdint.h>

typedef struct {
  volatile uint16_t CR1;
  uint16_t R0;
  volatile uint16_t CR2;
  uint16_t R1;
  volatile uint16_t OAR1;
  uint16_t R2;
  volatile uint16_t OAR2;
  uint16_t R3;
  volatile uint16_t DR;
  uint16_t R4;
  volatile uint16_t SR1;
  uint16_t R5;
  volatile uint16_t SR2;
  uint16_t R6;
  volatile uint16_t CCR;
  uint16_t R7;
  volatile uint16_t TRISE;
  uint16_t R8;
} STM32_I2C;


#define I2C_CR1_PE                      ((uint16_t)0x0001)
#define I2C_CR1_SMBUS                   ((uint16_t)0x0002)
#define I2C_CR1_SMBTYPE                 ((uint16_t)0x0008)
#define I2C_CR1_ENARP                   ((uint16_t)0x0010)
#define I2C_CR1_ENPEC                   ((uint16_t)0x0020)
#define I2C_CR1_ENGC                    ((uint16_t)0x0040)
#define I2C_CR1_NOSTRETCH               ((uint16_t)0x0080)
#define I2C_CR1_START                   ((uint16_t)0x0100)
#define I2C_CR1_STOP                    ((uint16_t)0x0200)
#define I2C_CR1_ACK                     ((uint16_t)0x0400)
#define I2C_CR1_POS                     ((uint16_t)0x0800)
#define I2C_CR1_PEC                     ((uint16_t)0x1000)
#define I2C_CR1_ALERT                   ((uint16_t)0x2000)
#define I2C_CR1_SWRST                   ((uint16_t)0x8000)

#define I2C_CR2_FREQ                    ((uint16_t)0x003f)
#define I2C_CR2_ITERREN                 ((uint16_t)0x0100)
#define I2C_CR2_ITEVTEN                 ((uint16_t)0x0200)
#define I2C_CR2_ITBUFEN                 ((uint16_t)0x0400)
#define I2C_CR2_DMAEN                   ((uint16_t)0x0800)
#define I2C_CR2_LAST                    ((uint16_t)0x1000)

#define I2C_SR1_SB                      ((uint16_t)0x0001)
#define I2C_SR1_ADDR                    ((uint16_t)0x0002)
#define I2C_SR1_BTF                     ((uint16_t)0x0004)
#define I2C_SR1_ADD10                   ((uint16_t)0x0008)
#define I2C_SR1_STOPF                   ((uint16_t)0x0010)
#define I2C_SR1_RXNE                    ((uint16_t)0x0040)
#define I2C_SR1_TXE                     ((uint16_t)0x0080)
#define I2C_SR1_BERR                    ((uint16_t)0x0100)
#define I2C_SR1_ARLO                    ((uint16_t)0x0200)
#define I2C_SR1_AF                      ((uint16_t)0x0400)
#define I2C_SR1_OVR                     ((uint16_t)0x0800)
#define I2C_SR1_PECERR                  ((uint16_t)0x1000)
#define I2C_SR1_TIMEOUT                 ((uint16_t)0x4000)
#define I2C_SR1_SMBALERT                ((uint16_t)0x8000)

#define I2C_SR2_MSL                     ((uint16_t)0x0001)
#define I2C_SR2_BUSY                    ((uint16_t)0x0002)
#define I2C_SR2_TRA                     ((uint16_t)0x0004)
#define I2C_SR2_GENCALL                 ((uint16_t)0x0010)
#define I2C_SR2_SMBDEFAULT              ((uint16_t)0x0020)
#define I2C_SR2_SMBHOST                 ((uint16_t)0x0040)
#define I2C_SR2_DUALF                   ((uint16_t)0x0080)
#define I2C_SR2_PEC                     ((uint16_t)0xff00)


void stm32_i2c_disable(STM32_I2C *i2c);
void stm32_i2c_enable(STM32_I2C *i2c);
void stm32_i2c_init(STM32_I2C *i2c);
void stm32_i2c_set_rate(STM32_I2C *i2c, uint32_t rate);
int i2c_write(STM32_I2C *i2c, uint8_t addr, const uint8_t len, uint8_t *buf, 
              uint8_t nostop);

#endif

