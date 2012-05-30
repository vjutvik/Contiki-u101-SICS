/*
 * Copyright (c) 2012, UPWIS AB, Erik Jansson (erik at upwis.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include <stdint.h>
#include "stm32-clk.h"
#include "stm32-i2c.h"

/*
  I2C operations implemented somewhat according to app. note AN2824 -
  ("STM32F10xxx I2C optimized examples") using polling read and write.
  Warning, i2c_read() makes you sad.

  It appears to be this complicated because of certain limitations
  in silicon.

  "Due to the 'Wrong data read into data register' limitation described
  in the device errata sheet, interrupts should be masked between STOP
  programming and DataN-1 reading.  Please refer to the device errata
  sheet for more details."

  "The EV6_1 software sequence must complete before the ACK pulse of
  the current byte transfer. To ensure this, interrupts must be disabled
  between ADDR clearing and ACK clearing."

  Not sure if masking of interrupts is enough, or if we actually need
  to disable *all other* interrupts in order to keep within timing
  constraints.
*/

void
stm32_i2c_init(STM32_I2C *i2c)
{
  uint32_t tmp;
  uint32_t pclk_rate;

  /* Enable peripheral clock */
  stm32_clk_pclk_enable(i2c);

  /* Changes to speed settings must be done with peripheral disabled */
  stm32_i2c_disable(i2c);

  /* Get our peripheral clock rate */
  pclk_rate = stm32_clk_frequency(stm32_clk_clkof(i2c));

  /* Set peripheral frequency (APB clock) */
  tmp = i2c->CR2 & ~(I2C_CR2_FREQ);
  tmp |= pclk_rate / 1000 * 1000;
  i2c->CR2 = tmp;

  /* Rise time. */
  tmp = pclk_rate / 1000 * 1000;
  i2c->TRISE = tmp + 1;

  /* Set ACK bit */
  i2c->CR1 |= I2C_CR1_ACK;

  /* Enable peripheral */
  stm32_i2c_enable(i2c);
}


void
stm32_i2c_set_rate(STM32_I2C *i2c, uint32_t rate_hz)
{
  uint32_t pclk_rate;

  pclk_rate = stm32_clk_frequency(stm32_clk_clkof(i2c));

  /* Set i2c clock period according to the rate we want.
     Ti2c = 2 * CCR * Tpclk   ->   
     1/fi2c = 2 * CCR * 1/fpclk   ->
     CCR = fpclk/(fi2c * 2)
  */
  i2c->CCR = pclk_rate / (2 * rate_hz);
}

/* A messy busy wait */
#define I2C_WHILE(x, t, e)              \
  do {                                  \
    volatile int _i, _j;                \
    for (_i=0; (_i < (t)) && (x); _i++) \
      for(_j=0; _j < 100; _j++)         \
        ;                               \
    if (_i >= timeout)                  \
      return e;                         \
  } while(0)


static int 
stm32_i2c_start(STM32_I2C *i2c) {

  int timeout;

  timeout = 0x100;
  
  /* Send start */
  i2c->CR1 |= I2C_CR1_START;
  
  /* Wait until start is sent and bus is switched to master */
  I2C_WHILE(!(i2c->SR1 & I2C_SR1_SB), timeout, -1);

  /* No more START? */
  /* i2c->CR1 &= ~(I2C_CR1_START); */

  return 0;
}

int 
stm32_i2c_write(STM32_I2C *i2c, uint8_t addr, const uint8_t len, uint8_t *buf, 
                uint8_t stop)
{
  volatile uint32_t tmp;
  int timeout;
  int i;

  timeout = 0x10000;

  /* LSB must be zero, otherwise it's not a write */
  if (addr & 0x01) {
    return -1;
  }
  
  if (stm32_i2c_start(i2c) < 0) {
    return -2;
  }
  
  /* Send address */
  i2c->DR = addr;
  
  /* Wait until address is set */
  I2C_WHILE(!(i2c->SR1 & I2C_SR1_ADDR), timeout, -3);

  /* Clear ADDR in SR1 by reading SR1 and SR2 */
  tmp = i2c->SR1;
  tmp = i2c->SR2;
  
  for (i=0; i<len; i++) {
    /* Send byte */
    i2c->DR = buf[i];
    /* Wait until byte is sent */
    I2C_WHILE(!(i2c->SR1 & I2C_SR1_TXE), timeout, -4);
  }

  /* Wait until BTF is set by hw */
  I2C_WHILE(!(i2c->SR1 & I2C_SR1_BTF), timeout, -5);

  if (stop) {
    /* Send stop */
    i2c->CR1 |= (I2C_CR1_STOP);
    
    /* Wait until STOP is cleared by hw */
    I2C_WHILE((i2c->CR1 & I2C_CR1_STOP), timeout, -6);
  }
  
  return 0;
}

/**
   Immensely hard to grasp receive function. 
*/
int
stm32_i2c_read(STM32_I2C *i2c, uint8_t addr, const uint8_t len, uint8_t *buf)
{
  int timeout;
  int i;
  uint8_t n;
  volatile uint32_t tmp;

  if (stm32_i2c_start(i2c) < 0) {
    return -1;
  }

  /* Send address, 1 for read */
  i2c->DR = addr | 0x01;
  
  timeout = 0x10000;
  
  /* Wait until address is set */
  I2C_WHILE(!(i2c->SR1 & I2C_SR1_ADDR), timeout, -2);

  if (len == 1) {
    
    /* ACK = 0 */
    i2c->CR1 &= ~(I2C_CR1_ACK);
                
    /* Disable interrupts */
    i2c->CR2 &= ~(I2C_CR2_ITBUFEN | 
                 I2C_CR2_ITEVTEN | 
                 I2C_CR2_ITERREN);
                
    /* Read SR1 and SR2 to clear the ADDR bit in SR1 */
    tmp = i2c->SR1;
    tmp = i2c->SR2;
                
    /* Set STOP */
    i2c->CR1 |= I2C_CR1_STOP;
    
    /* Enable interrupts */
    i2c->CR2 |= (I2C_CR2_ITBUFEN | 
                I2C_CR2_ITEVTEN | 
                I2C_CR2_ITERREN);
    
    /* Wait until RXNE is 1 */
    I2C_WHILE(!(i2c->SR1 & I2C_SR1_RXNE), timeout, -3);

    /* Read data */
    tmp = (i2c->DR);
    
    /* len = 1 so we only want one byte */
    buf[0] = tmp;
    
    /* Wait until STOP is cleared by hw */
    I2C_WHILE((i2c->CR1 & I2C_CR1_STOP), timeout, -4);

    /* ACK = 1 so we're ready for next reception */
    i2c->CR1 |= (I2C_CR1_ACK);

  } else if (len == 2) {

    /* POS = 1 */
    i2c->CR1 |= (I2C_CR1_POS);
    
    /* Disable interrupts */
    i2c->CR2 &= ~(I2C_CR2_ITBUFEN | 
                 I2C_CR2_ITEVTEN | 
                 I2C_CR2_ITERREN);
    
    /* Clear ADDR in SR1 by reading SR1 and SR2 */
    tmp = i2c->SR1;
    tmp = i2c->SR2;
    
    /* ACK = 0 */
    i2c->CR1 &= ~(I2C_CR1_ACK);
    
    /* Enable interrupts */
    i2c->CR2 |= (I2C_CR2_ITBUFEN | 
                I2C_CR2_ITEVTEN | 
                I2C_CR2_ITERREN);
    
    /* Wait for BTF == 1 */
    I2C_WHILE(!(i2c->SR1 & I2C_SR1_BTF), timeout, -5);

    /* Disable interrupts */
    i2c->CR2 &= ~(I2C_CR2_ITBUFEN | 
                 I2C_CR2_ITEVTEN | 
                 I2C_CR2_ITERREN);
    
    /* STOP = 1 */
    i2c->CR1 |= I2C_CR1_STOP;
    
    /* Wait until RXNE == 1 */
    I2C_WHILE(!(i2c->SR1 & I2C_SR1_RXNE), timeout, -6);

    /* Read data 1 (0) */
    tmp = (i2c->DR);
    
    /* len = 2, this is the first of the two bytes  */
    buf[0] = tmp;
    
    /* Enable interrupts */
    i2c->CR2 |= (I2C_CR2_ITBUFEN | 
                I2C_CR2_ITEVTEN | 
                I2C_CR2_ITERREN);

    /* Read data 2 (1) */
    tmp = i2c->DR;
    
    /* Now we get the second byte */
    buf[1] = tmp;
    
    /* Wait until STOP is cleared by hw */
    I2C_WHILE((i2c->CR1 & I2C_CR1_STOP), timeout, -7);

    /* POS = 0 */
    i2c->CR1 &= ~(I2C_CR1_POS);
    
    /* ACK = 1 so we're ready for next reception */
    i2c->CR1 |= (I2C_CR1_ACK);
    
  } else {

    /* In this case, we have at least 3 bytes to read (len >= 3).
       Read all bytes but the last two in this loop. */
    for (n=0; n<len-2; i++) {

      /* Wait until RXNE == 1 */
      I2C_WHILE(!(i2c->SR1 & I2C_SR1_RXNE), timeout, -8);

      /* Read data */
      buf[n] = i2c->DR;
    }

    /* Wait until BTF == 1 is set */
    I2C_WHILE(!(i2c->SR1 & I2C_SR1_BTF), timeout, -9);

    /* ACK = 0 */
    i2c->CR1 &= ~(I2C_CR1_ACK);
    
    /* Disable interrupts */
    i2c->CR2 &= ~(I2C_CR2_ITBUFEN | 
                 I2C_CR2_ITEVTEN | 
                 I2C_CR2_ITERREN);
    
    /* STOP = 1 */
    i2c->CR1 |= I2C_CR1_STOP;
    
    /* Read one more */
    buf[n++] = i2c->DR;
    
    /* Enable interrupts */
    i2c->CR2 |= (I2C_CR2_ITBUFEN | 
                I2C_CR2_ITEVTEN | 
                I2C_CR2_ITERREN);
    
    /* Wait until RXNE == 1 */
    I2C_WHILE(!(i2c->SR1 & I2C_SR1_RXNE), timeout, -10);

    /* Read the last byte */
    buf[n++] = i2c->DR;
             
    /* Wait until STOP is cleared by hw */
    I2C_WHILE((i2c->CR1 & I2C_CR1_STOP), timeout, -11);

    /* ACK = 1 so we're ready for next reception */
    i2c->CR1 |= I2C_CR1_ACK;
    
    return -20;
  }

  
  /* Phew. */
  return 0;
}

void
stm32_i2c_enable(STM32_I2C *i2c)
{
  i2c->CR1 |= I2C_CR1_PE;
}

void
stm32_i2c_disable(STM32_I2C *i2c)
{
  i2c->CR1 &= ~(I2C_CR1_PE);
}
