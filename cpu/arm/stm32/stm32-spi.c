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

#include <stdio.h>
#include <stdint.h>
#include "stm32-clk.h"
#include "stm32-spi.h"


/**
   Set up SPI controller in master mode with software CS management.
*/
void stm32_spi_init(STM32_SPI *spi)
{
  stm32_clk_pclk_enable(spi);

  spi->CR1 =
    SPI_CR1_MSTR |
    SPI_CR1_SSM |
    SPI_CR1_SSI;

  stm32_spi_enable(spi);
}

/**
   Set the highest rate possible given a maximum rate using the rate
   of the clock clocking the SPI peripheral and the divisor.
*/
void
stm32_spi_set_rate(STM32_SPI *spi, uint32_t max_rate)
{
  uint32_t clkdiv;
  uint32_t actual_rate;
  uint32_t pclk_rate;
  uint32_t enabled;

  /* Check if SPI is enabled, if so disable first */
  enabled = spi->CR1 & SPI_CR1_SPE;
  if (enabled) {
    stm32_spi_disable(spi);
  }

  pclk_rate = stm32_clk_frequency(stm32_clk_clkof(spi));
  clkdiv = 0;

  do {
    clkdiv++;
    actual_rate = pclk_rate / (1 << clkdiv);
  } while (max_rate < actual_rate && (clkdiv) < 8);

  printf("SPI rate: %lu Hz (device max: %lu Hz), clkdiv: %lu\n",
         actual_rate, max_rate, clkdiv);

  /* Set new rate */
  spi->CR1 &= ~(SPI_CR1_BR);
  spi->CR1 |= ((clkdiv << 3) & SPI_CR1_BR);

  /* If SPI was enabled before, enable it again */
  if (enabled) {
    stm32_spi_enable(spi);
  }
}

void 
stm32_spi_set_mode(STM32_SPI *spi, uint32_t mode)
{
  spi->CR1 |= mode;
}

uint16_t
stm32_spi_rx(STM32_SPI *spi)
{
  while(!(spi->SR & SPI_SR_RXNE))
    ;
  return spi->DR;
}

void
stm32_spi_tx(STM32_SPI *spi, uint16_t data)
{
  while(!(spi->SR & SPI_SR_TXE))
    ;
  spi->DR = data;
}

uint16_t
stm32_spi_txrx(STM32_SPI *spi, uint16_t data)
{
  (void)stm32_spi_tx(spi, data);

  return stm32_spi_rx(spi);
}

void
stm32_spi_enable(STM32_SPI *spi)
{
  spi->CR1 |= SPI_CR1_SPE;
}

void
stm32_spi_disable(STM32_SPI *spi)
{
  spi->CR1 &= ~(SPI_CR1_SPE);
}
