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

#ifndef STM32_CLK_H
#define STM32_CLK_H

#include <stdint.h>

#define KHZ (1000)
#define MHZ (1000 * KHZ)

typedef uint32_t (*stm32_clk_freq_fn)(void);
typedef void (*stm32_clk_enable_fn)(int);

typedef struct {
  stm32_clk_freq_fn freq;
  /* We might want to add something more here to track stuff for power
     management later on. */
} stm32_clk;

uint32_t stm32_clk_frequency(stm32_clk *clk);

stm32_clk *stm32_clk_clkof(void *periph);
void stm32_clk_pclk_enable(void *periph);
void stm32_clk_pclk_disable(void *periph);

/* An STM32 port should implement these. */

/* Given a peripheral base address, it should return an stm32_clk which
   is the clock of the bus which the peripheral sits on. */
stm32_clk *stm32_clk_arch_clkof(void *periph);

/* Given a peripheral base address, enables the clock to that peripheral. */
void stm32_clk_arch_pclk_enable(void *periph);

/* Given a peripheral base address, disables the clock to that peripheral. */
void stm32_clk_arch_pclk_disable(void *periph);


#endif
