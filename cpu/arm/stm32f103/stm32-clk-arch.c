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

#include <stdlib.h>
#include "stm32-clk.h"
#include "stm32f10x_map.h"

const uint32_t osc_in = 8 * 1000 * 1000l;

static uint32_t get_pllmul(void)
{
  uint32_t pllmul = RCC->CFGR & RCC_CFGR_PLLMULL;
  uint32_t mul;
  switch(pllmul) {
  case RCC_CFGR_PLLMULL2: mul = 2; break;
  case RCC_CFGR_PLLMULL3: mul = 3; break;
  case RCC_CFGR_PLLMULL4: mul = 4; break;
  case RCC_CFGR_PLLMULL5: mul = 5; break;
  case RCC_CFGR_PLLMULL6: mul = 6; break;
  case RCC_CFGR_PLLMULL7: mul = 7; break;
  case RCC_CFGR_PLLMULL8: mul = 8; break;
  case RCC_CFGR_PLLMULL9: mul = 9; break;
  case RCC_CFGR_PLLMULL10: mul = 10; break;
  case RCC_CFGR_PLLMULL11: mul = 11; break;
  case RCC_CFGR_PLLMULL12: mul = 12; break;
  case RCC_CFGR_PLLMULL13: mul = 13; break;
  case RCC_CFGR_PLLMULL14: mul = 14; break;
  case RCC_CFGR_PLLMULL15: mul = 15; break;
  case RCC_CFGR_PLLMULL16: mul = 16; break;
  default: mul = 0; break;
  }
  return mul;
}

static uint32_t get_hpre(void)
{
  uint32_t plldiv = RCC->CFGR & RCC_CFGR_HPRE;
  uint32_t div;
  switch(plldiv) {
  case RCC_CFGR_HPRE_DIV1: div = 1; break;
  case RCC_CFGR_HPRE_DIV2: div = 2; break;
  case RCC_CFGR_HPRE_DIV4: div = 4; break;
  case RCC_CFGR_HPRE_DIV8: div = 8; break;
  case RCC_CFGR_HPRE_DIV16: div = 16; break;
  case RCC_CFGR_HPRE_DIV64: div = 64; break;
  case RCC_CFGR_HPRE_DIV128: div = 128; break;
  case RCC_CFGR_HPRE_DIV256: div = 256; break;
  case RCC_CFGR_HPRE_DIV512: div = 512; break;
  default: div = 1; break;
  }
  return div;
}

static uint32_t get_ppre1(void)
{
  uint32_t ppre;
  uint32_t div;
  ppre = RCC->CFGR & RCC_CFGR_PPRE1;
  switch(ppre) {
  case RCC_CFGR_PPRE1_DIV1: div = 1; break;
  case RCC_CFGR_PPRE1_DIV2: div = 2; break;
  case RCC_CFGR_PPRE1_DIV4: div = 4; break;
  case RCC_CFGR_PPRE1_DIV8: div = 8; break;
  case RCC_CFGR_PPRE1_DIV16: div = 16; break;
  default: div = 1; break;
  }
  return div;
}

static uint32_t get_ppre2(void)
{
  uint32_t ppre;
  uint32_t div;
  ppre = RCC->CFGR & RCC_CFGR_PPRE2;
  switch(ppre) {
  case RCC_CFGR_PPRE2_DIV1: div = 1; break;
  case RCC_CFGR_PPRE2_DIV2: div = 2; break;
  case RCC_CFGR_PPRE2_DIV4: div = 4; break;
  case RCC_CFGR_PPRE2_DIV8: div = 8; break;
  case RCC_CFGR_PPRE2_DIV16: div = 16; break;
  default: div = 1; break;
  }
  return div;
}



/*
  Three different clock sources can be used to drive the system clock
  (SYSCLK):

 * HSI ((high-speed internal) oscillator clock
 * HSE (high-speed external) oscillator clock
 * PLL clock
*/
uint32_t stm32f_clk_sysclk(void)
{
  uint32_t sysclk;
  const uint32_t hsiclk = 8 * MHZ;
  const uint32_t hseclk = osc_in;
  uint32_t clk_conf = RCC->CFGR & RCC_CFGR_SWS;
  uint32_t pllsrc = RCC->CFGR & RCC_CFGR_PLLSRC;
  uint32_t pllin;
  
  switch (clk_conf) {
  case RCC_CFGR_SWS_HSI:
    sysclk = hsiclk;
    break;
  case RCC_CFGR_SWS_HSE:
    sysclk = osc_in;
    break;
  case RCC_CFGR_SWS_PLL:
    if (pllsrc) {
      pllin = hseclk;
    } else {
      pllin = hsiclk / 2;
    }
    sysclk = pllin * get_pllmul();
    break;
  default:
    sysclk = 0;
  }
  return sysclk;
}


uint32_t stm32f_clk_hclk(void)
{
  return stm32f_clk_sysclk() / get_hpre();
}

uint32_t stm32f_clk_pclk1(void)
{
  return (stm32f_clk_sysclk() / get_hpre()) / get_ppre1();
}

uint32_t stm32f_clk_pclk2(void)
{
  return (stm32f_clk_sysclk() / get_hpre()) / get_ppre2();
}

/** 
    SYSCLK
*/
stm32_clk sys_clk = {
  .freq = stm32f_clk_sysclk,
};

/** 
    AHB bus clock
*/
stm32_clk ahb_clk = {
  .freq = stm32f_clk_hclk,
};

/** 
    Clock for APB bus 1 (APB1 clock 1, PCLK1, etc) 
*/
stm32_clk apb1_clk = {
  .freq = stm32f_clk_pclk1,
};

/** 
    Clock for APB bus 2 (APB1 clock 2, PCLK2, etc)
*/
stm32_clk apb2_clk = {
  .freq = stm32f_clk_pclk2,
};

stm32_clk *stm32_clk_arch_clkof(void *periph)
{
  switch ((uint32_t)periph) {
    /*
  case ADC1_BASE:
  case ADC2_BASE:
  case ADC3_BASE:
  case TIM1_BASE:
  case TIM8_BASE:
    */
  case SPI1_BASE:
  case USART1_BASE:
    return &apb2_clk;
    break;
    /*
  case DAC_BASE:
  case PWR_BASE:
  case BKP_BASE:
  case CAN_BASE:
  case I2C1_BASE:
  case I2C2_BASE:
  case UART4_BASE:
  case UART5_BASE:
    */
  case SPI2_BASE:
  case USART2_BASE:
  case USART3_BASE:
    return &apb1_clk;
    break;

  default:
    /* Badness */
    return NULL;
    break;
  }
}

static void stm32f10x_switch_pclk(void *periph, int enable)
{
  volatile uint32_t *reg;
  uint32_t bit;

  reg = NULL;

  switch ((uint32_t)periph) {

    /* This indentation style hopefully makes this more readable and
       not less... */

  case USART1_BASE:
    reg = &(RCC->APB2ENR);  bit = RCC_APB2ENR_USART1EN;  break;
  case USART2_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_USART2EN;  break;
  case USART3_BASE: 
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_USART3EN;  break;

  case SPI1_BASE:
    reg = &(RCC->APB2ENR);  bit = RCC_APB2ENR_SPI1EN;  break;
  case SPI2_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_SPI2EN;  break;

  default:
    /* No such peripheral - we should probably do something noisy here */
    break;
  }

  if (!reg) {
    return;
  }

  /* Enable or disable? */
  if (enable) {
    *reg |= bit;
  } else {
    *reg &= ~(bit);
  }
}

void stm32_clk_arch_pclk_enable(void *periph)
{
  stm32f10x_switch_pclk(periph, 1);
}

void stm32_clk_arch_pclk_disable(void *periph)
{
  stm32f10x_switch_pclk(periph, 0);
}
