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
#include <stdlib.h>
#include "stm32-clk.h"
#include "stm32l-rcc.h"
#include "stm32l.h"
#include "contiki-conf.h"

/* Frequency in HZ of the crystal oscillator input.
   Should be defined in platform-conf.h */
const uint32_t osc_in = OSC_IN_FREQ;

static uint32_t get_pllmul(void)
{
        uint32_t pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
        uint32_t mul;
        switch(pllmul) {
        case RCC_CFGR_PLLMUL3: mul = 3; break;
        case RCC_CFGR_PLLMUL4: mul = 4; break;
        case RCC_CFGR_PLLMUL6: mul = 6; break;
        case RCC_CFGR_PLLMUL8: mul = 8; break;
        case RCC_CFGR_PLLMUL12: mul = 12; break;
        case RCC_CFGR_PLLMUL16: mul = 16; break;
        case RCC_CFGR_PLLMUL24: mul = 24; break;
        case RCC_CFGR_PLLMUL32: mul = 32; break;
        case RCC_CFGR_PLLMUL48: mul = 48; break;
        default: mul = 0; break;
        }
        return mul;
}

static uint32_t get_plldiv(void)
{
        uint32_t plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
        uint32_t div;
        switch(plldiv) {
        case RCC_CFGR_PLLDIV1: div = 1; break;
        case RCC_CFGR_PLLDIV2: div = 2; break;
        case RCC_CFGR_PLLDIV3: div = 3; break;
        case RCC_CFGR_PLLDIV4: div = 4; break;
        default: div = 1; break;
        }
        return div;
}

static uint32_t get_hpre(void)
{
        uint32_t hpre = RCC->CFGR & RCC_CFGR_HPRE;
        uint32_t div;
        switch(hpre) {
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

  Four different clock sources can be used to drive the system clock
  (SYSCLK):

 * HSI ((high-speed internal) oscillator clock
 * HSE (high-speed external) oscillator clock
 * PLL clock
 * MSI (multispeed internal) oscillator clock
a
 The MSI is used as system clock source after startup from Reset,
 wake-up from Stop or Standby low power modes.

*/
uint32_t stm32l_clk_sysclk(void)
{
        uint32_t sysclk;
        const uint32_t hsiclk = 16 * MHZ;
        const uint32_t hseclk = osc_in;
        uint32_t clk_conf = RCC->CFGR & RCC_CFGR_SWS;
        uint32_t pllsrc = RCC->CFGR & RCC_CFGR_PLLSRC;
        uint32_t pllin;
        const uint32_t icsrange = RCC->ICSCR & RCC_ICSCR;

        switch (clk_conf) {
                 
        case RCC_CFGR_SWS_MSI:
                /* Affected by RCC_ICSCR but the default value is this. */

                switch (icsrange) {
                case RCC_ICSCR_MSIRANGE_65KHZ:
                        sysclk = 65536;
                        break;
                case RCC_ICSCR_MSIRANGE_131KHZ:
                        sysclk = 131072;
                        break;
                case RCC_ICSCR_MSIRANGE_252KHZ:
                        sysclk = 263144;
                        break;
                case RCC_ICSCR_MSIRANGE_524KHZ:
                        sysclk = 524288;
                        break;
                case RCC_ICSCR_MSIRANGE_1048KHZ:
                        sysclk = 1048000;
                        break;
                case RCC_ICSCR_MSIRANGE_2097KHZ:
                        sysclk = 2097000;
                        break;
                case RCC_ICSCR_MSIRANGE_4194KHZ:
                        sysclk = 4194000;
                        break;
                default:
                        sysclk = 0;
                        break;
                }
                sysclk = 2097000;
                break;
        case RCC_CFGR_SWS_HSI:
                sysclk = hsiclk;
                break;
        case RCC_CFGR_SWS_HSE:
                sysclk = osc_in;
                break;
        case RCC_CFGR_SWS_PLL:
                if (RCC_CFGR_PLLSRC_HSI == pllsrc) {
                        pllin = hsiclk;
                } else {
                        pllin = hseclk;
                }
                sysclk = pllin * get_pllmul() / get_plldiv();
                break;
        default:
                sysclk = 0;
        }
        return sysclk;
}


uint32_t stm32l_clk_hclk(void)
{
        return stm32l_clk_sysclk() / get_hpre();
}

uint32_t stm32l_clk_pclk1(void)
{
        return (stm32l_clk_sysclk() / get_hpre()) / get_ppre1();
}

uint32_t stm32l_clk_pclk2(void)
{
        return (stm32l_clk_sysclk() / get_hpre()) / get_ppre2();
}

/** 
    SYSCLK
*/
stm32_clk sys_clk = {
  .freq = stm32l_clk_sysclk,
};

/**
    AHB bus clock
*/
stm32_clk ahb_clk = {
  .freq = stm32l_clk_hclk,
};

/** 
    Clock for APB bus 1 (APB1 clock 1, PCLK1, etc) 
*/
stm32_clk apb1_clk = {
  .freq = stm32l_clk_pclk1,
};

/** 
    Clock for APB bus 2 (APB1 clock 2, PCLK2, etc)
*/
stm32_clk apb2_clk = {
  .freq = stm32l_clk_pclk2,
};

stm32_clk *stm32_clk_arch_clkof(void *periph)
{
  switch ((uint32_t)periph) {
    /*
  case ADC1_BASE:
  case ADC2_BASE:
  case ADC3_BASE:
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
  case I2C2_BASE:
  case UART4_BASE:
  case UART5_BASE:
    */
  case I2C1_BASE:
  case TIM2_BASE:
  case TIM3_BASE:
  case TIM4_BASE:
  case TIM6_BASE:
  case TIM7_BASE:
  case USART2_BASE:
  case USART3_BASE:
    return &apb1_clk;
    break;
  case GPIOA_BASE:
  case GPIOB_BASE:
  case GPIOC_BASE:
  case GPIOD_BASE:
  case GPIOE_BASE:
  case GPIOH_BASE:
    return &ahb_clk;
    break;
  default:
    /* Badness */
    printf("Bad peripheral (%lx)\n", periph);
    while(1)
      ;
    return NULL;
    break;
  }
}

static void stm32l_switch_pclk(void *periph, int enable)
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
  case GPIOA_BASE:
    reg = &(RCC->AHBENR);   bit = RCC_AHBENR_GPIOAEN;    break;
  case GPIOB_BASE:
    reg = &(RCC->AHBENR);   bit = RCC_AHBENR_GPIOBEN;    break;
  case GPIOC_BASE:
    reg = &(RCC->AHBENR);   bit = RCC_AHBENR_GPIOCEN;    break;
  case GPIOD_BASE:
    reg = &(RCC->AHBENR);   bit = RCC_AHBENR_GPIODEN;    break;
  case GPIOE_BASE:
    reg = &(RCC->AHBENR);   bit = RCC_AHBENR_GPIOEEN;    break;
  case GPIOH_BASE:
    reg = &(RCC->AHBENR);   bit = RCC_AHBENR_GPIOHEN;    break;
  case TIM2_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_TIM2EN;    break;
  case TIM3_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_TIM3EN;    break;
  case TIM4_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_TIM4EN;    break;
  case TIM6_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_TIM6EN;    break;
  case TIM7_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_TIM7EN;    break;
  case SPI1_BASE:
    reg = &(RCC->APB2ENR);  bit = RCC_APB2ENR_SPI1EN;    break;
  case I2C1_BASE:
    reg = &(RCC->APB1ENR);  bit = RCC_APB1ENR_I2C1EN;    break;
  case TIM9_BASE:
    reg = &(RCC->APB2ENR);  bit = RCC_APB2ENR_TIM9EN;    break;
  case TIM10_BASE:
    reg = &(RCC->APB2ENR);  bit = RCC_APB2ENR_TIM10EN;   break;
  case TIM11_BASE:
    reg = &(RCC->APB2ENR);  bit = RCC_APB2ENR_TIM11EN;   break;


  default:
    printf("Can not enable peripheral at address %lx\n", (uint32_t)periph);
    /* No such peripheral */
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
  stm32l_switch_pclk(periph, 1);
}

void stm32_clk_arch_pclk_disable(void *periph)
{
  stm32l_switch_pclk(periph, 0);
}
