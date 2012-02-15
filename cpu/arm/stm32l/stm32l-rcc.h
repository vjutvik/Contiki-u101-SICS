/*
 * Copyright (c) 2011, UPWIS AB, Erik Jansson (erik at upwis.com)
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

#ifndef STM32L_RCC_H
#define STM32L_RCC_H

#include <stm32l.h>

typedef struct {
  volatile uint32_t CR;
  volatile uint32_t ICSCR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHBRSTR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t AHBLPENR;
  volatile uint32_t APB2LPENR;
  volatile uint32_t APB1LPENR;
  volatile uint32_t CSR;
} STM32L_RCC;

#define RCC_BASE                        (AHBPERIPH_BASE + 0x3800)
#define RCC                             ((STM32L_RCC *)RCC_BASE)

#define RCC_CR_HSION                    ((uint32_t)0x00000001)
#define RCC_CR_HSIRDY                   ((uint32_t)0x00000002)
#define RCC_CR_MSION                    ((uint32_t)0x00000100)
#define RCC_CR_MSIRDY                   ((uint32_t)0x00000200)
#define RCC_CR_HSEON                    ((uint32_t)0x00010000)
#define RCC_CR_HSERDY                   ((uint32_t)0x00020000)
#define RCC_CR_HSEBYP                   ((uint32_t)0x00040000)
#define RCC_CR_PLLON                    ((uint32_t)0x01000000)
#define RCC_CR_PLLRDY                   ((uint32_t)0x02000000)
#define RCC_CR_CSSON                    ((uint32_t)0x10000000)
#define RCC_CR_RTCPRE                   ((uint32_t)0x60000000)
#define RCC_CR_RTCPRE_0                 ((uint32_t)0x20000000)
#define RCC_CR_RTCPRE_1                 ((uint32_t)0x40000000)

#define RCC_ICSCR                       ((uint32_t)0x0000e000)
#define RCC_ICSCR_MSIRANGE_65KHZ        ((uint32_t)0x00000000)
#define RCC_ICSCR_MSIRANGE_131KHZ       ((uint32_t)0x00002000)
#define RCC_ICSCR_MSIRANGE_252KHZ       ((uint32_t)0x00004000)
#define RCC_ICSCR_MSIRANGE_524KHZ       ((uint32_t)0x00006000)
#define RCC_ICSCR_MSIRANGE_1048KHZ      ((uint32_t)0x00008000)
#define RCC_ICSCR_MSIRANGE_2097KHZ      ((uint32_t)0x0000c000)
#define RCC_ICSCR_MSIRANGE_4194KHZ      ((uint32_t)0x0000e000)

#define RCC_CFGR_SW                     ((uint32_t)0x00000003)

#define RCC_CFGR_SW_MSI                 ((uint32_t)0x00000000)
#define RCC_CFGR_SW_HSI                 ((uint32_t)0x00000001)
#define RCC_CFGR_SW_HSE                 ((uint32_t)0x00000002)
#define RCC_CFGR_SW_PLL                 ((uint32_t)0x00000003)

#define RCC_CFGR_SWS                    ((uint32_t)0x0000000c)

#define RCC_CFGR_SWS_MSI                ((uint32_t)0x00000000)
#define RCC_CFGR_SWS_HSI                ((uint32_t)0x00000004)
#define RCC_CFGR_SWS_HSE                ((uint32_t)0x00000008)
#define RCC_CFGR_SWS_PLL                ((uint32_t)0x0000000c)

#define RCC_CFGR_HPRE                   ((uint32_t)0x000000f0)

#define RCC_CFGR_HPRE_DIV1              ((uint32_t)0x00000000)
#define RCC_CFGR_HPRE_DIV2              ((uint32_t)0x00000080)
#define RCC_CFGR_HPRE_DIV4              ((uint32_t)0x00000090)
#define RCC_CFGR_HPRE_DIV8              ((uint32_t)0x000000a0)
#define RCC_CFGR_HPRE_DIV16             ((uint32_t)0x000000b0)
#define RCC_CFGR_HPRE_DIV64             ((uint32_t)0x000000c0)
#define RCC_CFGR_HPRE_DIV128            ((uint32_t)0x000000d0)
#define RCC_CFGR_HPRE_DIV256            ((uint32_t)0x000000e0)
#define RCC_CFGR_HPRE_DIV512            ((uint32_t)0x000000f0)

#define RCC_CFGR_PPRE1                  ((uint32_t)0x00000700)

#define RCC_CFGR_PPRE1_DIV1             ((uint32_t)0x00000000)
#define RCC_CFGR_PPRE1_DIV2             ((uint32_t)0x00000400)
#define RCC_CFGR_PPRE1_DIV4             ((uint32_t)0x00000500)
#define RCC_CFGR_PPRE1_DIV8             ((uint32_t)0x00000600)
#define RCC_CFGR_PPRE1_DIV16            ((uint32_t)0x00000700)

#define RCC_CFGR_PPRE2                  ((uint32_t)0x00003800)
#define RCC_CFGR_PPRE2_DIV1             ((uint32_t)0x00000000)
#define RCC_CFGR_PPRE2_DIV2             ((uint32_t)0x00002000)
#define RCC_CFGR_PPRE2_DIV4             ((uint32_t)0x00002800)
#define RCC_CFGR_PPRE2_DIV8             ((uint32_t)0x00003000)
#define RCC_CFGR_PPRE2_DIV16            ((uint32_t)0x00003800)

#define RCC_CFGR_PLLSRC                 ((uint32_t)0x00010000)
#define RCC_CFGR_PLLSRC_HSI             ((uint32_t)0x00000000)
#define RCC_CFGR_PLLSRC_HSE             ((uint32_t)0x00010000)
#define RCC_CFGR_PLLMUL                 ((uint32_t)0x003c0000)
#define RCC_CFGR_PLLMUL3                ((uint32_t)0x00000000)
#define RCC_CFGR_PLLMUL4                ((uint32_t)0x00040000)
#define RCC_CFGR_PLLMUL6                ((uint32_t)0x00080000)
#define RCC_CFGR_PLLMUL8                ((uint32_t)0x000C0000)
#define RCC_CFGR_PLLMUL12               ((uint32_t)0x00100000)
#define RCC_CFGR_PLLMUL16               ((uint32_t)0x00140000)
#define RCC_CFGR_PLLMUL24               ((uint32_t)0x00180000)
#define RCC_CFGR_PLLMUL32               ((uint32_t)0x001c0000)
#define RCC_CFGR_PLLMUL48               ((uint32_t)0x00200000)
#define RCC_CFGR_PLLDIV                 ((uint32_t)0x00c00000)
#define RCC_CFGR_PLLDIV1                ((uint32_t)0x00000000)
#define RCC_CFGR_PLLDIV2                ((uint32_t)0x00400000)
#define RCC_CFGR_PLLDIV3                ((uint32_t)0x00800000)
#define RCC_CFGR_PLLDIV4                ((uint32_t)0x00c00000)

#define RCC_CFGR_MCOSEL                 ((uint32_t)0x07000000)

#define RCC_CFGR_MCO_NOCLOCK            ((uint32_t)0x00000000)
#define RCC_CFGR_MCO_SYSCLK             ((uint32_t)0x01000000)
#define RCC_CFGR_MCO_HSI                ((uint32_t)0x02000000)
#define RCC_CFGR_MCO_MSI                ((uint32_t)0x03000000)
#define RCC_CFGR_MCO_HSE                ((uint32_t)0x04000000)
#define RCC_CFGR_MCO_PLL                ((uint32_t)0x05000000)
#define RCC_CFGR_MCO_LSI                ((uint32_t)0x06000000)
#define RCC_CFGR_MCO_LSE                ((uint32_t)0x07000000)

#define RCC_CFGR_MCOPRE                 ((uint32_t)0x70000000)

#define RCC_CFGR_MCO_DIV1               ((uint32_t)0x00000000)
#define RCC_CFGR_MCO_DIV2               ((uint32_t)0x10000000)
#define RCC_CFGR_MCO_DIV4               ((uint32_t)0x20000000)
#define RCC_CFGR_MCO_DIV8               ((uint32_t)0x30000000)
#define RCC_CFGR_MCO_DIV16              ((uint32_t)0x40000000)

#define RCC_CIR_LSIRDYF                 ((uint32_t)0x00000001)
#define RCC_CIR_LSERDYF                 ((uint32_t)0x00000002)
#define RCC_CIR_HSIRDYF                 ((uint32_t)0x00000004)
#define RCC_CIR_HSERDYF                 ((uint32_t)0x00000008)
#define RCC_CIR_PLLRDYF                 ((uint32_t)0x00000010)
#define RCC_CIR_MSIRDYF                 ((uint32_t)0x00000020)
#define RCC_CIR_CSSF                    ((uint32_t)0x00000080)

#define RCC_CIR_LSIRDYIE                ((uint32_t)0x00000100)
#define RCC_CIR_LSERDYIE                ((uint32_t)0x00000200)
#define RCC_CIR_HSIRDYIE                ((uint32_t)0x00000400)
#define RCC_CIR_HSERDYIE                ((uint32_t)0x00000800)
#define RCC_CIR_PLLRDYIE                ((uint32_t)0x00001000)
#define RCC_CIR_MSIRDYIE                ((uint32_t)0x00002000)

#define RCC_CIR_LSIRDYC                 ((uint32_t)0x00010000)
#define RCC_CIR_LSERDYC                 ((uint32_t)0x00020000)
#define RCC_CIR_HSIRDYC                 ((uint32_t)0x00040000)
#define RCC_CIR_HSERDYC                 ((uint32_t)0x00080000)
#define RCC_CIR_PLLRDYC                 ((uint32_t)0x00100000)
#define RCC_CIR_MSIRDYC                 ((uint32_t)0x00200000)
#define RCC_CIR_CSSC                    ((uint32_t)0x00800000)

#define RCC_AHBRSTR_GPIOARST            ((uint32_t)0x00000001)
#define RCC_AHBRSTR_GPIOBRST            ((uint32_t)0x00000002)
#define RCC_AHBRSTR_GPIOCRST            ((uint32_t)0x00000004)
#define RCC_AHBRSTR_GPIODRST            ((uint32_t)0x00000008)
#define RCC_AHBRSTR_GPIOERST            ((uint32_t)0x00000010)
#define RCC_AHBRSTR_GPIOHRST            ((uint32_t)0x00000020)
#define RCC_AHBRSTR_CRCRST              ((uint32_t)0x00001000)
#define RCC_AHBRSTR_FLITFRST            ((uint32_t)0x00008000)
#define RCC_AHBRSTR_DMA1RST             ((uint32_t)0x01000000)

#define RCC_APB2RSTR_SYSCFGRST          ((uint32_t)0x00000001)
#define RCC_APB2RSTR_TIM9RST            ((uint32_t)0x00000004)
#define RCC_APB2RSTR_TIM10RST           ((uint32_t)0x00000008)
#define RCC_APB2RSTR_TIM11RST           ((uint32_t)0x00000010)
#define RCC_APB2RSTR_ADC1RST            ((uint32_t)0x00000200)
#define RCC_APB2RSTR_SPI1RST            ((uint32_t)0x00001000)
#define RCC_APB2RSTR_USART1RST          ((uint32_t)0x00004000)

#define RCC_APB1RSTR_TIM2RST            ((uint32_t)0x00000001)
#define RCC_APB1RSTR_TIM3RST            ((uint32_t)0x00000002)
#define RCC_APB1RSTR_TIM4RST            ((uint32_t)0x00000004)
#define RCC_APB1RSTR_TIM6RST            ((uint32_t)0x00000010)
#define RCC_APB1RSTR_TIM7RST            ((uint32_t)0x00000020)
#define RCC_APB1RSTR_LCDRST             ((uint32_t)0x00000200)
#define RCC_APB1RSTR_WWDGRST            ((uint32_t)0x00000800)
#define RCC_APB1RSTR_SPI2RST            ((uint32_t)0x00004000)
#define RCC_APB1RSTR_USART2RST          ((uint32_t)0x00020000)
#define RCC_APB1RSTR_USART3RST          ((uint32_t)0x00040000)
#define RCC_APB1RSTR_I2C1RST            ((uint32_t)0x00200000)
#define RCC_APB1RSTR_I2C2RST            ((uint32_t)0x00400000)
#define RCC_APB1RSTR_USBRST             ((uint32_t)0x00800000)
#define RCC_APB1RSTR_PWRRST             ((uint32_t)0x10000000)
#define RCC_APB1RSTR_DACRST             ((uint32_t)0x20000000)
#define RCC_APB1RSTR_COMPRST            ((uint32_t)0x80000000)

#define RCC_AHBENR_GPIOAEN              ((uint32_t)0x00000001)
#define RCC_AHBENR_GPIOBEN              ((uint32_t)0x00000002)
#define RCC_AHBENR_GPIOCEN              ((uint32_t)0x00000004)
#define RCC_AHBENR_GPIODEN              ((uint32_t)0x00000008)
#define RCC_AHBENR_GPIOEEN              ((uint32_t)0x00000010)
#define RCC_AHBENR_GPIOHEN              ((uint32_t)0x00000020)
#define RCC_AHBENR_CRCEN                ((uint32_t)0x00001000)
#define RCC_AHBENR_FLITFEN              ((uint32_t)0x00008000)

#define RCC_AHBENR_DMA1EN               ((uint32_t)0x01000000)

#define RCC_APB2ENR_SYSCFGEN            ((uint32_t)0x00000001)
#define RCC_APB2ENR_TIM9EN              ((uint32_t)0x00000004)
#define RCC_APB2ENR_TIM10EN             ((uint32_t)0x00000008)
#define RCC_APB2ENR_TIM11EN             ((uint32_t)0x00000010)
#define RCC_APB2ENR_ADC1EN              ((uint32_t)0x00000200)
#define RCC_APB2ENR_SPI1EN              ((uint32_t)0x00001000)
#define RCC_APB2ENR_USART1EN            ((uint32_t)0x00004000)

#define RCC_APB1ENR_TIM2EN              ((uint32_t)0x00000001)
#define RCC_APB1ENR_TIM3EN              ((uint32_t)0x00000002)
#define RCC_APB1ENR_TIM4EN              ((uint32_t)0x00000004)
#define RCC_APB1ENR_TIM6EN              ((uint32_t)0x00000010)
#define RCC_APB1ENR_TIM7EN              ((uint32_t)0x00000020)
#define RCC_APB1ENR_LCDEN               ((uint32_t)0x00000200)
#define RCC_APB1ENR_WWDGEN              ((uint32_t)0x00000800)
#define RCC_APB1ENR_SPI2EN              ((uint32_t)0x00004000)
#define RCC_APB1ENR_USART2EN            ((uint32_t)0x00020000)
#define RCC_APB1ENR_USART3EN            ((uint32_t)0x00040000)
#define RCC_APB1ENR_I2C1EN              ((uint32_t)0x00200000)
#define RCC_APB1ENR_I2C2EN              ((uint32_t)0x00400000)
#define RCC_APB1ENR_USBEN               ((uint32_t)0x00800000)
#define RCC_APB1ENR_PWREN               ((uint32_t)0x10000000)
#define RCC_APB1ENR_DACEN               ((uint32_t)0x20000000)
#define RCC_APB1ENR_COMPEN              ((uint32_t)0x80000000)

#define RCC_AHBLPENR_GPIOALPEN          ((uint32_t)0x00000001)
#define RCC_AHBLPENR_GPIOBLPEN          ((uint32_t)0x00000002)
#define RCC_AHBLPENR_GPIOCLPEN          ((uint32_t)0x00000004)
#define RCC_AHBLPENR_GPIODLPEN          ((uint32_t)0x00000008)
#define RCC_AHBLPENR_GPIOELPEN          ((uint32_t)0x00000010)
#define RCC_AHBLPENR_GPIOHLPEN          ((uint32_t)0x00000020)
#define RCC_AHBLPENR_CRCLPEN            ((uint32_t)0x00001000)
#define RCC_AHBLPENR_FLITFLPEN          ((uint32_t)0x00008000)

#define RCC_AHBLPENR_SRAMLPEN           ((uint32_t)0x00010000)
#define RCC_AHBLPENR_DMA1LPEN           ((uint32_t)0x01000000)

#define RCC_APB2LPENR_SYSCFGLPEN        ((uint32_t)0x00000001)
#define RCC_APB2LPENR_TIM9LPEN          ((uint32_t)0x00000004)
#define RCC_APB2LPENR_TIM10LPEN         ((uint32_t)0x00000008)
#define RCC_APB2LPENR_TIM11LPEN         ((uint32_t)0x00000010)
#define RCC_APB2LPENR_ADC1LPEN          ((uint32_t)0x00000200)
#define RCC_APB2LPENR_SPI1LPEN          ((uint32_t)0x00001000)
#define RCC_APB2LPENR_USART1LPEN        ((uint32_t)0x00004000)

#define RCC_APB1LPENR_TIM2LPEN          ((uint32_t)0x00000001)
#define RCC_APB1LPENR_TIM3LPEN          ((uint32_t)0x00000002)
#define RCC_APB1LPENR_TIM4LPEN          ((uint32_t)0x00000004)
#define RCC_APB1LPENR_TIM6LPEN          ((uint32_t)0x00000010)
#define RCC_APB1LPENR_TIM7LPEN          ((uint32_t)0x00000020)
#define RCC_APB1LPENR_LCDLPEN           ((uint32_t)0x00000200)
#define RCC_APB1LPENR_WWDGLPEN          ((uint32_t)0x00000800)
#define RCC_APB1LPENR_SPI2LPEN          ((uint32_t)0x00004000)
#define RCC_APB1LPENR_USART2LPEN        ((uint32_t)0x00020000)
#define RCC_APB1LPENR_USART3LPEN        ((uint32_t)0x00040000)
#define RCC_APB1LPENR_I2C1LPEN          ((uint32_t)0x00200000)
#define RCC_APB1LPENR_I2C2LPEN          ((uint32_t)0x00400000)
#define RCC_APB1LPENR_USBLPEN           ((uint32_t)0x00800000)
#define RCC_APB1LPENR_PWRLPEN           ((uint32_t)0x10000000)
#define RCC_APB1LPENR_DACLPEN           ((uint32_t)0x20000000)
#define RCC_APB1LPENR_COMPLPEN          ((uint32_t)0x80000000)

#define RCC_CSR_LSION                   ((uint32_t)0x00000001)
#define RCC_CSR_LSIRDY                  ((uint32_t)0x00000002)

#define RCC_CSR_LSEON                   ((uint32_t)0x00000100)
#define RCC_CSR_LSERDY                  ((uint32_t)0x00000200)
#define RCC_CSR_LSEBYP                  ((uint32_t)0x00000400)


uint32_t stm32l_clocks_hclk(void);
uint32_t stm32l_clocks_pclk1(void);
uint32_t stm32l_clocks_pclk2(void);

#endif
