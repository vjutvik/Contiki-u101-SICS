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

#ifndef STM32L_H
#define STM32L_H

#include <stdint.h>
#include "stm32l-gpio.h"
#include "stm32l-rcc.h"
#include "stm32l-flash.h"
#include "stm32l-dma.h"
#include "stm32l-pwr.h"
#include "stm32l-syscfg.h"
#include "stm32l-usart.h"

#define FLASH_BASE                      ((uint32_t)0x08000000)
#define SRAM_BASE                       ((uint32_t)0x20000000)

#define PERIPHERAL_BASE                 ((uint32_t)0x40000000)
#define APB1PERIPH_BASE                 (PERIPHERAL_BASE)
#define APB2PERIPH_BASE                 (PERIPHERAL_BASE + 0x10000)
#define AHBPERIPH_BASE                  (PERIPHERAL_BASE + 0x20000)

/* AHB */

#define GPIOA_BASE                      (AHBPERIPH_BASE + 0x0000)
#define GPIOA                           ((STM32L_GPIO *)GPIOA_BASE)

#define GPIOB_BASE                      (AHBPERIPH_BASE + 0x0400)
#define GPIOB                           ((STM32L_GPIO *)GPIOB_BASE)

#define GPIOC_BASE                      (AHBPERIPH_BASE + 0x0800)
#define GPIOC                           ((STM32L_GPIO *)GPIOC_BASE)

#define GPIOD_BASE                      (AHBPERIPH_BASE + 0x0C00)
#define GPIOD                           ((STM32L_GPIO *)GPIOD_BASE)

#define GPIOE_BASE                      (AHBPERIPH_BASE + 0x1000)
#define GPIOE                           ((STM32L_GPIO *)GPIOE_BASE)

#define GPIOH_BASE                      (AHBPERIPH_BASE + 0x1400)
#define GPIOH                           ((STM32L_GPIO *)GPIOH_BASE)

#define RCC_BASE                        (AHBPERIPH_BASE + 0x3800)
#define RCC                             ((STM32L_RCC *)RCC_BASE)

#define FLASH_R_BASE                    (AHBPERIPH_BASE + 0x3c00)
#define FLASH                           ((STM32L_FLASH *)FLASH_R_BASE)

#define DMA1_BASE                       (AHBPERIPH_BASE + 0x6000)
#define DMA1                            ((STM32L_DMA *)DMA1_BASE)

#define DMA1_CHANNEL1_BASE              (DMA1_BASE + 0x0008)
#define DMA1_CHANNEL1                   ((STM32L_DMA_CHN *)DMA1_CHANNEL1_BASE)

#define DMA1_CHANNEL2_BASE              (DMA1_BASE + 0x001C)
#define DMA1_CHANNEL2                   ((STM32L_DMA_CHN *)DMA1_CHANNEL2_BASE)

#define DMA1_CHANNEL3_BASE              (DMA1_BASE + 0x0030)
#define DMA1_CHANNEL3                   ((STM32L_DMA_CHN *)DMA1_CHANNEL3_BASE)

#define DMA1_CHANNEL4_BASE              (DMA1_BASE + 0x0044)
#define DMA1_CHANNEL4                   ((STM32L_DMA_CHN *)DMA1_CHANNEL4_BASE)

#define DMA1_CHANNEL5_BASE              (DMA1_BASE + 0x0058)
#define DMA1_CHANNEL5                   ((STM32L_DMA_CHN *)DMA1_CHANNEL5_BASE)

#define DMA1_CHANNEL6_BASE              (DMA1_BASE + 0x006C)
#define DMA1_CHANNEL6                   ((STM32L_DMA_CHN *)DMA1_CHANNEL6_BASE)

#define DMA1_CHANNEL7_BASE              (DMA1_BASE + 0x0080)
#define DMA1_CHANNEL7                   ((STM32L_DMA_CHN *)DMA1_CHANNEL7_BASE)

#define PWR_BASE                        (APB1PERIPH_BASE + 0x7000)
#define PWR                             ((STM32L_PWR *)PWR_BASE)

/* APB1 */

#define USART2_BASE                     (APB1PERIPH_BASE + 0x4400)
#define USART2                          ((STM32L_USART *)USART2_BASE)

#define USART3_BASE                     (APB1PERIPH_BASE + 0x4800)
#define USART3                          ((STM32L_USART *)USART3_BASE)

/* APB2 */

#define SYSCFG_BASE                     (APB2PERIPH_BASE + 0x0000)
#define SYSCFG                          ((STM32L_SYSCFG *)SYSCFG_BASE)

#define USART1_BASE                     (APB2PERIPH_BASE + 0x3800)
#define USART1                          ((STM32L_USART *)USART1_BASE)

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = 0x0)

#define WRITE_REG(REG, VAL)   ((REG) = VAL)

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~CLEARMASK)) | (SETMASK)))

#endif
