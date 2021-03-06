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

/* Generic STM32 peripherals */
#include "stm32-spi.h"
#include "stm32-i2c.h"
#include "stm32-usart.h"
#include "stm32-tim.h"

/* STM32L-specific */
#include "stm32l-gpio.h"
#include "stm32l-rcc.h"
#include "stm32l-flash.h"
#include "stm32l-dma.h"
#include "stm32l-pwr.h"
#include "stm32l-syscfg.h"
#include "stm32l-exti.h"


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
#define TIM2_BASE                       (APB1PERIPH_BASE + 0x0000)
#define TIM2                            ((STM32_TIM *)TIM2_BASE)

#define TIM3_BASE                       (APB1PERIPH_BASE + 0x0400)
#define TIM3                            ((STM32_TIM *)TIM3_BASE)

#define TIM4_BASE                       (APB1PERIPH_BASE + 0x0800)
#define TIM4                            ((STM32_TIM *)TIM4_BASE)

/* TIM5 is a 32-bit timer and doesn't exist in some STM32L models */
#define TIM5_BASE                       (APB1PERIPH_BASE + 0x0c00)
#define TIM5                            ((STM32_TIM *)TIM5_BASE)

#define TIM6_BASE                       (APB1PERIPH_BASE + 0x1000)
#define TIM6                            ((STM32_TIM *)TIM6_BASE)

#define TIM7_BASE                       (APB1PERIPH_BASE + 0x1400)
#define TIM7                            ((STM32_TIM *)TIM7_BASE)

#define SPI2_BASE                       (APB1PERIPH_BASE + 0x3800)
#define SPI2                            ((STM32_SPI *)SPI2_BASE)

/* Base address must be a typo in the datasheet */
#define SPI3_BASE                       (APB1PERIPH_BASE + 0x3900)
#define SPI3                            ((STM32_SPI *)SPI3_BASE)

#define USART2_BASE                     (APB1PERIPH_BASE + 0x4400)
#define USART2                          ((STM32_USART *)USART2_BASE)

#define USART3_BASE                     (APB1PERIPH_BASE + 0x4800)
#define USART3                          ((STM32_USART *)USART3_BASE)

#define I2C1_BASE                       (APB1PERIPH_BASE + 0x5400)
#define I2C1                            ((STM32_I2C *)I2C1_BASE)

#define I2C2_BASE                       (APB1PERIPH_BASE + 0x5400)
#define I2C2                            ((STM32_I2C *)I2C2_BASE)

/* APB2 */

#define SYSCFG_BASE                     (APB2PERIPH_BASE + 0x0000)
#define SYSCFG                          ((STM32L_SYSCFG *)SYSCFG_BASE)

#define EXTI_BASE                       (APB2PERIPH_BASE + 0x0400)
#define EXTI                            ((STM32L_EXTI *)EXTI_BASE)

#define TIM9_BASE                       (APB2PERIPH_BASE + 0x0800)
#define TIM9                            ((STM32_TIM *)TIM9_BASE)

#define TIM10_BASE                      (APB2PERIPH_BASE + 0x0C00)
#define TIM10                           ((STM32_TIM *)TIM10_BASE)

#define TIM11_BASE                      (APB2PERIPH_BASE + 0x1000)
#define TIM11                           ((STM32_TIM *)TIM11_BASE)

#define SPI1_BASE                       (APB2PERIPH_BASE + 0x3000)
#define SPI1                            ((STM32_SPI *)SPI1_BASE)

#define USART1_BASE                     (APB2PERIPH_BASE + 0x3800)
#define USART1                          ((STM32_USART *)USART1_BASE)



/* Convenience, from STM32F CPU port */
#define SET_BIT(REG, BIT)               ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)             ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)              ((REG) & (BIT))
#define CLEAR_REG(REG)                  ((REG) = 0x0)
#define WRITE_REG(REG, VAL)             ((REG) = VAL)
#define READ_REG(REG)                   ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
  WRITE_REG((REG), (((READ_REG(REG)) & (~CLEARMASK)) | (SETMASK)))

#endif
