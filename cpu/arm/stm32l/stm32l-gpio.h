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

#ifndef STM32L_GPIO_H
#define STM32L_GPIO_H

#include "stm32l.h"

/* Do we want something like this?
#define GPIO_HIGH(a,b)          a->BSRRL = b
#define GPIO_LOW(a,b)           a->BSRRH = b
#define GPIO_TOGGLE(a,b)        a->ODR ^= b 
*/
typedef struct {
  volatile uint32_t MODER;
  volatile uint16_t OTYPER;
  const uint16_t R0;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint16_t IDR;
  const uint16_t R1;
  volatile uint16_t ODR;
  const uint16_t R2;
  volatile uint16_t BSRRL;
  volatile uint16_t BSRRH;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} STM32L_GPIO;


#define GPIO_MODE_MASK                  ((uint32_t)0x3)
#define GPIO_MODE_INPUT                 ((uint32_t)0x0)
#define GPIO_MODE_OUTPUT                ((uint32_t)0x1)
#define GPIO_MODE_AF                    ((uint32_t)0x2)
#define GPIO_MODE_ANALOG                ((uint32_t)0x3)

#define GPIO_OUTPUT_TYPE_PPULL          ((uint32_t)0x0)
#define GPIO_OUTPUT_TYPE_ODRAIN         ((uint32_t)0x1)

#define GPIO_RESISTORS_MASK             ((uint32_t)0x3)
#define GPIO_RESISTORS_NONE             ((uint32_t)0x0)
#define GPIO_RESISTORS_PULLUP           ((uint32_t)0x1)
#define GPIO_RESISTORS_PULLDN           ((uint32_t)0x2)
#define GPIO_RESISTORS_RESERVED         ((uint32_t)0x3)

#define GPIO_OSPEED_400KHZ              ((uint32_t)0x0)
#define GPIO_OSPEED_2MHZ                ((uint32_t)0x1)
#define GPIO_OSPEED_10MHZ               ((uint32_t)0x2)
#define GPIO_OSPEED_40MHZ               ((uint32_t)0x3)
#define GPIO_OSPEED_MASK                ((uint32_t)0x3)

#define GPIO_AF_RTC_50Hz                ((uint8_t)0x0)
#define GPIO_AF_MCO                     ((uint8_t)0x0)
#define GPIO_AF_RTC_AF1                 ((uint8_t)0x0)
#define GPIO_AF_WKUP                    ((uint8_t)0x0)
#define GPIO_AF_SWJ                     ((uint8_t)0x0)
#define GPIO_AF_TRACE                   ((uint8_t)0x0)
#define GPIO_AF_TIM2                    ((uint8_t)0x1)
#define GPIO_AF_TIM3                    ((uint8_t)0x2)
#define GPIO_AF_TIM4                    ((uint8_t)0x2)
#define GPIO_AF_TIM9                    ((uint8_t)0x3)
#define GPIO_AF_TIM10                   ((uint8_t)0x3)
#define GPIO_AF_TIM11                   ((uint8_t)0x3)
#define GPIO_AF_I2C1                    ((uint8_t)0x4)
#define GPIO_AF_I2C2                    ((uint8_t)0x4)
#define GPIO_AF_SPI1                    ((uint8_t)0x5)
#define GPIO_AF_SPI2                    ((uint8_t)0x5)
#define GPIO_AF_USART1                  ((uint8_t)0x7)
#define GPIO_AF_USART2                  ((uint8_t)0x7)
#define GPIO_AF_USART3                  ((uint8_t)0x7)
#define GPIO_AF_USB                     ((uint8_t)0xa)
#define GPIO_AF_LCD                     ((uint8_t)0xb)
#define GPIO_AF_RI                      ((uint8_t)0xe)
#define GPIO_AF_EVENTOUT                ((uint8_t)0xf)

void stm32l_gpio_set_mode(STM32L_GPIO *g, uint32_t pin, uint32_t mode);
void stm32l_gpio_set_output_type(STM32L_GPIO *g, uint32_t pin, uint16_t type);
void stm32l_gpio_set_resistors(STM32L_GPIO *g, uint32_t pin, uint16_t res);
void stm32l_gpio_set_output_speed(STM32L_GPIO *g, uint32_t pin, uint16_t spd);
void stm32l_gpio_conf_output(STM32L_GPIO *port, uint32_t pin, uint16_t mode, uint16_t spd);
void stm32l_gpio_conf_af(STM32L_GPIO *port, uint32_t pin, uint16_t mode, uint16_t spd, uint16_t res);
void stm32l_gpio_map_af(STM32L_GPIO *port, uint32_t pin, uint32_t af);
void stm32l_gpio_output_set(STM32L_GPIO *port, uint16_t pin);
void stm32l_gpio_output_clear(STM32L_GPIO *port, uint16_t pin);
int stm32l_gpio_output_get(STM32L_GPIO *port, uint16_t pin);
void stm32l_gpio_conf_input(STM32L_GPIO *port, uint32_t pin, uint16_t res);
void stm32l_gpio_conf_low_power(STM32L_GPIO *port, uint16_t pin);

#endif
