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

#include <stdint.h>
#include "stm32l-gpio.h"


/*
Mode:
00: Input (reset state)
01: General purpose output mode
10: Alternate function mode
11: Analog mode
*/
inline void stm32l_gpio_set_mode(STM32L_GPIO *g, uint32_t pin, uint32_t mode) {
        /* Clear old mode */
        g->MODER &= ~(GPIO_MODE_MASK << (2 * pin));
        /* Set new mode */
        g->MODER |= mode << (2 * pin);
}

/*
Output type:
0: Output push-pull (reset state)
1: Output open-drain
*/
void stm32l_gpio_set_output_type(STM32L_GPIO *g, uint32_t pin, uint16_t type) {
        if (GPIO_OUTPUT_TYPE_PPULL == type) {
                g->OTYPER &= ~(1 << pin);
        } else {
                g->OTYPER |= (1 << pin);
        }
}

/*
Res:
00: No pull-up, pull-down
01: Pull-up
10: Pull-down
11: Reserved
*/
void stm32l_gpio_set_resistors(STM32L_GPIO *g, uint32_t pin, uint16_t res) {
        /* Clear old res */
        g->PUPDR &= ~(GPIO_RESISTORS_MASK << (2 * pin));
        /* Set new res */
        g->PUPDR |= (res << (2 * pin));
}
/*
Spd:
00: 400 kHz Very low speed
01: 2 MHz Low speed
10: 10 MHz Medium speed
11: 40 MHz High speed on 50 pF (50 MHz output max speed on 30 pF)
*/
void stm32l_gpio_set_output_speed(STM32L_GPIO *g, uint32_t pin, uint16_t spd) {
        /* Clear old mode */
        g->OSPEEDR &= ~(GPIO_OSPEED_MASK << (2 * pin));
        /* Set new mode */
        g->OSPEEDR |= (spd << (2 * pin));
}

void stm32l_gpio_conf_output(STM32L_GPIO *port, uint32_t pin, uint16_t mode, uint16_t spd) {
       stm32l_gpio_set_mode(port, pin, GPIO_MODE_OUTPUT);
       stm32l_gpio_set_output_type(port, pin, mode);
       stm32l_gpio_set_output_speed(port, pin, spd);
}

void stm32l_gpio_conf_input(STM32L_GPIO *port, uint32_t pin, uint16_t res) {
       stm32l_gpio_set_mode(port, pin, GPIO_MODE_INPUT);
       stm32l_gpio_set_resistors(port, pin, res);
}

void stm32l_gpio_conf_af(STM32L_GPIO *port, uint32_t pin, uint16_t type, 
                  uint16_t spd, uint16_t res) {
       stm32l_gpio_set_mode(port, pin, GPIO_MODE_AF);
       stm32l_gpio_set_output_type(port, pin, type);
       stm32l_gpio_set_output_speed(port, pin, spd);
       stm32l_gpio_set_resistors(port, pin, res);
}

/*
  af is a four-bit mask that defines a function for a specific pin. 

  Each GPIO port has two registers, AFRL and AFRH (AFR[0] and AFR[1]),
  where this mask can be set for each GPIO pin in the port. GPIO pins
  0-7 are mapped in AFRL/AFR[0] and pins 8-15 are mapped in
  AFRH/AFR[1].

  Example:
  We want PB10 as USART3 TX.
  port: B, pin: 10 -> want to modify GPIOB->AFR[1].
  We use the afSTM32L_GPIO_AF_USART3 which is 0x7 (b0111)
  We shift...
  
*/

void stm32l_gpio_map_af(STM32L_GPIO *port, uint32_t pin, uint32_t af) {
        uint32_t afreg;
        uint32_t shift;

        /* AFRL or AFRH? */
        afreg = pin >> 0x03;
        /* Bits to shift in either AFRL or AFRH (& 0x07) */
        shift = (pin & 0x07) << 2;

        /* Clear previous mapping for pin */
        port->AFR[afreg] &= ~(((uint32_t)0xf) << shift);
        /* Set new mapping */
        port->AFR[afreg] |= (((uint32_t)(af)) << shift);
}

void stm32l_gpio_output_set(STM32L_GPIO *port, uint16_t pin) {
        /* Low part of BSRR is BS (bit set) and thus sets bit in ODR */
        port->BSRRL = (1 << pin);
}

void stm32l_gpio_output_clear(STM32L_GPIO *port, uint16_t pin) {
        /* High part of BSRR is BR (bit reset) and thus clears bit in ODR */
        port->BSRRH = (1 << pin);
}

int stm32l_gpio_output_get(STM32L_GPIO *port, uint16_t pin) {
        return port->ODR & (1 << pin);
}

void stm32l_gpio_conf_low_power(STM32L_GPIO *port, uint16_t pin) {

        /* Clear old mode (SPD = 0 -> 400 kHz, which is what we want) */
        //g->OSPEEDR &= ~(GPIO_OSPEED_MASK << (2 * pin));
        /* Clear old mode */
        port->MODER &= ~(GPIO_MODE_MASK << (2 * pin));
        /* Configure as analog input */
        port->MODER |= GPIO_MODE_ANALOG << (2 * pin);
        /* Output type push-pull */
        //g->OTYPER &= ~(1 << pin);
        /* No pull-up or pull-down resistors */
        port->PUPDR &= ~(GPIO_RESISTORS_MASK << (2 * pin));
}
