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

#ifndef STM32_SYSTICK_H
#define STM32_SYSTICK_H

#include <stdint.h>
#include "stm32-scb.h"

typedef struct {
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  const volatile uint32_t CALIB;
} STM32_SYSTICK;

#define SYSTICK_BASE                    (SCS_BASE + 0x0010)
#define SysTick                         ((STM32_SYSTICK *)SYSTICK_BASE)

#define SysTick_CTRL_ENABLE             ((uint32_t)0x00000001)
#define SysTick_CTRL_TICKINT            ((uint32_t)0x00000002)
#define SysTick_CTRL_CLKSOURCE          ((uint32_t)0x00000004)
#define SysTick_CTRL_COUNTFLAG          ((uint32_t)0x00010000)

#define SysTick_LOAD_RELOAD             ((uint32_t)0x00ffffff)

#define SysTick_VAL_CURRENT             ((uint32_t)0x00ffffff)

#define SysTick_CALIB_TENMS             ((uint32_t)0x00ffffff)
#define SysTick_CALIB_SKEW              ((uint32_t)0x40000000)
#define SysTick_CALIB_NOREF             ((uint32_t)0x80000000)

#endif
