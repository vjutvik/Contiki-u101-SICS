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

#ifndef STM32_USART_H
#define STM32_USART_H

#include <stdint.h>

typedef struct {
  volatile uint16_t SR;
  const uint16_t R0;
  volatile uint16_t DR;
  const uint16_t R1;
  volatile uint16_t BRR;
  const uint16_t R2;
  volatile uint16_t CR1;
  const uint16_t R3;
  volatile uint16_t CR2;
  const uint16_t R4;
  volatile uint16_t CR3;
  const uint16_t R5;
  volatile uint16_t GTPR;
  const uint16_t R6;
} STM32L_USART;

#define USART_SR_PE                     ((uint16_t)0x0001)
#define USART_SR_FE                     ((uint16_t)0x0002)
#define USART_SR_NE                     ((uint16_t)0x0004)
#define USART_SR_ORE                    ((uint16_t)0x0008)
#define USART_SR_IDLE                   ((uint16_t)0x0010)
#define USART_SR_RXNE                   ((uint16_t)0x0020)
#define USART_SR_TC                     ((uint16_t)0x0040)
#define USART_SR_TXE                    ((uint16_t)0x0080)
#define USART_SR_LBD                    ((uint16_t)0x0100)
#define USART_SR_CTS                    ((uint16_t)0x0200)

#define USART_CR1_SBK                   ((uint16_t)0x0001)
#define USART_CR1_RWU                   ((uint16_t)0x0002)
#define USART_CR1_RE                    ((uint16_t)0x0004)
#define USART_CR1_TE                    ((uint16_t)0x0008)
#define USART_CR1_IDLEIE                ((uint16_t)0x0010)
#define USART_CR1_RXNEIE                ((uint16_t)0x0020)
#define USART_CR1_TCIE                  ((uint16_t)0x0040)
#define USART_CR1_TXEIE                 ((uint16_t)0x0080)
#define USART_CR1_PEIE                  ((uint16_t)0x0100)
#define USART_CR1_PS                    ((uint16_t)0x0200)
#define USART_CR1_PCE                   ((uint16_t)0x0400)
#define USART_CR1_WAKE                  ((uint16_t)0x0800)
#define USART_CR1_M                     ((uint16_t)0x1000)
#define USART_CR1_UE                    ((uint16_t)0x2000)
#define USART_CR1_OVER8                 ((uint16_t)0x8000)

#define USART_CR2_ADD                   ((uint16_t)0x000f)
#define USART_CR2_LBDL                  ((uint16_t)0x0020)
#define USART_CR2_LBDIE                 ((uint16_t)0x0040)
#define USART_CR2_LBCL                  ((uint16_t)0x0100)
#define USART_CR2_CPHA                  ((uint16_t)0x0200)
#define USART_CR2_CPOL                  ((uint16_t)0x0400)
#define USART_CR2_CLKEN                 ((uint16_t)0x0800)

#define USART_CR2_STOP                  ((uint16_t)0x3000)
#define USART_CR2_STOP_0                ((uint16_t)0x1000)
#define USART_CR2_STOP_1                ((uint16_t)0x2000)

#define USART_CR2_LINEN                 ((uint16_t)0x4000)

#define USART_CR3_EIE                   ((uint16_t)0x0001)
#define USART_CR3_IREN                  ((uint16_t)0x0002)
#define USART_CR3_IRLP                  ((uint16_t)0x0004)
#define USART_CR3_HDSEL                 ((uint16_t)0x0008)
#define USART_CR3_NACK                  ((uint16_t)0x0010)
#define USART_CR3_SCEN                  ((uint16_t)0x0020)
#define USART_CR3_DMAR                  ((uint16_t)0x0040)
#define USART_CR3_DMAT                  ((uint16_t)0x0080)
#define USART_CR3_RTSE                  ((uint16_t)0x0100)
#define USART_CR3_CTSE                  ((uint16_t)0x0200)
#define USART_CR3_CTSIE                 ((uint16_t)0x0400)
#define USART_CR3_ONEBIT                ((uint16_t)0x0800)

#define USART_GTPR_PSC                  ((uint16_t)0x00ff)
#define USART_GTPR_PSC_0                ((uint16_t)0x0001)
#define USART_GTPR_PSC_1                ((uint16_t)0x0002)
#define USART_GTPR_PSC_2                ((uint16_t)0x0004)
#define USART_GTPR_PSC_3                ((uint16_t)0x0008)
#define USART_GTPR_PSC_4                ((uint16_t)0x0010)
#define USART_GTPR_PSC_5                ((uint16_t)0x0020)
#define USART_GTPR_PSC_6                ((uint16_t)0x0040)
#define USART_GTPR_PSC_7                ((uint16_t)0x0080)

#define USART_GTPR_GT                   ((uint16_t)0xff00)

#endif
