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

#ifndef STM32L_PWR_H
#define STM32L_PWR_H

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} STM32L_PWR;

#define PWR_CR_LPSDSR                   ((uint16_t)0x0001)
#define PWR_CR_PDDS                     ((uint16_t)0x0002)
#define PWR_CR_CWUF                     ((uint16_t)0x0004)
#define PWR_CR_CSBF                     ((uint16_t)0x0008)
#define PWR_CR_PVDE                     ((uint16_t)0x0010)

#define PWR_CR_PLS                      ((uint16_t)0x00E0)

#define PWR_CR_PLS_LEV0                 ((uint16_t)0x0000)
#define PWR_CR_PLS_LEV1                 ((uint16_t)0x0020)
#define PWR_CR_PLS_LEV2                 ((uint16_t)0x0040)
#define PWR_CR_PLS_LEV3                 ((uint16_t)0x0060)
#define PWR_CR_PLS_LEV4                 ((uint16_t)0x0080)
#define PWR_CR_PLS_LEV5                 ((uint16_t)0x00A0)
#define PWR_CR_PLS_LEV6                 ((uint16_t)0x00C0)
#define PWR_CR_PLS_LEV7                 ((uint16_t)0x00E0)

#define PWR_CR_DBP                      ((uint16_t)0x0100)
#define PWR_CR_ULP                      ((uint16_t)0x0200)
#define PWR_CR_FWU                      ((uint16_t)0x0400)

#define PWR_CR_VOS                      ((uint16_t)0x1800)
#define PWR_CR_VOS_0                    ((uint16_t)0x0800)
#define PWR_CR_VOS_1                    ((uint16_t)0x1000)
#define PWR_CR_LPRUN                    ((uint16_t)0x4000)

#define PWR_CSR_WUF                     ((uint16_t)0x0001)
#define PWR_CSR_SBF                     ((uint16_t)0x0002)
#define PWR_CSR_PVDO                    ((uint16_t)0x0004)
#define PWR_CSR_VREFINTRDYF             ((uint16_t)0x0008)
#define PWR_CSR_VOSF                    ((uint16_t)0x0010)
#define PWR_CSR_REGLPF                  ((uint16_t)0x0020)

#define PWR_CSR_EWUP1                   ((uint16_t)0x0100)
#define PWR_CSR_EWUP2                   ((uint16_t)0x0200)
#define PWR_CSR_EWUP3                   ((uint16_t)0x0400)

#endif
