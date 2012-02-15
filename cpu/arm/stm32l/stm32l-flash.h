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

#ifndef STM32L_FLASH_H
#define STM32L_FLASH_H

#include "stm32l.h"

typedef struct {
  volatile uint32_t ACR;
  volatile uint32_t PECR;
  volatile uint32_t PDKEYR;
  volatile uint32_t PEKEYR;
  volatile uint32_t PRGKEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
} STM32L_FLASH;

#define  FLASH_ACR_LATENCY              ((uint32_t)0x00000001)
#define  FLASH_ACR_PRFTEN               ((uint32_t)0x00000002)
#define  FLASH_ACR_ACC64                ((uint32_t)0x00000004)
#define  FLASH_ACR_SLEEP_PD             ((uint32_t)0x00000008)
#define  FLASH_ACR_RUN_PD               ((uint32_t)0x00000010)

#define FLASH_PECR_PELOCK               ((uint32_t)0x00000001)
#define FLASH_PECR_PRGLOCK              ((uint32_t)0x00000002)
#define FLASH_PECR_OPTLOCK              ((uint32_t)0x00000004)
#define FLASH_PECR_PROG                 ((uint32_t)0x00000008)
#define FLASH_PECR_DATA                 ((uint32_t)0x00000010)
#define FLASH_PECR_FTDW                 ((uint32_t)0x00000100)
#define FLASH_PECR_ERASE                ((uint32_t)0x00000200)
#define FLASH_PECR_FPRG                 ((uint32_t)0x00000400)
#define FLASH_PECR_EOPIE                ((uint32_t)0x00010000)
#define FLASH_PECR_ERRIE                ((uint32_t)0x00020000)
#define FLASH_PECR_OBL_LAUNCH           ((uint32_t)0x00040000)

#define FLASH_SR_BSY                    ((uint32_t)0x00000001)
#define FLASH_SR_EOP                    ((uint32_t)0x00000002)
#define FLASH_SR_ENHV                   ((uint32_t)0x00000004)
#define FLASH_SR_READY                  ((uint32_t)0x00000008)

#define FLASH_SR_WRPERR                 ((uint32_t)0x00000100)
#define FLASH_SR_PGAERR                 ((uint32_t)0x00000200)
#define FLASH_SR_SIZERR                 ((uint32_t)0x00000400)
#define FLASH_SR_OPTVERR                ((uint32_t)0x00000800)

#define FLASH_OBR_RDPRT                 ((uint16_t)0x000000AA)
#define FLASH_OBR_BOR_LEV               ((uint16_t)0x000f0000)
#define FLASH_OBR_USER                  ((uint32_t)0x00700000)
#define FLASH_OBR_IWDG_SW               ((uint32_t)0x00100000)
#define FLASH_OBR_NRST_STOP             ((uint32_t)0x00200000)
#define FLASH_OBR_NRST_STDBY            ((uint32_t)0x00400000)

void stm32l_flash_setup(void);

#endif
