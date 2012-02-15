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

#ifndef STM32_NVIC_H
#define STM32_NVIC_H

#include "stm32-scb.h"
/*

  From Cortex-M3 Technical Reference Manual (Revision: r1p1)

  ---8<---  

  Irq 0 to 31 Set Enable Register       Read/write  0xE000E100 0x00000000
  [...]
  Irq 224 to 239 Set Enable Register    Read/write  0xE000E11C 0x00000000
  Irq 0 to 31 Clear Enable Register     Read/write  0xE000E180 0x00000000
  [...]
  Irq 224 to 239 Clear Enable Register  Read/write  0xE000E19C 0x00000000
  Irq 0 to 31 Set Pending Register      Read/write  0xE000E200 0x00000000 
  [...]
  Irq 224 to 239 Set Pending Register   Read/write  0xE000E21C 0x00000000
  Irq 0 to 31 Clear Pending Register    Read/write  0xE000E280 0x00000000
  [...]
  Irq 224 to 239 Clear Pending Register Read/write  0xE000E29C 0x00000000
  Irq 0 to 31 Active Bit Register       Read-only   0xE000E300 0x00000000
  [...]
  Irq 224 to 239 Active Bit Register    Read-only   0xE000E31C 0x00000000
  Irq 0 to 31 Priority Register         Read/write  0xE000E400 0x00000000
  [...]
  Irq 236 to 239 Priority Register      Read/write  0xE000E41F 0x00000000

  ---8<---  

  Implementations of Cortex M3 will support a subset of all these interrupt
  lines and registers.

*/

typedef struct {
  volatile uint32_t ISER[8];
  volatile uint32_t R0[24];
  volatile uint32_t ICER[8];
  volatile uint32_t R1[24];
  volatile uint32_t ISPR[8];
  volatile uint32_t R2[24];
  volatile uint32_t ICPR[8];
  volatile uint32_t R3[24];
  volatile uint32_t IABR[8];
  volatile uint32_t R4[24];
  volatile uint32_t IPR[8];
} STM32_NVIC;

#define NVIC_BASE                       (SCS_BASE + 0x0100)
#define NVIC                            ((STM32_NVIC *)NVIC_BASE)

#define NVIC_ENABLE_INT(i) \
  WRITE_REG(NVIC->ISER[(i)/32], 1<<((i) & 0x1f))
#define NVIC_DISABLE_INT(i) \
  WRITE_REG(NVIC->ICER[(i)/32], 1<<((i) & 0x1f))
#define NVIC_SET_PENDING(i) \
  WRITE_REG(NVIC->ISPR[(i)/32], 1<<((i) & 0x1f))
#define NVIC_CLEAR_PENDING(i) \
  WRITE_REG(NVIC->ICPR[(i)/32], 1<<((i) & 0x1f))
#define NVIC_SET_PRIORITY(i,p) \
  MODIFY_REG(NVIC->IPR[(i)/4], 0xf<<(((i)&3)*8), (p)<<(((i)&3)*8))
#define NVIC_SET_SYSTICK_PRI(p) \
  MODIFY_REG(SCB->SHPR[2], 0xf<<24, (p)<<24)

#define NVIC_ISER_SETENA                ((uint32_t)0xffffffff)
#define NVIC_ISER_SETENA_0              ((uint32_t)0x00000001)
#define NVIC_ISER_SETENA_1              ((uint32_t)0x00000002)
#define NVIC_ISER_SETENA_2              ((uint32_t)0x00000004)
#define NVIC_ISER_SETENA_3              ((uint32_t)0x00000008)
#define NVIC_ISER_SETENA_4              ((uint32_t)0x00000010)
#define NVIC_ISER_SETENA_5              ((uint32_t)0x00000020)
#define NVIC_ISER_SETENA_6              ((uint32_t)0x00000040)
#define NVIC_ISER_SETENA_7              ((uint32_t)0x00000080)
#define NVIC_ISER_SETENA_8              ((uint32_t)0x00000100)
#define NVIC_ISER_SETENA_9              ((uint32_t)0x00000200)
#define NVIC_ISER_SETENA_10             ((uint32_t)0x00000400)
#define NVIC_ISER_SETENA_11             ((uint32_t)0x00000800)
#define NVIC_ISER_SETENA_12             ((uint32_t)0x00001000)
#define NVIC_ISER_SETENA_13             ((uint32_t)0x00002000)
#define NVIC_ISER_SETENA_14             ((uint32_t)0x00004000)
#define NVIC_ISER_SETENA_15             ((uint32_t)0x00008000)
#define NVIC_ISER_SETENA_16             ((uint32_t)0x00010000)
#define NVIC_ISER_SETENA_17             ((uint32_t)0x00020000)
#define NVIC_ISER_SETENA_18             ((uint32_t)0x00040000)
#define NVIC_ISER_SETENA_19             ((uint32_t)0x00080000)
#define NVIC_ISER_SETENA_20             ((uint32_t)0x00100000)
#define NVIC_ISER_SETENA_21             ((uint32_t)0x00200000)
#define NVIC_ISER_SETENA_22             ((uint32_t)0x00400000)
#define NVIC_ISER_SETENA_23             ((uint32_t)0x00800000)
#define NVIC_ISER_SETENA_24             ((uint32_t)0x01000000)
#define NVIC_ISER_SETENA_25             ((uint32_t)0x02000000)
#define NVIC_ISER_SETENA_26             ((uint32_t)0x04000000)
#define NVIC_ISER_SETENA_27             ((uint32_t)0x08000000)
#define NVIC_ISER_SETENA_28             ((uint32_t)0x10000000)
#define NVIC_ISER_SETENA_29             ((uint32_t)0x20000000)
#define NVIC_ISER_SETENA_30             ((uint32_t)0x40000000)
#define NVIC_ISER_SETENA_31             ((uint32_t)0x80000000)

#define NVIC_ICER_CLRENA                ((uint32_t)0xffffffff) 
#define NVIC_ICER_CLRENA_0              ((uint32_t)0x00000001)
#define NVIC_ICER_CLRENA_1              ((uint32_t)0x00000002)
#define NVIC_ICER_CLRENA_2              ((uint32_t)0x00000004)
#define NVIC_ICER_CLRENA_3              ((uint32_t)0x00000008)
#define NVIC_ICER_CLRENA_4              ((uint32_t)0x00000010)
#define NVIC_ICER_CLRENA_5              ((uint32_t)0x00000020)
#define NVIC_ICER_CLRENA_6              ((uint32_t)0x00000040)
#define NVIC_ICER_CLRENA_7              ((uint32_t)0x00000080)
#define NVIC_ICER_CLRENA_8              ((uint32_t)0x00000100)
#define NVIC_ICER_CLRENA_9              ((uint32_t)0x00000200)
#define NVIC_ICER_CLRENA_10             ((uint32_t)0x00000400)
#define NVIC_ICER_CLRENA_11             ((uint32_t)0x00000800)
#define NVIC_ICER_CLRENA_12             ((uint32_t)0x00001000)
#define NVIC_ICER_CLRENA_13             ((uint32_t)0x00002000)
#define NVIC_ICER_CLRENA_14             ((uint32_t)0x00004000)
#define NVIC_ICER_CLRENA_15             ((uint32_t)0x00008000)
#define NVIC_ICER_CLRENA_16             ((uint32_t)0x00010000)
#define NVIC_ICER_CLRENA_17             ((uint32_t)0x00020000)
#define NVIC_ICER_CLRENA_18             ((uint32_t)0x00040000)
#define NVIC_ICER_CLRENA_19             ((uint32_t)0x00080000)
#define NVIC_ICER_CLRENA_20             ((uint32_t)0x00100000)
#define NVIC_ICER_CLRENA_21             ((uint32_t)0x00200000)
#define NVIC_ICER_CLRENA_22             ((uint32_t)0x00400000)
#define NVIC_ICER_CLRENA_23             ((uint32_t)0x00800000)
#define NVIC_ICER_CLRENA_24             ((uint32_t)0x01000000)
#define NVIC_ICER_CLRENA_25             ((uint32_t)0x02000000)
#define NVIC_ICER_CLRENA_26             ((uint32_t)0x04000000)
#define NVIC_ICER_CLRENA_27             ((uint32_t)0x08000000)
#define NVIC_ICER_CLRENA_28             ((uint32_t)0x10000000)
#define NVIC_ICER_CLRENA_29             ((uint32_t)0x20000000)
#define NVIC_ICER_CLRENA_30             ((uint32_t)0x40000000)
#define NVIC_ICER_CLRENA_31             ((uint32_t)0x80000000)

#define NVIC_ISPR_SETPEND               ((uint32_t)0xffffffff)
#define NVIC_ISPR_SETPEND_0             ((uint32_t)0x00000001)
#define NVIC_ISPR_SETPEND_1             ((uint32_t)0x00000002)
#define NVIC_ISPR_SETPEND_2             ((uint32_t)0x00000004)
#define NVIC_ISPR_SETPEND_3             ((uint32_t)0x00000008)
#define NVIC_ISPR_SETPEND_4             ((uint32_t)0x00000010)
#define NVIC_ISPR_SETPEND_5             ((uint32_t)0x00000020)
#define NVIC_ISPR_SETPEND_6             ((uint32_t)0x00000040)
#define NVIC_ISPR_SETPEND_7             ((uint32_t)0x00000080)
#define NVIC_ISPR_SETPEND_8             ((uint32_t)0x00000100)
#define NVIC_ISPR_SETPEND_9             ((uint32_t)0x00000200)
#define NVIC_ISPR_SETPEND_10            ((uint32_t)0x00000400)
#define NVIC_ISPR_SETPEND_11            ((uint32_t)0x00000800)
#define NVIC_ISPR_SETPEND_12            ((uint32_t)0x00001000)
#define NVIC_ISPR_SETPEND_13            ((uint32_t)0x00002000)
#define NVIC_ISPR_SETPEND_14            ((uint32_t)0x00004000)
#define NVIC_ISPR_SETPEND_15            ((uint32_t)0x00008000)
#define NVIC_ISPR_SETPEND_16            ((uint32_t)0x00010000)
#define NVIC_ISPR_SETPEND_17            ((uint32_t)0x00020000)
#define NVIC_ISPR_SETPEND_18            ((uint32_t)0x00040000)
#define NVIC_ISPR_SETPEND_19            ((uint32_t)0x00080000)
#define NVIC_ISPR_SETPEND_20            ((uint32_t)0x00100000)
#define NVIC_ISPR_SETPEND_21            ((uint32_t)0x00200000)
#define NVIC_ISPR_SETPEND_22            ((uint32_t)0x00400000)
#define NVIC_ISPR_SETPEND_23            ((uint32_t)0x00800000)
#define NVIC_ISPR_SETPEND_24            ((uint32_t)0x01000000)
#define NVIC_ISPR_SETPEND_25            ((uint32_t)0x02000000)
#define NVIC_ISPR_SETPEND_26            ((uint32_t)0x04000000)
#define NVIC_ISPR_SETPEND_27            ((uint32_t)0x08000000)
#define NVIC_ISPR_SETPEND_28            ((uint32_t)0x10000000)
#define NVIC_ISPR_SETPEND_29            ((uint32_t)0x20000000)
#define NVIC_ISPR_SETPEND_30            ((uint32_t)0x40000000)
#define NVIC_ISPR_SETPEND_31            ((uint32_t)0x80000000)

#define NVIC_ICPR_CLRPEND               ((uint32_t)0xffffffff)
#define NVIC_ICPR_CLRPEND_0             ((uint32_t)0x00000001)
#define NVIC_ICPR_CLRPEND_1             ((uint32_t)0x00000002)
#define NVIC_ICPR_CLRPEND_2             ((uint32_t)0x00000004)
#define NVIC_ICPR_CLRPEND_3             ((uint32_t)0x00000008)
#define NVIC_ICPR_CLRPEND_4             ((uint32_t)0x00000010)
#define NVIC_ICPR_CLRPEND_5             ((uint32_t)0x00000020)
#define NVIC_ICPR_CLRPEND_6             ((uint32_t)0x00000040)
#define NVIC_ICPR_CLRPEND_7             ((uint32_t)0x00000080)
#define NVIC_ICPR_CLRPEND_8             ((uint32_t)0x00000100)
#define NVIC_ICPR_CLRPEND_9             ((uint32_t)0x00000200)
#define NVIC_ICPR_CLRPEND_10            ((uint32_t)0x00000400)
#define NVIC_ICPR_CLRPEND_11            ((uint32_t)0x00000800)
#define NVIC_ICPR_CLRPEND_12            ((uint32_t)0x00001000)
#define NVIC_ICPR_CLRPEND_13            ((uint32_t)0x00002000)
#define NVIC_ICPR_CLRPEND_14            ((uint32_t)0x00004000)
#define NVIC_ICPR_CLRPEND_15            ((uint32_t)0x00008000)
#define NVIC_ICPR_CLRPEND_16            ((uint32_t)0x00010000)
#define NVIC_ICPR_CLRPEND_17            ((uint32_t)0x00020000)
#define NVIC_ICPR_CLRPEND_18            ((uint32_t)0x00040000)
#define NVIC_ICPR_CLRPEND_19            ((uint32_t)0x00080000)
#define NVIC_ICPR_CLRPEND_20            ((uint32_t)0x00100000)
#define NVIC_ICPR_CLRPEND_21            ((uint32_t)0x00200000)
#define NVIC_ICPR_CLRPEND_22            ((uint32_t)0x00400000)
#define NVIC_ICPR_CLRPEND_23            ((uint32_t)0x00800000)
#define NVIC_ICPR_CLRPEND_24            ((uint32_t)0x01000000)
#define NVIC_ICPR_CLRPEND_25            ((uint32_t)0x02000000)
#define NVIC_ICPR_CLRPEND_26            ((uint32_t)0x04000000)
#define NVIC_ICPR_CLRPEND_27            ((uint32_t)0x08000000)
#define NVIC_ICPR_CLRPEND_28            ((uint32_t)0x10000000)
#define NVIC_ICPR_CLRPEND_29            ((uint32_t)0x20000000)
#define NVIC_ICPR_CLRPEND_30            ((uint32_t)0x40000000)
#define NVIC_ICPR_CLRPEND_31            ((uint32_t)0x80000000)

#define NVIC_IABR_ACTIVE                ((uint32_t)0xffffffff)
#define NVIC_IABR_ACTIVE_0              ((uint32_t)0x00000001)
#define NVIC_IABR_ACTIVE_1              ((uint32_t)0x00000002)
#define NVIC_IABR_ACTIVE_2              ((uint32_t)0x00000004)
#define NVIC_IABR_ACTIVE_3              ((uint32_t)0x00000008)
#define NVIC_IABR_ACTIVE_4              ((uint32_t)0x00000010)
#define NVIC_IABR_ACTIVE_5              ((uint32_t)0x00000020)
#define NVIC_IABR_ACTIVE_6              ((uint32_t)0x00000040)
#define NVIC_IABR_ACTIVE_7              ((uint32_t)0x00000080)
#define NVIC_IABR_ACTIVE_8              ((uint32_t)0x00000100)
#define NVIC_IABR_ACTIVE_9              ((uint32_t)0x00000200)
#define NVIC_IABR_ACTIVE_10             ((uint32_t)0x00000400)
#define NVIC_IABR_ACTIVE_11             ((uint32_t)0x00000800)
#define NVIC_IABR_ACTIVE_12             ((uint32_t)0x00001000)
#define NVIC_IABR_ACTIVE_13             ((uint32_t)0x00002000)
#define NVIC_IABR_ACTIVE_14             ((uint32_t)0x00004000)
#define NVIC_IABR_ACTIVE_15             ((uint32_t)0x00008000)
#define NVIC_IABR_ACTIVE_16             ((uint32_t)0x00010000)
#define NVIC_IABR_ACTIVE_17             ((uint32_t)0x00020000)
#define NVIC_IABR_ACTIVE_18             ((uint32_t)0x00040000)
#define NVIC_IABR_ACTIVE_19             ((uint32_t)0x00080000)
#define NVIC_IABR_ACTIVE_20             ((uint32_t)0x00100000)
#define NVIC_IABR_ACTIVE_21             ((uint32_t)0x00200000)
#define NVIC_IABR_ACTIVE_22             ((uint32_t)0x00400000)
#define NVIC_IABR_ACTIVE_23             ((uint32_t)0x00800000)
#define NVIC_IABR_ACTIVE_24             ((uint32_t)0x01000000)
#define NVIC_IABR_ACTIVE_25             ((uint32_t)0x02000000)
#define NVIC_IABR_ACTIVE_26             ((uint32_t)0x04000000)
#define NVIC_IABR_ACTIVE_27             ((uint32_t)0x08000000)
#define NVIC_IABR_ACTIVE_28             ((uint32_t)0x10000000)
#define NVIC_IABR_ACTIVE_29             ((uint32_t)0x20000000)
#define NVIC_IABR_ACTIVE_30             ((uint32_t)0x40000000)
#define NVIC_IABR_ACTIVE_31             ((uint32_t)0x80000000)

#define NVIC_IPR0_PRI_0                 ((uint32_t)0x000000ff)
#define NVIC_IPR0_PRI_1                 ((uint32_t)0x0000ff00)
#define NVIC_IPR0_PRI_2                 ((uint32_t)0x00ff0000)
#define NVIC_IPR0_PRI_3                 ((uint32_t)0xff000000)

#define NVIC_IPR1_PRI_4                 ((uint32_t)0x000000ff)
#define NVIC_IPR1_PRI_5                 ((uint32_t)0x0000ff00)
#define NVIC_IPR1_PRI_6                 ((uint32_t)0x00ff0000)
#define NVIC_IPR1_PRI_7                 ((uint32_t)0xff000000)

#define NVIC_IPR2_PRI_8                 ((uint32_t)0x000000ff)
#define NVIC_IPR2_PRI_9                 ((uint32_t)0x0000ff00)
#define NVIC_IPR2_PRI_10                ((uint32_t)0x00ff0000)
#define NVIC_IPR2_PRI_11                ((uint32_t)0xff000000)

#define NVIC_IPR3_PRI_12                ((uint32_t)0x000000ff)
#define NVIC_IPR3_PRI_13                ((uint32_t)0x0000ff00)
#define NVIC_IPR3_PRI_14                ((uint32_t)0x00ff0000)
#define NVIC_IPR3_PRI_15                ((uint32_t)0xff000000)

#define NVIC_IPR4_PRI_16                ((uint32_t)0x000000ff)
#define NVIC_IPR4_PRI_17                ((uint32_t)0x0000ff00)
#define NVIC_IPR4_PRI_18                ((uint32_t)0x00ff0000)
#define NVIC_IPR4_PRI_19                ((uint32_t)0xff000000)

#define NVIC_IPR5_PRI_20                ((uint32_t)0x000000ff)
#define NVIC_IPR5_PRI_21                ((uint32_t)0x0000ff00)
#define NVIC_IPR5_PRI_22                ((uint32_t)0x00ff0000)
#define NVIC_IPR5_PRI_23                ((uint32_t)0xff000000)

#define NVIC_IPR6_PRI_24                ((uint32_t)0x000000ff)
#define NVIC_IPR6_PRI_25                ((uint32_t)0x0000ff00)
#define NVIC_IPR6_PRI_26                ((uint32_t)0x00ff0000)
#define NVIC_IPR6_PRI_27                ((uint32_t)0xff000000)

#define NVIC_IPR7_PRI_28                ((uint32_t)0x000000ff)
#define NVIC_IPR7_PRI_29                ((uint32_t)0x0000ff00)
#define NVIC_IPR7_PRI_30                ((uint32_t)0x00ff0000)
#define NVIC_IPR7_PRI_31                ((uint32_t)0xff000000)

#endif
