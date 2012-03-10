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

#ifndef STM32_SCB_H
#define STM32_SCB_H

#include <stdint.h>

/*
  Relevant information in:
  Cortex-M3 Technical Reference Manual
  Cortex-M3 Devices Generic User Guide

  "The System control block (SCB) provides system implementation
  information, and system control. This includes configuration, control,
  and reporting of the system exceptions."
  
  "0xE000ED00 - 0xE000ED8F. System Control Block, including:
   — CPUID
   — System control, configuration, and status
   — Fault reporting"

*/

typedef struct {
  const volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint32_t SHPR[3];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
} STM32_SCB;

/* System Control Space */
#define SCS_BASE                        ((uint32_t)0xe000e000)
#define SCB_BASE                        (SCS_BASE + 0x0d00)
#define SCB                             ((STM32_SCB *)SCB_BASE)

#define SCB_CPUID_REVISION              ((uint32_t)0x0000000f)
#define SCB_CPUID_PARTNO                ((uint32_t)0x0000fff0)
#define SCB_CPUID_CONSTANT              ((uint32_t)0x000f0000)
#define SCB_CPUID_VARIANT               ((uint32_t)0x00f00000)
#define SCB_CPUID_IMPLEMENTER           ((uint32_t)0xff000000)

#define SCB_ICSR_VECTACTIVE             ((uint32_t)0x000001ff)
#define SCB_ICSR_RETTOBASE              ((uint32_t)0x00000800)
#define SCB_ICSR_VECTPENDING            ((uint32_t)0x003ff000)
#define SCB_ICSR_ISRPENDING             ((uint32_t)0x00400000)
#define SCB_ICSR_ISRPREEMPT             ((uint32_t)0x00800000)
#define SCB_ICSR_PENDSTCLR              ((uint32_t)0x02000000)
#define SCB_ICSR_PENDSTSET              ((uint32_t)0x04000000)
#define SCB_ICSR_PENDSVCLR              ((uint32_t)0x08000000)
#define SCB_ICSR_PENDSVSET              ((uint32_t)0x10000000)
#define SCB_ICSR_NMIPENDSET             ((uint32_t)0x80000000)

#define SCB_AIRCR_VECTRESET             ((uint32_t)0x00000001)
#define SCB_AIRCR_VECTCLRACTIVE         ((uint32_t)0x00000002)
#define SCB_AIRCR_SYSRESETREQ           ((uint32_t)0x00000004)

#define SCB_SCR_SLEEPONEXIT             ((uint8_t)0x02)       
#define SCB_SCR_SLEEPDEEP               ((uint8_t)0x04)       
#define SCB_SCR_SEVONPEND               ((uint8_t)0x10)       

#define SCB_CCR_NONBASETHRDENA          ((uint16_t)0x0001)    
#define SCB_CCR_USERSETMPEND            ((uint16_t)0x0002)    
#define SCB_CCR_UNALIGN_TRP             ((uint16_t)0x0008)    
#define SCB_CCR_DIV_0_TRP               ((uint16_t)0x0010)    
#define SCB_CCR_BFHFNMIGN               ((uint16_t)0x0100)    
#define SCB_CCR_STKALIGN                ((uint16_t)0x0200)    

#define SCB_SHCSR_MEMFAULTACT           ((uint32_t)0x00000001)
#define SCB_SHCSR_BUSFAULTACT           ((uint32_t)0x00000002)
#define SCB_SHCSR_USGFAULTACT           ((uint32_t)0x00000008)
#define SCB_SHCSR_SVCALLACT             ((uint32_t)0x00000080)
#define SCB_SHCSR_MONITORACT            ((uint32_t)0x00000100)
#define SCB_SHCSR_PENDSVACT             ((uint32_t)0x00000400)
#define SCB_SHCSR_SYSTICKACT            ((uint32_t)0x00000800)
#define SCB_SHCSR_USGFAULTPENDED        ((uint32_t)0x00001000)
#define SCB_SHCSR_MEMFAULTPENDED        ((uint32_t)0x00002000)
#define SCB_SHCSR_BUSFAULTPENDED        ((uint32_t)0x00004000)
#define SCB_SHCSR_SVCALLPENDED          ((uint32_t)0x00008000)
#define SCB_SHCSR_MEMFAULTENA           ((uint32_t)0x00010000)
#define SCB_SHCSR_BUSFAULTENA           ((uint32_t)0x00020000)
#define SCB_SHCSR_USGFAULTENA           ((uint32_t)0x00040000)

#define SCB_CFSR_IACCVIOL               ((uint32_t)0x00000001)
#define SCB_CFSR_DACCVIOL               ((uint32_t)0x00000002)
#define SCB_CFSR_MUNSTKERR              ((uint32_t)0x00000008)
#define SCB_CFSR_MSTKERR                ((uint32_t)0x00000010)
#define SCB_CFSR_MMARVALID              ((uint32_t)0x00000080)

#define SCB_CFSR_IBUSERR                ((uint32_t)0x00000100)
#define SCB_CFSR_PRECISERR              ((uint32_t)0x00000200)
#define SCB_CFSR_IMPRECISERR            ((uint32_t)0x00000400)
#define SCB_CFSR_UNSTKERR               ((uint32_t)0x00000800)
#define SCB_CFSR_STKERR                 ((uint32_t)0x00001000)
#define SCB_CFSR_BFARVALID              ((uint32_t)0x00008000)

#define SCB_CFSR_UNDEFINSTR             ((uint32_t)0x00010000)
#define SCB_CFSR_INVSTATE               ((uint32_t)0x00020000)
#define SCB_CFSR_INVPC                  ((uint32_t)0x00040000)
#define SCB_CFSR_NOCP                   ((uint32_t)0x00080000)
#define SCB_CFSR_UNALIGNED              ((uint32_t)0x01000000)
#define SCB_CFSR_DIVBYZERO              ((uint32_t)0x02000000)

#define SCB_HFSR_VECTTBL                ((uint32_t)0x00000002)
#define SCB_HFSR_FORCED                 ((uint32_t)0x40000000)
#define SCB_HFSR_DEBUGEVT               ((uint32_t)0x80000000)

#define SCB_DFSR_HALTED                 ((uint8_t)0x01)       
#define SCB_DFSR_BKPT                   ((uint8_t)0x02)       
#define SCB_DFSR_DWTTRAP                ((uint8_t)0x04)       
#define SCB_DFSR_VCATCH                 ((uint8_t)0x08)       
#define SCB_DFSR_EXTERNAL               ((uint8_t)0x10)       

#endif
