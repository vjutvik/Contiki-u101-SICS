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

#ifndef STM32L_DMA_H
#define STM32L_DMA_H

typedef struct {
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} STM32L_DMA;

typedef struct {
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} STM32L_DMA_CHN;

#define DMA_ISR_GIF1                    ((uint32_t)0x00000001)
#define DMA_ISR_TCIF1                   ((uint32_t)0x00000002)
#define DMA_ISR_HTIF1                   ((uint32_t)0x00000004)
#define DMA_ISR_TEIF1                   ((uint32_t)0x00000008)
#define DMA_ISR_GIF2                    ((uint32_t)0x00000010)
#define DMA_ISR_TCIF2                   ((uint32_t)0x00000020)
#define DMA_ISR_HTIF2                   ((uint32_t)0x00000040)
#define DMA_ISR_TEIF2                   ((uint32_t)0x00000080)
#define DMA_ISR_GIF3                    ((uint32_t)0x00000100)
#define DMA_ISR_TCIF3                   ((uint32_t)0x00000200)
#define DMA_ISR_HTIF3                   ((uint32_t)0x00000400)
#define DMA_ISR_TEIF3                   ((uint32_t)0x00000800)
#define DMA_ISR_GIF4                    ((uint32_t)0x00001000)
#define DMA_ISR_TCIF4                   ((uint32_t)0x00002000)
#define DMA_ISR_HTIF4                   ((uint32_t)0x00004000)
#define DMA_ISR_TEIF4                   ((uint32_t)0x00008000)
#define DMA_ISR_GIF5                    ((uint32_t)0x00010000)
#define DMA_ISR_TCIF5                   ((uint32_t)0x00020000)
#define DMA_ISR_HTIF5                   ((uint32_t)0x00040000)
#define DMA_ISR_TEIF5                   ((uint32_t)0x00080000)
#define DMA_ISR_GIF6                    ((uint32_t)0x00100000)
#define DMA_ISR_TCIF6                   ((uint32_t)0x00200000)
#define DMA_ISR_HTIF6                   ((uint32_t)0x00400000)
#define DMA_ISR_TEIF6                   ((uint32_t)0x00800000)
#define DMA_ISR_GIF7                    ((uint32_t)0x01000000)
#define DMA_ISR_TCIF7                   ((uint32_t)0x02000000)
#define DMA_ISR_HTIF7                   ((uint32_t)0x04000000)
#define DMA_ISR_TEIF7                   ((uint32_t)0x08000000)

#define DMA_IFCR_CGIF1                  ((uint32_t)0x00000001)
#define DMA_IFCR_CTCIF1                 ((uint32_t)0x00000002)
#define DMA_IFCR_CHTIF1                 ((uint32_t)0x00000004)
#define DMA_IFCR_CTEIF1                 ((uint32_t)0x00000008)
#define DMA_IFCR_CGIF2                  ((uint32_t)0x00000010)
#define DMA_IFCR_CTCIF2                 ((uint32_t)0x00000020)
#define DMA_IFCR_CHTIF2                 ((uint32_t)0x00000040)
#define DMA_IFCR_CTEIF2                 ((uint32_t)0x00000080)
#define DMA_IFCR_CGIF3                  ((uint32_t)0x00000100)
#define DMA_IFCR_CTCIF3                 ((uint32_t)0x00000200)
#define DMA_IFCR_CHTIF3                 ((uint32_t)0x00000400)
#define DMA_IFCR_CTEIF3                 ((uint32_t)0x00000800)
#define DMA_IFCR_CGIF4                  ((uint32_t)0x00001000)
#define DMA_IFCR_CTCIF4                 ((uint32_t)0x00002000)
#define DMA_IFCR_CHTIF4                 ((uint32_t)0x00004000)
#define DMA_IFCR_CTEIF4                 ((uint32_t)0x00008000)
#define DMA_IFCR_CGIF5                  ((uint32_t)0x00010000)
#define DMA_IFCR_CTCIF5                 ((uint32_t)0x00020000)
#define DMA_IFCR_CHTIF5                 ((uint32_t)0x00040000)
#define DMA_IFCR_CTEIF5                 ((uint32_t)0x00080000)
#define DMA_IFCR_CGIF6                  ((uint32_t)0x00100000)
#define DMA_IFCR_CTCIF6                 ((uint32_t)0x00200000)
#define DMA_IFCR_CHTIF6                 ((uint32_t)0x00400000)
#define DMA_IFCR_CTEIF6                 ((uint32_t)0x00800000)
#define DMA_IFCR_CGIF7                  ((uint32_t)0x01000000)
#define DMA_IFCR_CTCIF7                 ((uint32_t)0x02000000)
#define DMA_IFCR_CHTIF7                 ((uint32_t)0x04000000)
#define DMA_IFCR_CTEIF7                 ((uint32_t)0x08000000)

/* Channels */

#define DMA_CCR1_EN                     ((uint16_t)0x0001)
#define DMA_CCR1_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR1_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR1_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR1_DIR                    ((uint16_t)0x0010)
#define DMA_CCR1_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR1_PINC                   ((uint16_t)0x0040)
#define DMA_CCR1_MINC                   ((uint16_t)0x0080)
#define DMA_CCR1_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR1_MSIZE                  ((uint16_t)0x0C00)
#define DMA_CCR1_PL                     ((uint16_t)0x3000)
#define DMA_CCR1_MEM2MEM                ((uint16_t)0x4000)

#define DMA_CCR2_EN                     ((uint16_t)0x0001)
#define DMA_CCR2_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR2_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR2_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR2_DIR                    ((uint16_t)0x0010)
#define DMA_CCR2_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR2_PINC                   ((uint16_t)0x0040)
#define DMA_CCR2_MINC                   ((uint16_t)0x0080)
#define DMA_CCR2_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR2_MSIZE                  ((uint16_t)0x0C00)
#define DMA_CCR2_PL                     ((uint16_t)0x3000)
#define DMA_CCR2_MEM2MEM                ((uint16_t)0x4000)

#define DMA_CCR3_EN                     ((uint16_t)0x0001)
#define DMA_CCR3_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR3_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR3_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR3_DIR                    ((uint16_t)0x0010)
#define DMA_CCR3_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR3_PINC                   ((uint16_t)0x0040)
#define DMA_CCR3_MINC                   ((uint16_t)0x0080)
#define DMA_CCR3_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR3_MSIZE                  ((uint16_t)0x0C00)
#define DMA_CCR3_PL                     ((uint16_t)0x3000)
#define DMA_CCR3_MEM2MEM                ((uint16_t)0x4000)

#define DMA_CCR4_EN                     ((uint16_t)0x0001)
#define DMA_CCR4_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR4_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR4_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR4_DIR                    ((uint16_t)0x0010)
#define DMA_CCR4_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR4_PINC                   ((uint16_t)0x0040)
#define DMA_CCR4_MINC                   ((uint16_t)0x0080)
#define DMA_CCR4_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR4_MSIZE                  ((uint16_t)0x0C00)
#define DMA_CCR4_PL                     ((uint16_t)0x3000)
#define DMA_CCR4_MEM2MEM                ((uint16_t)0x4000)

#define DMA_CCR5_EN                     ((uint16_t)0x0001)
#define DMA_CCR5_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR5_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR5_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR5_DIR                    ((uint16_t)0x0010)
#define DMA_CCR5_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR5_PINC                   ((uint16_t)0x0040)
#define DMA_CCR5_MINC                   ((uint16_t)0x0080)
#define DMA_CCR5_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR5_MSIZE                  ((uint16_t)0x0C00)
#define DMA_CCR5_PL                     ((uint16_t)0x3000)
#define DMA_CCR5_MEM2MEM                ((uint16_t)0x4000)

#define DMA_CCR6_EN                     ((uint16_t)0x0001)
#define DMA_CCR6_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR6_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR6_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR6_DIR                    ((uint16_t)0x0010)
#define DMA_CCR6_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR6_PINC                   ((uint16_t)0x0040)
#define DMA_CCR6_MINC                   ((uint16_t)0x0080)
#define DMA_CCR6_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR6_MSIZE                  ((uint16_t)0x0C00)
#define DMA_CCR6_PL                     ((uint16_t)0x3000)
#define DMA_CCR6_MEM2MEM                ((uint16_t)0x4000)

#define DMA_CCR7_EN                     ((uint16_t)0x0001)
#define DMA_CCR7_TCIE                   ((uint16_t)0x0002)
#define DMA_CCR7_HTIE                   ((uint16_t)0x0004)
#define DMA_CCR7_TEIE                   ((uint16_t)0x0008)
#define DMA_CCR7_DIR                    ((uint16_t)0x0010)
#define DMA_CCR7_CIRC                   ((uint16_t)0x0020)
#define DMA_CCR7_PINC                   ((uint16_t)0x0040)
#define DMA_CCR7_MINC                   ((uint16_t)0x0080)
#define DMA_CCR7_PSIZE                  ((uint16_t)0x0300)
#define DMA_CCR7_MSIZE                  ((uint16_t)0x0c00)
#define DMA_CCR7_PL                     ((uint16_t)0x3000)
#define DMA_CCR7_MEM2MEM                ((uint16_t)0x4000)

#define DMA_PRIORITY_VERYHIGH           ((uint32_t)0x00003000)
#define DMA_PRIORITY_HIGH               ((uint32_t)0x00002000)
#define DMA_PRIORITY_MEDIUM             ((uint32_t)0x00001000)
#define DMA_PRIORITY_LOW                ((uint32_t)0x00000000)

#define DMA_DIR_PERIPHERALDST           ((uint32_t)0x00000010)
#define DMA_DIR_PERIPHERALSRC           ((uint32_t)0x00000000)

#define DMA_PERIPHERALINC_ENABLE        ((uint32_t)0x00000040)
#define DMA_PERIPHERALINC_DISABLE       ((uint32_t)0x00000000)

#define DMA_MEMORYINC_ENABLE            ((uint32_t)0x00000080)
#define DMA_MEMORYINC_DISABLE           ((uint32_t)0x00000000)

#define DMA_PERIPHERALDATASIZE_BYTE     ((uint32_t)0x00000000)
#define DMA_PERIPHERALDATASIZE_HALFWORD ((uint32_t)0x00000100)
#define DMA_PERIPHERALDATASIZE_WORD     ((uint32_t)0x00000200)

#define DMA_MEMORYDATASIZE_BYTE         ((uint32_t)0x00000000)
#define DMA_MEMORYDATASIZE_HALFWORD     ((uint32_t)0x00000400)
#define DMA_MEMORYDATASIZE_WORD         ((uint32_t)0x00000800)

#define DMA_MODE_CIRCULAR               ((uint32_t)0x00000020)
#define DMA_MODE_NORMAL                 ((uint32_t)0x00000000)


#endif

