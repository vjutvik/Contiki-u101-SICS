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

#ifndef STM32L_USB_H
#define STM32L_USB_H

/*
  USB endpoint n register (USB_EPnR), n=[0..7]
  Bit 15 CTR_RX: Correct Transfer for reception
  Bit 14 DTOG_RX: Data Toggle, for reception transfers
  Bits 13:12 STAT_RX [1:0]: Status bits, for reception transfers
  Bit 11 SETUP: Setup transaction completed
  Bits 10:9 EP_TYPE[1:0]: Endpoint type
  Bit 8 EP_KIND: Endpoint kind
  Bit 7 CTR_TX: Correct Transfer for transmission
  Bit 6 DTOG_TX: Data Toggle, for transmission transfers
  Bits 5:4 STAT_TX [1:0]: Status bits, for transmission transfers
  Bits 3:0 EA[3:0]: Endpoint address
*/
#define USB_EP0R_EA                     ((uint16_t)0x000f)
#define USB_EP0R_STAT_TX                ((uint16_t)0x0030)
#define USB_EP0R_STAT_TX_0              ((uint16_t)0x0010)
#define USB_EP0R_STAT_TX_1              ((uint16_t)0x0020)
#define USB_EP0R_DTOG_TX                ((uint16_t)0x0040)
#define USB_EP0R_CTR_TX                 ((uint16_t)0x0080)
#define USB_EP0R_EP_KIND                ((uint16_t)0x0100)
#define USB_EP0R_EP_TYPE                ((uint16_t)0x0600)
#define USB_EP0R_EP_TYPE_0              ((uint16_t)0x0200)
#define USB_EP0R_EP_TYPE_1              ((uint16_t)0x0400)
#define USB_EP0R_SETUP                  ((uint16_t)0x0800)
#define USB_EP0R_STAT_RX                ((uint16_t)0x3000)
#define USB_EP0R_STAT_RX_0              ((uint16_t)0x1000)
#define USB_EP0R_STAT_RX_1              ((uint16_t)0x2000)
#define USB_EP0R_DTOG_RX                ((uint16_t)0x4000)
#define USB_EP0R_CTR_RX                 ((uint16_t)0x8000)

#define USB_EP1R_EA                     ((uint16_t)0x000f)
#define USB_EP1R_STAT_TX                ((uint16_t)0x0030)
#define USB_EP1R_STAT_TX_0              ((uint16_t)0x0010)
#define USB_EP1R_STAT_TX_1              ((uint16_t)0x0020)
#define USB_EP1R_DTOG_TX                ((uint16_t)0x0040)
#define USB_EP1R_CTR_TX                 ((uint16_t)0x0080)
#define USB_EP1R_EP_KIND                ((uint16_t)0x0100)
#define USB_EP1R_EP_TYPE                ((uint16_t)0x0600)
#define USB_EP1R_EP_TYPE_0              ((uint16_t)0x0200)
#define USB_EP1R_EP_TYPE_1              ((uint16_t)0x0400)
#define USB_EP1R_SETUP                  ((uint16_t)0x0800)
#define USB_EP1R_STAT_RX                ((uint16_t)0x3000)
#define USB_EP1R_STAT_RX_0              ((uint16_t)0x1000)
#define USB_EP1R_STAT_RX_1              ((uint16_t)0x2000)
#define USB_EP1R_DTOG_RX                ((uint16_t)0x4000)
#define USB_EP1R_CTR_RX                 ((uint16_t)0x8000)

#define USB_EP3R_EA                     ((uint16_t)0x000f)
#define USB_EP3R_STAT_TX                ((uint16_t)0x0030)
#define USB_EP3R_STAT_TX_0              ((uint16_t)0x0010)
#define USB_EP3R_STAT_TX_1              ((uint16_t)0x0020)
#define USB_EP3R_DTOG_TX                ((uint16_t)0x0040)
#define USB_EP3R_CTR_TX                 ((uint16_t)0x0080)
#define USB_EP3R_EP_KIND                ((uint16_t)0x0100)
#define USB_EP3R_EP_TYPE                ((uint16_t)0x0600)
#define USB_EP3R_EP_TYPE_0              ((uint16_t)0x0200)
#define USB_EP3R_EP_TYPE_1              ((uint16_t)0x0400)
#define USB_EP3R_SETUP                  ((uint16_t)0x0800)
#define USB_EP3R_STAT_RX                ((uint16_t)0x3000)
#define USB_EP3R_STAT_RX_0              ((uint16_t)0x1000)
#define USB_EP3R_STAT_RX_1              ((uint16_t)0x2000)
#define USB_EP3R_DTOG_RX                ((uint16_t)0x4000)
#define USB_EP3R_CTR_RX                 ((uint16_t)0x8000)

/*
  Bit 15 CTRM: Correct transfer interrupt mask
  Bit 14 PMAOVRM: Packet memory area over / underrun interrupt mask
  Bit 13 ERRM: Error interrupt mask
  Bit 12 WKUPM: Wakeup interrupt mask
  Bit 11 SUSPM: Suspend mode interrupt mask
  Bit 10 RESETM: USB reset interrupt mask
  Bit 9 SOFM: Start of frame interrupt mask
  Bit 8 ESOFM: Expected start of frame interrupt mask
  Bits 7:5 Reserved.
  Bit 4 RESUME: Resume request
  Bit 3 FSUSP: Force suspend
  Bit 2 LP_MODE: Low-power mode
  Bit 1 PDWN: Power down
  Bit 0 FRES: Force USB Reset
*/

#define USB_CNTR_FRES                   ((uint16_t)0x0001)
#define USB_CNTR_PDWN                   ((uint16_t)0x0002)
#define USB_CNTR_LP_MODE                ((uint16_t)0x0004)
#define USB_CNTR_FSUSP                  ((uint16_t)0x0008)
#define USB_CNTR_RESUME                 ((uint16_t)0x0010)
#define USB_CNTR_ESOFM                  ((uint16_t)0x0100)
#define USB_CNTR_SOFM                   ((uint16_t)0x0200)
#define USB_CNTR_RESETM                 ((uint16_t)0x0400)
#define USB_CNTR_SUSPM                  ((uint16_t)0x0800)
#define USB_CNTR_WKUPM                  ((uint16_t)0x1000)
#define USB_CNTR_ERRM                   ((uint16_t)0x2000)
#define USB_CNTR_PMAOVRM                ((uint16_t)0x4000)
#define USB_CNTR_CTRM                   ((uint16_t)0x8000)

/*
  USB interrupt status register (USB_ISTR)
  Bit 15 CTR: Correct transfer
  Bit 14 PMAOVR: Packet memory area over / underrun
  Bit 13 ERR: Error
  Bit 12 WKUP: Wakeup
  Bit 11 SUSP: Suspend mode request
  Bit 10 RESET: USB reset request
  Bit 9 SOF: Start of frame
  Bit 8 ESOF: Expected start of frame
  Bits 7:5 Reserved.
  Bit 4 DIR: Direction of transaction
  Bits 3:0 EP_ID[3:0]: Endpoint Identifier
*/
#define USB_ISTR_EP_ID                  ((uint16_t)0x000f)
#define USB_ISTR_DIR                    ((uint16_t)0x0010)
#define USB_ISTR_ESOF                   ((uint16_t)0x0100)
#define USB_ISTR_SOF                    ((uint16_t)0x0200)
#define USB_ISTR_RESET                  ((uint16_t)0x0400)
#define USB_ISTR_SUSP                   ((uint16_t)0x0800)
#define USB_ISTR_WKUP                   ((uint16_t)0x1000)
#define USB_ISTR_ERR                    ((uint16_t)0x2000)
#define USB_ISTR_PMAOVR                 ((uint16_t)0x4000)
#define USB_ISTR_CTR                    ((uint16_t)0x8000)

#define USB_FNR_FN                      ((uint16_t)0x07ff)
#define USB_FNR_LSOF                    ((uint16_t)0x1800)
#define USB_FNR_LCK                     ((uint16_t)0x2000)
#define USB_FNR_RXDM                    ((uint16_t)0x4000)
#define USB_FNR_RXDP                    ((uint16_t)0x8000)

#define USB_DADDR_ADD                   ((uint8_t)0x7f)   
#define USB_DADDR_ADD0                  ((uint8_t)0x01)   
#define USB_DADDR_ADD1                  ((uint8_t)0x02)   
#define USB_DADDR_ADD2                  ((uint8_t)0x04)   
#define USB_DADDR_ADD3                  ((uint8_t)0x08)   
#define USB_DADDR_ADD4                  ((uint8_t)0x10)   
#define USB_DADDR_ADD5                  ((uint8_t)0x20)   
#define USB_DADDR_ADD6                  ((uint8_t)0x40)   

#define USB_DADDR_EF                    ((uint8_t)0x80)   

#define USB_BTABLE_BTABLE               ((uint16_t)0xfff8)

#define USB_COUNT0_RX_COUNT0_RX         ((uint16_t)0x03ff)
#define USB_COUNT0_RX_NUM_BLOCK         ((uint16_t)0x7C00)
#define USB_COUNT0_RX_BLSIZE            ((uint16_t)0x8000)

#endif
