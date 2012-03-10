/*
 * Copyright (c) 2012, UPWIS AB, Erik Jansson (erik at upwis.com)
 *
 * Originally from STM32F103 port by KSB
 *
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

#include <string.h>
#include "contiki-conf.h"
#include "nvic.h"
#include "stm32-nvic.h"
#include "stm32l-gpio.h"
#include "debug-uart.h"
#include "stm32l-rcc.h"
#include "stm32-usart.h"
#include "stm32l-dma.h"
#include "stm32-clk.h"

/* UART number */
#ifndef DBG_UART_NUM
#define DBG_UART_NUM 3
#endif

/* UART ports and pins */
#ifndef DBG_UART_TXPORT
#define DBG_UART_TXPORT B
#endif
#ifndef DBG_UART_TXPIN
#define DBG_UART_TXPIN 10
#endif
#ifndef DBG_UART_RXPORT
#define DBG_UART_RXPORT B
#endif
#ifndef DBG_UART_RXPIN
#define DBG_UART_RXPIN 11
#endif

/* Remap: 1, no remap: 0 */
#ifndef DBG_UART_REMAP
#define DBG_UART_REMAP 0
#endif

#define _DBG_UART_NAME(a, num) a ## num
#define DBG_UART_NAME(a, num) _DBG_UART_NAME(a, num)

#ifdef DBG_UART_NUM
#define DBG_UART DBG_UART_NAME(USART, DBG_UART_NUM)
#endif

/* DMA controller for TX is always 1 */
#define DBG_DMA_NO 1

/* DMA request mapping, channels for TX */
#if (DBG_UART_NUM) == (1)
#define DBG_DMA_CHANNEL_NO 4
#elif (DBG_UART_NUM) == (2)
#define DBG_DMA_CHANNEL_NO 7
#elif (DBG_UART_NUM) == (3)
#define DBG_DMA_CHANNEL_NO 2
#else
#error "Debug UART not supported"
#endif

#define _DBG_DMA_NAME(x) DMA##x
#define DBG_DMA_NAME(x) _DBG_DMA_NAME(x)
#define DBG_DMA DBG_DMA_NAME(DBG_DMA_NO)

#define _DMA_CHANNEL_NAME(x,c) DMA ## x ## _CHANNEL ## c
#define DMA_CHANNEL_NAME(x,c) _DMA_CHANNEL_NAME(x,c)
#define DBG_DMA_CHANNEL  DMA_CHANNEL_NAME(DBG_DMA_NO, DBG_DMA_CHANNEL_NO)

#define _DBG_DMA_CHANNEL_IFCR_CGIF(c) DMA_IFCR_CGIF ## c
#define _XDBG_DMA_CHANNEL_IFCR_CGIF(c) _DBG_DMA_CHANNEL_IFCR_CGIF(c)
#define DBG_DMA_CHANNEL_IFCR_CGIF \
_XDBG_DMA_CHANNEL_IFCR_CGIF(DBG_DMA_CHANNEL_NO)

#ifndef DBG_XMIT_BUFFER_LEN
#define DBG_XMIT_BUFFER_LEN 512
#endif

static unsigned char xmit_buffer[DBG_XMIT_BUFFER_LEN];
#define XMIT_BUFFER_END &xmit_buffer[DBG_XMIT_BUFFER_LEN]
int dbg_active = 0;

void
dbg_setup_uart_default(int baudrate)
{
  stm32_clk_pclk_enable(DBG_UART);

#if (DBG_UART_NUM) == (1)
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  stm32l_gpio_conf_af(GPIOA, 9);
  stm32l_gpio_conf_af(GPIOA, 10);
#error "Debug UART1 not yet supported"

#elif (DBG_UART_NUM) == (2)
#error "Debug UART2 not yet supported"

#elif (DBG_UART_NUM) == (3)
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  stm32l_gpio_map_af(GPIOB, 10, GPIO_AF_USART3);
  stm32l_gpio_map_af(GPIOB, 11, GPIO_AF_USART3);
  stm32l_gpio_conf_af(GPIOB, 10, GPIO_OUTPUT_TYPE_PPULL, GPIO_OSPEED_40MHZ, GPIO_RESISTORS_NONE);
  stm32l_gpio_conf_af(GPIOB, 11, GPIO_OUTPUT_TYPE_PPULL, GPIO_OSPEED_40MHZ, GPIO_RESISTORS_NONE);
#else

#error "Selected UART not supported"
#endif

  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  DBG_UART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  DBG_UART->CR2 = 0;
  DBG_UART->CR3 = USART_CR3_DMAT;
  DBG_UART->GTPR = 0x1;
  DBG_UART->BRR = stm32_clk_frequency(stm32_clk_clkof(DBG_UART))/baudrate;
}

/* Valid data in head to tail-1 */
/* Read position */
static unsigned char * volatile xmit_buffer_head = xmit_buffer;

/* Write position */
static unsigned char * volatile xmit_buffer_tail = xmit_buffer;

/* xmit_buffer_head == xmit_buffer_tail means empty so we can only store
   DBG_XMIT_BUFFER_LEN-1 characters */

volatile unsigned char dma_running = 0;
static unsigned char * volatile dma_end;
void DMA1_Channel1_IRQhandler() __attribute__((interrupt));
void DMA1_Channel2_IRQhandler() __attribute__((interrupt));
void DMA1_Channel3_IRQhandler() __attribute__((interrupt));
void DMA1_Channel4_IRQhandler() __attribute__((interrupt));
void DMA2_Channel4_5_IRQhandler() __attribute__((interrupt));
void USART3_IRQhandler() __attribute__((interrupt));

static void
update_dma(void)
{
  if (xmit_buffer_tail == xmit_buffer_head) {
    dbg_active = 0;
    return;
  }
  dbg_active = 1;
  DBG_DMA_CHANNEL->CCR = (DMA_PRIORITY_LOW |
			  DMA_PERIPHERALDATASIZE_BYTE |
			  DMA_MEMORYDATASIZE_BYTE |
			  DMA_PERIPHERALINC_DISABLE |
			  DMA_MEMORYINC_ENABLE |
			  DMA_MODE_NORMAL |
			  DMA_DIR_PERIPHERALDST |
			  DMA_CCR4_TCIE
			  );
  DBG_DMA_CHANNEL->CPAR = (uint32_t)&DBG_UART->DR;
  DBG_DMA_CHANNEL->CMAR = (uint32_t)xmit_buffer_head;
  if (xmit_buffer_head < xmit_buffer_tail) {
    DBG_DMA_CHANNEL->CNDTR = xmit_buffer_tail - xmit_buffer_head;
    dma_end = xmit_buffer_tail;
  } else {
    DBG_DMA_CHANNEL->CNDTR =  XMIT_BUFFER_END - xmit_buffer_head;
    dma_end = xmit_buffer;
  }

#if DBG_UART_NUM == 1
  NVIC_ENABLE_INT(DMA1_Channel4_IRQChannel);
  NVIC_SET_PRIORITY(DMA1_Channel4_IRQChannel, 2);
  DBG_DMA_CHANNEL->CCR |= DMA_CCR4_EN;
#elif DBG_UART_NUM == 2
  NVIC_ENABLE_INT(DMA1_Channel7_IRQChannel);
  NVIC_SET_PRIORITY(DMA1_Channel7_IRQChannel, 2);
  DBG_DMA_CHANNEL->CCR |= DMA_CCR2_EN;
#elif DBG_UART_NUM == 3
  NVIC_ENABLE_INT(DMA1_Channel2_IRQChannel);
  NVIC_SET_PRIORITY(DMA1_Channel2_IRQChannel, 2);
  DBG_DMA_CHANNEL->CCR |= DMA_CCR2_EN;
#else
#error "DGB_DMA is not 1 or 2"
#endif

}

void
DMA1_Channel2_IRQhandler()
{
  DBG_DMA->IFCR = DBG_DMA_CHANNEL_IFCR_CGIF;
  xmit_buffer_head = dma_end;
  if (xmit_buffer_tail == xmit_buffer_head) {
    dma_running = 0;
    dbg_active = 0;
    return;
  }
  update_dma();
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
  /* Since each of the pointers should be read atomically
     there's no need to disable interrupts */
  unsigned char *head = xmit_buffer_head;
  unsigned char *tail = xmit_buffer_tail;
  if (tail >= head) {
    /* Free space wraps */
    unsigned int xfer_len = XMIT_BUFFER_END - tail;
    unsigned int free = DBG_XMIT_BUFFER_LEN - (tail - head) - 1;
    if (len > free) len = free;
    if (xfer_len < len) {
      memcpy(tail, seq, xfer_len);
      seq += xfer_len;
      xfer_len = len - xfer_len;
      memcpy(xmit_buffer, seq, xfer_len);
      tail = xmit_buffer + xfer_len;
    } else {
      memcpy(tail, seq, len);
      tail += len;
      if (tail == XMIT_BUFFER_END) tail = xmit_buffer;
    }
  } else {
    /* Free space continuous */
    unsigned int free = (head - tail) - 1;
    if (len > free) len = free;
    memcpy(tail, seq, len);
    tail += len;
  }
  xmit_buffer_tail = tail;
  if (!dma_running) {
    dma_running = 1;
    update_dma();
  }
  return len;
}

static unsigned char dbg_write_overrun = 0;

void
dbg_putchar(const char ch)
{
  if (dbg_write_overrun) {
    if (dbg_send_bytes((const unsigned char*)"^",1) != 1) return;
  }
  dbg_write_overrun = 0;
  if (dbg_send_bytes((const unsigned char*)&ch,1) != 1) {
    dbg_write_overrun = 1;
  }
}

void
dbg_blocking_putchar(const char ch)
{
  if (dbg_write_overrun) {
    while (dbg_send_bytes((const unsigned char*)"^", 1) != 1);
  }
  dbg_write_overrun = 0;
  while (dbg_send_bytes((const unsigned char*)&ch, 1) != 1);
}

void
dbg_drain()
{
  while(xmit_buffer_tail != xmit_buffer_head);
}

int 
dbg_getbyte(unsigned char *byte)
{
  if (DBG_UART->SR & USART_SR_RXNE) {
    *byte = DBG_UART->DR & 0xFF;
    return 1;
  }
  return 0;
}
