#include <debug-uart.h>
#include <string.h>
#include <stm32f10x_map.h>
#include <stm32f10x_dma.h>
#include <gpio.h>
#include <nvic.h>
#include "stm32-usart.h"
#include "stm32-clk.h"
#include "stm32-nvic.h"
#include "contiki-conf.h"

/* UART number */
#ifndef DBG_UART_NUM
#define DBG_UART_NUM 1
#endif

/* UART ports and pins */
#ifndef DBG_UART_TXPORT
#define DBG_UART_TXPORT B
#endif
#ifndef DBG_UART_TXPIN
#define DBG_UART_TXPIN 6
#endif
#ifndef DBG_UART_RXPORT
#define DBG_UART_RXPORT B
#endif
#ifndef DBG_UART_RXPIN
#define DBG_UART_RXPIN 7
#endif

/* Remap: 1, no remap: 0 */
#ifndef DBG_UART_REMAP
#define DBG_UART_REMAP 1
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

#define _DMA_CHANNEL_NAME(x,c) DMA ## x ## _Channel ## c
#define DMA_CHANNEL_NAME(x,c) _DMA_CHANNEL_NAME(x,c)
#define DBG_DMA_CHANNEL  DMA_CHANNEL_NAME(DBG_DMA_NO, DBG_DMA_CHANNEL_NO)

#define _DBG_DMA_CHANNEL_IFCR_CGIF(c) DMA_IFCR_CGIF ## c
#define _XDBG_DMA_CHANNEL_IFCR_CGIF(c) _DBG_DMA_CHANNEL_IFCR_CGIF(c)
#define DBG_DMA_CHANNEL_IFCR_CGIF \
_XDBG_DMA_CHANNEL_IFCR_CGIF(DBG_DMA_CHANNEL_NO)

#ifndef DBG_XMIT_BUFFER_LEN
#define DBG_XMIT_BUFFER_LEN 2048
#endif

static unsigned char xmit_buffer[DBG_XMIT_BUFFER_LEN];
#define XMIT_BUFFER_END &xmit_buffer[DBG_XMIT_BUFFER_LEN]

void
dbg_setup_uart_default(int baudrate)
{
  stm32_clk_pclk_enable(DBG_UART);

  /* Enable GPIO clocks. */
  RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN |
		   RCC_APB2ENR_IOPAEN |
		   RCC_APB2ENR_IOPBEN);
#if (DBG_UART_NUM) == 1
#if (DBG_UART_REMAP)
  AFIO_REMAP(AFIO_MAPR_USART1_REMAP, AFIO_MAPR_USART1_REMAP);
#endif
#elif (DBG_UART_NUM) == 2
#if (DBG_UART_REMAP)
  AFIO_REMAP(AFIO_MAPR_USART2_REMAP, AFIO_MAPR_USART2_REMAP);
#endif
#elif (DBG_UART_NUM) == 3
#if (DBG_UART_REMAP)
  AFIO_REMAP(AFIO_MAPR_USART3_REMAP, AFIO_MAPR_USART3_REMAP);
#endif
#endif

  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

#define DBG_UART_SET_TXPORT(port,pin,pp,n) GPIO_CONF_OUTPUT_PORT(port,pin,pp,n)
#define DBG_UART_SET_RXPORT(port,pin,f) GPIO_CONF_INPUT_PORT(port,pin,f)

  DBG_UART_SET_TXPORT( DBG_UART_TXPORT, DBG_UART_TXPIN, ALT_PUSH_PULL, 50);
  DBG_UART_SET_RXPORT( DBG_UART_RXPORT, DBG_UART_RXPIN, FLOATING);

#undef DBG_UART_SET_TXPORT
#undef DBG_UART_SET_RXPORT

  DBG_UART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  DBG_UART->CR2 = 0;
  DBG_UART->CR3 = USART_CR3_DMAT;
  DBG_UART->BRR = stm32_clk_frequency(stm32_clk_clkof(DBG_UART)) / baudrate;
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
void DMA1_Channel4_handler() __attribute__((interrupt));
void DMA2_Channel4_5_handler() __attribute__((interrupt));


static void
update_dma(void)
{
  if (xmit_buffer_tail == xmit_buffer_head) return;
  DBG_DMA_CHANNEL->CCR = (DMA_Priority_Low |
			  DMA_PeripheralDataSize_Byte |
			  DMA_MemoryDataSize_Byte |
			  DMA_PeripheralInc_Disable |
			  DMA_MemoryInc_Enable |
			  DMA_Mode_Normal |
			  DMA_DIR_PeripheralDST |
			  DMA_CCR4_TCIE
			  );
  DBG_DMA_CHANNEL->CPAR = (u32)&DBG_UART->DR;
  DBG_DMA_CHANNEL->CMAR = (u32)xmit_buffer_head;
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

#if DBG_UART_NUM == 1
void
DMA1_Channel4_handler()
{
  DBG_DMA->IFCR = DBG_DMA_CHANNEL_IFCR_CGIF;
  xmit_buffer_head = dma_end;
  if (xmit_buffer_tail == xmit_buffer_head) {
    dma_running = 0;
    return;
  }
  update_dma();
}
#endif

#if DBG_UART_NUM == 2
void
DMA1_Channel7_handler()
{
  DBG_DMA->IFCR = DBG_DMA_CHANNEL_IFCR_CGIF;
  xmit_buffer_head = dma_end;
  if (xmit_buffer_tail == xmit_buffer_head) {
    dma_running = 0;
    return;
  }
  update_dma();
}
#endif

#if DBG_UART_NUM == 3
void
DMA1_Channel2_handler()
{
  DBG_DMA->IFCR = DBG_DMA_CHANNEL_IFCR_CGIF;
  xmit_buffer_head = dma_end;
  if (xmit_buffer_tail == xmit_buffer_head) {
    dma_running = 0;
    return;
  }
  update_dma();
}
#endif

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
    while (dbg_send_bytes((const unsigned char*)"^",1) != 1);
  }
  dbg_write_overrun = 0;
  while (dbg_send_bytes((const unsigned char*)&ch,1) != 1);
}

void
dbg_drain()
{
  while(xmit_buffer_tail != xmit_buffer_head);
}

int 
dbg_getchar(unsigned char *c)
{
  if (DBG_UART->SR & USART_SR_RXNE) {
    *c = DBG_UART->DR & 0xFF;
    return 1;
  }
  return 0;
}
