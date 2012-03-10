#include <stdint.h>
#include <stdio.h>
#include <gpio.h>
#include <uspi.h>
#include <stm32-nvic.h>
#include <nvic.h>
#include "rf230hal.h"
#include "u101-stm32f.h"

hal_irq_callback_t hal_irq_callback = 0;

/* Which bus is the radio connected to, 1 or 2. 
   Bus 2 is the one closest to the CPU. */
#define RF230_BUS 1

#if (RF230_BUS == 1)
#define RF230_SPI_BUS SPI_BUS_1
#define RADIO_IRQ_PIN 11 /* PORT C */
#else
#error "Dont use this right now."
#define RF230_SPI_BUS SPI_PORT 2
#define RADIO_IRQ_PIN 9 /* PORT C */
#endif

static void spi_radio_cs(int hi) {
  volatile int dummy;
  int i;
#if (RF230_BUS == 1)
  if (hi) {

    GPIOA->BSRR = 1<<4;
  } else {
    GPIOA->BRR = 1<<4;

  }
  if (!hi) {
    for (i=0; i<10; i++) {
      dummy = i;
    }
  }
#else
#endif
}

inline void hal_set_slptr_low()
{
#if (RF230_BUS == 1)
  GPIOA->BRR = 1<<10;
#else
#endif
}

inline void hal_set_slptr_high()
{
#if (RF230_BUS == 1)
  GPIOA->BSRR = 1<<10;
#else
#endif
}

inline uint8_t hal_get_slptr()
{
#if (RF230_BUS == 1)
  return GPIOA->ODR & 1<<10;
#else
#endif
}

inline void hal_set_rst_low()
{
#if (RF230_BUS == 1)
  GPIOA->BRR = 1<<9;
#else
#endif
}

inline void hal_set_rst_high()
{
#if (RF230_BUS == 1)
  GPIOA->BSRR = 1<<9;
#else
#endif
}

const uspi_device rf230_spi = {
  .master = &spim1,
  .cs = spi_radio_cs,
  .rate = 8000 * 1000,
  .mode = 0,
  .name = "rf230"
};

void hal_init(void)
{
  int i;


  /* Configure I/O */
#if (RF230_BUS == 1)
  /* Enable clock to peripheral IOPA */
  RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN |
                   RCC_APB2ENR_IOPAEN |
                   RCC_APB2ENR_IOPCEN);

  /* CS */
  GPIO_CONF_OUTPUT_PORT(A, 4, PUSH_PULL, 50);
  /* CLOCK */
  GPIO_CONF_OUTPUT_PORT(A, 5, ALT_PUSH_PULL, 50);
  /* MISO */
  GPIO_CONF_OUTPUT_PORT(A, 6, ALT_PUSH_PULL, 50);
  /* MOSI */
  GPIO_CONF_OUTPUT_PORT(A, 7, ALT_PUSH_PULL, 50);
  /* Reset */
  GPIO_CONF_OUTPUT_PORT(A, 9, PUSH_PULL, 50);
  /* Sleep */
  GPIO_CONF_OUTPUT_PORT(A, 10, PUSH_PULL, 50);

  /* Set up interrupt pin from radio */

  /* PC */
  AFIO->EXTICR[2] |= 0x00002000;

  /* Configure IRQ pin as input */
  GPIO_CONF_INPUT_PORT(C, 11, PU_PD);
  GPIOC->ODR &= ~(GPIOC->ODR & 1 << 11);

  /* Unmask interrupt for interrupt line */
  EXTI->IMR |= (1 << RADIO_IRQ_PIN);
  /* Unmask event for interrupt line */ 
  EXTI->EMR |= (1 << RADIO_IRQ_PIN);

  /* Rising edge trigger  */
  EXTI->RTSR |= (1 << RADIO_IRQ_PIN);
  /* NOT falling edge trigger  */
  EXTI->FTSR &= ~(1 << RADIO_IRQ_PIN);

  printf("Enabling interrupts\n");
  /* Clear any pending interrupts */
  (void)EXTI->PR;
  NVIC->ISER[1] |= (1 << (EXTI15_10_IRQChannel & 0x1F));
  NVIC_ENABLE_INT(EXTI15_10_IRQChannel);
  NVIC_SET_PRIORITY(EXTI15_10_IRQChannel, 4);
#endif

  /* Assert reset */
  hal_set_rst_low();
  /* slptr */
  hal_set_slptr_low();

  /* Init SPI */
  uspi_device_init(&rf230_spi);

  /* No chip select */
  uspi_cs_high(&rf230_spi);
  /* At 32 MHz, we only need about 20 instructions to cover the
     minimum reset pulse length of 625 ns. A short delay here is overkill
     and not really necessary but at least it's documented now.
   */
  for(i=0; i<50; i++) {
    volatile uint32_t dummy; (void)dummy;
  }

  /* De-assert reset */
  hal_set_rst_high();
}



uint8_t hal_register_read(uint8_t address)
{
  uint8_t data;

  address |= 0x80;

  spi_radio_cs(0);

  (void)uspi_txrx(&rf230_spi, address);
  data = uspi_txrx(&rf230_spi, 0x0);

  spi_radio_cs(1);

  return data;
}


void hal_register_write(uint8_t address, uint8_t value)
{
    address |= 0xc0;

    spi_radio_cs(0);

    (void)uspi_txrx(&rf230_spi, address);
    (void)uspi_txrx(&rf230_spi, value);

    spi_radio_cs(1);
}


uint8_t hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
    /* Read current register value and mask out subregister. */
    uint8_t register_value = hal_register_read(address);
    register_value &= mask;
    register_value >>= position; /* Align subregister value. */

    return register_value;
}

void hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
			   uint8_t value)
{
    /* Read current register value and mask area outside the subregister. */
    volatile uint8_t register_value = hal_register_read(address);
    register_value &= ~mask;

    /* Start preparing the new subregister value. shift in place and mask. */
    value <<= position;
    value &= mask;

    value |= register_value; /* Set the new subregister value. */

    /* Write the modified register value. */
    hal_register_write(address, value);
}

void hal_frame_write(uint8_t *write_buffer, uint8_t length)
{
  uint8_t tmp;
  length &= HAL_TRX_CMD_RADDRM; /* Truncate length to maximum frame length. */
  
  spi_radio_cs(0);
  
  /* Frame write command */
  (void)uspi_txrx(&rf230_spi, HAL_TRX_CMD_FW);
  /* Length */
  (void)uspi_txrx(&rf230_spi, length);
  
  /* Download to the Frame Buffer. */
  do {
    tmp = *write_buffer++;
    (void)uspi_txrx(&rf230_spi, tmp);
    --length;
  } while (length > 0);
  
  spi_radio_cs(1);
}

void hal_frame_read(hal_rx_frame_t *rx_frame)
{
  uint8_t phy_status;
  uint8_t frame_length;
  uint8_t *rx_data;

  spi_radio_cs(0);

  phy_status = uspi_txrx(&rf230_spi, HAL_TRX_CMD_FR);
  frame_length = uspi_txrx(&rf230_spi, 0);
  /*Check for correct frame length.*/
  if ((frame_length >= HAL_MIN_FRAME_LENGTH) && 
      (frame_length <= HAL_MAX_FRAME_LENGTH)){

    rx_data = (rx_frame->data);
    rx_frame->length = frame_length;

    do {
      *rx_data++ = uspi_txrx(&rf230_spi, 0);
    } while (--frame_length > 0);

    rx_frame->lqi = uspi_txrx(&rf230_spi, 0);

    rx_frame->crc = 1;

  } else {
    rx_frame->length = 0;
    rx_frame->lqi    = 0;
    rx_frame->crc    = 0;
  }
  spi_radio_cs(1);

}

void delay_us(volatile int i)
{
  for (; i > 0; i--) {          /* Needs fixing XXX */
    volatile unsigned j;
    for (j = 20; j > 0; j--)
      asm ("nop");
  }
}

void hal_disable_int(void)
{
  NVIC_DISABLE_INT(EXTI15_10_IRQChannel);
}

void hal_enable_int(void)
{
  NVIC_ENABLE_INT(EXTI15_10_IRQChannel);
}

void EXTI9_5_handler()
{
  uint32_t pending;

  pending = EXTI->PR;
  /* printf("(%x) ", pending); */
  if (pending & (1 << RADIO_IRQ_PIN)) {
    EXTI->PR |= (1 << RADIO_IRQ_PIN);
    /* Call interrupt handler */
    if (hal_irq_callback) {
      (void)hal_irq_callback();
    }
      
  } else {
    /* Badness, we don't do interrupt sharing */
    /*printf("Other device is using this interrupt line\n"); */
  }

}

void EXTI15_10_handler(void) __attribute__((interrupt));
void EXTI15_10_handler(void)
{
  uint32_t pending;
  pending = EXTI->PR;

  /* printf("(%lx) ", pending); */
  if (pending & (1 << RADIO_IRQ_PIN)) {
    EXTI->PR |= (1 << RADIO_IRQ_PIN);
    /* Call interrupt handler */
    if (hal_irq_callback) {
      (void)hal_irq_callback();
    }
      
  } else {
    /* Badness, we don't do interrupt sharing */
    /*printf("Other device is using this interrupt line\n"); */
  }
  
}
