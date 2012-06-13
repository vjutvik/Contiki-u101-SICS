

/**
 * \file
 *         stm32l-specific rtimer code
 * \author
 *         Erik Jansson
 */

#include <signal.h>
#include <stdio.h>
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "stm32-tim.h"
#include "stm32-clk.h"
#include "stm32-clk-arch.h"
#include "stm32-nvic.h"
#include "nvic.h"


/* We use the TIM2 peripheral as RTIMER */
#define RTIMER_NUMBER 4

/* Some preprocessor stupidity to give us... */
#define CONCAT(s1, s2) s1 ## s2
#define CONCAT3(s1, s2, s3) s1 ## s2 ## s3
#define _RTIMER_PERIPH(n) CONCAT(TIM, n)
/* ...these: */

#define RTIMER_PERIPH _RTIMER_PERIPH(RTIMER_NUMBER)

#define RTIMER_IRQ_HANDLER(n) CONCAT3(TIM, n, _IRQhandler)
#define RTIMER_IRQ_CHANNEL(n) CONCAT3(TIM, n, _IRQChannel)


/* IRQ handler */
void RTIMER_IRQ_HANDLER(RTIMER_NUMBER)(void)
{
  uint32_t sr;
  sr = RTIMER_PERIPH->SR;

  if (sr & TIM_SR_CC1OF) {
    /* printf("Timer counter match\n"); */
    /* Clear capture/compare interrupt flag */
    RTIMER_PERIPH->SR &= ~(TIM_SR_CC1OF);
    /* Disable capture/compare interrupt */
    RTIMER_PERIPH->DIER &= ~(TIM_DIER_CC1IE);
  }
  if (sr & TIM_SR_UIF) {
    /* printf("Timer reload\n"); */
    /* Clear update interrupt flag */
    RTIMER_PERIPH->SR &= ~(TIM_SR_UIF);
  }
  return;
}

/* Need to use EXTI for Compare events... */

void
rtimer_arch_init(void)
{
  stm32_clk *pclk;
  uint32_t pfreq;

  /* The stm32_clk and the frequency of the timer peripheral we use */
  pclk = stm32_clk_clkof(RTIMER_PERIPH);
  pfreq = stm32_clk_frequency(pclk);

  /* See clock tree in RM0038 document - the clock input to the timer
     is multiplied by two if the PCLK prescaler is not 1. */
  if (stm32_clk_frequency(&ahb_clk) != pfreq) {
    pfreq <<= 1;
  }

  /* Enable timer peripheral */
  stm32_clk_pclk_enable(RTIMER_PERIPH);
  stm32_clk_pclk_enable(GPIOB);

  /* Initiate timer */

  /* Disable */
  RTIMER_PERIPH->CR1 &= ~(TIM_CR1_CEN);
  RTIMER_PERIPH->CCER &= ~(TIM_CCER_CC1E);
  /* Counter to zero */
  RTIMER_PERIPH->CNT = 0;
  /* Set up prescaler so we end up with desired frequency */
  RTIMER_PERIPH->PSC = pfreq / RTIMER_ARCH_SECOND;
  /* Toggle output when a match is detected */
  RTIMER_PERIPH->CCMR1 = (TIM_CCMR1_OC1M0 | TIM_CCMR1_OC1M1);
  /* Auto-reload - "On upcounting mode, the counter counts from 0 to
     the auto-reload value (content of the TIMx_ARR register), then
     restarts from 0 and generates a counter overflow event." */
  RTIMER_PERIPH->ARR = 0xffff;
  /* Update Interrupt Enable -> produce interrupt when reloaded */
  RTIMER_PERIPH->DIER = 0; /* TIM_DIER_UIE; */
  /* Count upwards */
  RTIMER_PERIPH->CR1 |= TIM_CR1_DIR_UP;
  /* Capture/compare enable, channel 1 */
  RTIMER_PERIPH->CCER = TIM_CCER_CC1E;
  /* Enable */
  RTIMER_PERIPH->CR1 |= TIM_CR1_CEN;
  /* Enable interrupt */
  NVIC_ENABLE_INT(RTIMER_IRQ_CHANNEL(RTIMER_NUMBER));

  printf("Rtimer init done\n");
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
  /* printf("schedule %d\n", t); */
  /* Se compare register */
  RTIMER_PERIPH->CCR1 = (uint16_t)t;
  /* Enable interrupt */
  RTIMER_PERIPH->DIER |= TIM_DIER_CC1IE;
}

rtimer_clock_t rtimer_arch_now(void)
{
  /* Just return the counter */
  return (rtimer_clock_t)RTIMER_PERIPH->CNT;
}
