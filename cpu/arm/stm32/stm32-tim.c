#include <stdint.h>
#include <stdio.h>

#include <stm32l-rcc.h>
#include <stm32l-tim.h>
#include <nvic.h>



#if 0
void TIM2_IRQhandler(void) __attribute__((interrupt));
void TIM2_IRQhandler(void)
{
  /* Clear interrupt */
  TIM2->SR &= ~(TIM_SR_UIF);
  printf("TIM!\n");
}

  NVIC_ENABLE_INT(TIM2_IRQChannel);
#endif



void stm32_tim_init(STM32_TIM *tim)
{
  tim->CNT = 1;
  tim->PSC = 400;
  tim->ARR = 20000;
  tim->DIER |= TIM_DIER_UIE;
  
  tim->CR1 |= TIM_CR1_CEN;
}
