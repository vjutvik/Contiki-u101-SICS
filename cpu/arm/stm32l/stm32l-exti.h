#ifndef STM32L_EXTI_H
#define STM32L_EXTI_H

#include <stdint.h>

typedef struct {
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} STM32L_EXTI;

#endif
