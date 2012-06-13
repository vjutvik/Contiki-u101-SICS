#ifndef STM32_TIM_H
#define STM32_TIM_H

#include <stdint.h>

typedef struct {
  volatile uint16_t CR1;
  uint16_t R0;
  volatile uint16_t CR2;
  uint16_t R1;
  volatile uint16_t SMCR;
  uint16_t R2;
  volatile uint16_t DIER;
  uint16_t R3;
  volatile uint16_t SR;
  uint16_t R4;
  volatile uint16_t EGR;
  uint16_t R5;
  volatile uint16_t CCMR1;
  uint16_t R6;
  volatile uint16_t CCMR2;
  uint16_t R7;
  volatile uint16_t CCER;
  uint16_t R8;
  volatile uint16_t CNT;
  uint16_t R9;
  volatile uint16_t PSC;
  uint16_t R10;
  volatile uint16_t ARR;
  uint16_t R11;
  uint32_t R12;
  volatile uint16_t CCR1;
  uint16_t R13;
  volatile uint16_t CCR2;
  uint16_t R14;
  volatile uint16_t CCR3;
  uint16_t R15;
  volatile uint16_t CCR4;
  uint16_t R16;
  uint32_t R17;
  volatile uint16_t DCR;
  uint16_t R18;
  volatile uint16_t DMAR;
  uint16_t R19;
  volatile uint16_t OR;
  uint16_t R20;
} STM32_TIM;

/* Registers and bit definitions */
#define TIM_CR1_CEN                     ((uint16_t)0x0001) 
#define TIM_CR1_UDIS                    ((uint16_t)0x0002) 
#define TIM_CR1_URS                     ((uint16_t)0x0004) 
#define TIM_CR1_OPM                     ((uint16_t)0x0008) 
#define TIM_CR1_DIR                     ((uint16_t)0x0010)
#define TIM_CR1_DIR_UP                  ((uint16_t)0x0000)
#define TIM_CR1_DIR_DOWN                ((uint16_t)0x0010)
#define TIM_CR1_CMS                     ((uint16_t)0x0060) 
#define TIM_CR1_ARPE                    ((uint16_t)0x0080) 
#define TIM_CR1_CKD                     ((uint16_t)0x0300) 

#define TIM_CR2_CCPC                    ((uint16_t)0x0001)
#define TIM_CR2_CCUS                    ((uint16_t)0x0004)
#define TIM_CR2_CCDS                    ((uint16_t)0x0008)
#define TIM_CR2_MMS                     ((uint16_t)0x0070)
#define TIM_CR2_TI1S                    ((uint16_t)0x0080)
#define TIM_CR2_OIS1                    ((uint16_t)0x0100)
#define TIM_CR2_OIS1N                   ((uint16_t)0x0200)
#define TIM_CR2_OIS2                    ((uint16_t)0x0400)
#define TIM_CR2_OIS2N                   ((uint16_t)0x0800)
#define TIM_CR2_OIS3                    ((uint16_t)0x1000)
#define TIM_CR2_OIS3N                   ((uint16_t)0x2000)
#define TIM_CR2_OIS4                    ((uint16_t)0x4000)
                                    
#define TIM_DIER_UIE                    ((uint16_t)0x0001)
#define TIM_DIER_CC1IE                  ((uint16_t)0x0002)
#define TIM_DIER_CC2IE                  ((uint16_t)0x0004)
#define TIM_DIER_CC3IE                  ((uint16_t)0x0008)
#define TIM_DIER_CC4IE                  ((uint16_t)0x0010)
#define TIM_DIER_COMIE                  ((uint16_t)0x0020)
#define TIM_DIER_TIE                    ((uint16_t)0x0040)
#define TIM_DIER_BIE                    ((uint16_t)0x0080)
#define TIM_DIER_UDE                    ((uint16_t)0x0100)
#define TIM_DIER_CC1DE                  ((uint16_t)0x0200)
#define TIM_DIER_CC2DE                  ((uint16_t)0x0400)
#define TIM_DIER_CC3DE                  ((uint16_t)0x0800)
#define TIM_DIER_CC4DE                  ((uint16_t)0x1000)
#define TIM_DIER_COMDE                  ((uint16_t)0x2000)
#define TIM_DIER_TDE                    ((uint16_t)0x4000)

#define TIM_SR_UIF                      ((uint16_t)0x0001)
#define TIM_SR_TIF                      ((uint16_t)0x0040)
#define TIM_SR_CC1IF                    ((uint16_t)0x0002)
#define TIM_SR_CC1OF                    ((uint16_t)0x0200)

#define TIM_CCMR1_CC1S0                 ((uint16_t)0x0001)
#define TIM_CCMR1_CC1S1                 ((uint16_t)0x0002)
#define TIM_CCMR1_OC1FE                 ((uint16_t)0x0004)
#define TIM_CCMR1_OC1PE                 ((uint16_t)0x0008)
#define TIM_CCMR1_OC1M0                 ((uint16_t)0x0010)
#define TIM_CCMR1_OC1M1                 ((uint16_t)0x0020)
#define TIM_CCMR1_OC1M2                 ((uint16_t)0x0040)
#define TIM_CCMR1_OC1CE                 ((uint16_t)0x0080)
#define TIM_CCMR1_CC2S0                 ((uint16_t)0x0100)
#define TIM_CCMR1_CC2S1                 ((uint16_t)0x0200)
#define TIM_CCMR1_OC2FE                 ((uint16_t)0x0400)
#define TIM_CCMR1_OC2PE                 ((uint16_t)0x0800)
#define TIM_CCMR1_OC2M0                 ((uint16_t)0x1000)
#define TIM_CCMR1_OC2M1                 ((uint16_t)0x2000)
#define TIM_CCMR1_OC2M2                 ((uint16_t)0x4000)
#define TIM_CCMR1_OC2CE                 ((uint16_t)0x8000)

#define TIM_CCER_CC1E                   ((uint16_t)0x0001)
#define TIM_CCER_CC1P                   ((uint16_t)0x0002)
#define TIM_CCER_CC1NP                  ((uint16_t)0x0008)
#define TIM_CCER_CC2E                   ((uint16_t)0x0010)
#define TIM_CCER_CC2P                   ((uint16_t)0x0020)
#define TIM_CCER_CC2NP                  ((uint16_t)0x0080)
#define TIM_CCER_CC3E                   ((uint16_t)0x0100)
#define TIM_CCER_CC3P                   ((uint16_t)0x0200)
#define TIM_CCER_CC3NP                  ((uint16_t)0x0800)
#define TIM_CCER_CC4E                   ((uint16_t)0x1000)
#define TIM_CCER_CC4P                   ((uint16_t)0x2000)
#define TIM_CCER_CC4NP                  ((uint16_t)0x8000)


#endif

