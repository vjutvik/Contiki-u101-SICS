#include <stm32f10x_map.h>
#include <stm32f10x_dma.h>
#include <gpio.h>
#include <nvic.h>
#include <stdint.h>
#include <stdio.h>
#include <debug-uart.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>
#include "stm32-clk-arch.c"
#include "contiki.h"

unsigned int idle_count = 0;

int
main()
{
  dbg_setup_uart(115200);
  printf("Initialising %s\n", CONTIKI_VERSION_STRING);
  printf("STM32F\n");
  printf("SYSCLK: %ld\n", stm32_clk_frequency(sys_clk));
  printf("AHBCLK: %ld\n", stm32_clk_frequency(ahb_clk));
  printf("APBCLK1: %ld\n", stm32_clk_frequency(apb1_clk));
  printf("APBCLK2: %ld\n", stm32_clk_frequency(apb2_clk));
  clock_init();
  process_init();
  process_start(&etimer_process, NULL);
  autostart_start(autostart_processes);
  printf("Processes running\n");
  while(1) {
    do {
    } while(process_run() > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
  }
  return 0;
}




