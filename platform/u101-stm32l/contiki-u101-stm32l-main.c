#include <nvic.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include "clock.h"
#include "debug-uart.h"
#include "contiki.h"
#include "stm32-clk-arch.h"
#include "stm32-clk.h"

unsigned int idle_count = 0;

int
main()
{
  dbg_setup_uart(115200);
  printf("Initialising %s\n", CONTIKI_VERSION_STRING);
  printf("STM32L\n");
  printf("SYSCLK:  %lu\n", stm32_clk_frequency(sys_clk));
  printf("AHBCLK:  %lu\n", stm32_clk_frequency(ahb_clk));
  printf("APBCLK1: %lu\n", stm32_clk_frequency(apb1_clk));
  printf("APBCLK2: %lu\n", stm32_clk_frequency(apb2_clk));
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
