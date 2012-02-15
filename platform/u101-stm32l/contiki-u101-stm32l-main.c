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

unsigned int idle_count = 0;

int
main()
{
  dbg_setup_uart();
  printf("Initialising %s\n", CONTIKI_VERSION_STRING);
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




