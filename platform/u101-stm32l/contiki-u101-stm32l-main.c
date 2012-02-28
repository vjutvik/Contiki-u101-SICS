#include <nvic.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>

#include <net/netstack.h>
#include <net/mac/frame802154.h>
#include <net/uip.h>

#include "clock.h"
#include "debug-uart.h"
#include "contiki.h"
#include "stm32-clk-arch.h"

#include "stm32-clk.h"
#include "stm32-spi.h"
#include "uspi.h"
#include "stm32l.h"
#include "rf230bb.h"

unsigned int idle_count = 0;

const uspi_master spim1 = {
  .bus = (uint32_t)SPI1,
};

static void
banner(void)
{
  const char *mhz = "MHz";
  printf("\n\nInitialising %s\n", CONTIKI_VERSION_STRING);
  printf("STM32L\n");
  printf("SYSCLK:  %lu %s\n", stm32_clk_frequency(sys_clk), mhz);
  printf("AHBCLK:  %lu %s\n", stm32_clk_frequency(ahb_clk), mhz);
  printf("APBCLK1: %lu %s\n", stm32_clk_frequency(apb1_clk), mhz);
  printf("APBCLK2: %lu %s\n", stm32_clk_frequency(apb2_clk), mhz);
}

static void
u101_init(void)
{
  int r;
  rimeaddr_t rimeaddr;

  clock_init();

  /* Network needs processes */
  process_init();
  process_start(&etimer_process, NULL);

  /* Network needs SPI */
  uspi_master_init(&spim1);

  /* Init network stuff */
  r = NETSTACK_RADIO.init();
  if (0 != r) {
    printf("Couldn't initialize radio - halting");
    while (1)
      ;
  }

  NETSTACK_MAC.init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.on();
  NETSTACK_RADIO.on();

  rimeaddr.u8[0] = 1;
  rimeaddr.u8[0] = 3;

  rf230_set_channel(CHANNEL_802_15_4);

  rf230_set_pan_addr(IEEE802154_PANID, 0, (uint8_t *)&rimeaddr.u8);
  rimeaddr_set_node_addr(&rimeaddr);
  process_start(&tcpip_process, NULL);

  autostart_start(autostart_processes);
  printf("Processes running\n");
}

extern struct process *process_list;

int
main()
{
  /* We probably want the debug uart at all times */
  dbg_setup_uart(115200);

  /* Output some information initially */
  banner();

  /* Initialize everything */
  u101_init();

  /* The main loop */
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
