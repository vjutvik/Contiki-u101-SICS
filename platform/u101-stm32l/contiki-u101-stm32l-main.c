#include <nvic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>

#include <net/netstack.h>
#include <net/mac/frame802154.h>
#include <net/uip.h>
#include <net/uip-ds6.h>

#include "dev/leds.h"
#include "clock.h"
#include "debug-uart.h"
#include "contiki.h"
#include "stm32-clk-arch.h"

#include "stm32-clk.h"
#include "stm32-spi.h"
#include "stm32-i2c.h"
#include "stm32-id.h"
#include "uspi.h"
#include "ui2c.h"
#include "stm32l.h"
#include "rf230bb.h"
#include "crc16.h"
#include "sensors.h"
#include "dev/button-sensor.h"
#include "dev/slip.h"
#include "rtimer.h"

unsigned int idle_count = 0;

const uspi_master spim1 = {
  .bus = (uint32_t)SPI1,
};
const ui2c_master i2cm1 = {
  .bus = (uint32_t)I2C1,
};

SENSORS(&button_sensor);

extern char __heap_start__;
extern char __heap_end__;
extern char __stack_start__;

static void
u101_stm32l_banner(void)
{
  const char *mhz = "MHz";
  uint16_t uid96[6];
  uint16_t pid16;
  uint16_t flash_kb;
  uint16_t device;
  uint16_t revision;
  stm32_id_uid96(uid96);
  pid16 = crc16_data((unsigned char *)uid96, 12, 0);
  stm32_id_chipinfo(&device, &revision);
  stm32_id_flash_size_kb(&flash_kb);

  printf("\n\nInitialising %s\n", CONTIKI_VERSION_STRING);
  printf("Platform u101-stm32l (dev %04x, rev %04x)\n", device, revision);
#if 1
  printf("Flash: %d kB\n", flash_kb);
  printf("SYSCLK:  %lu %s\n", stm32_clk_frequency(&sys_clk), mhz);
  printf("AHBCLK:  %lu %s\n", stm32_clk_frequency(&ahb_clk), mhz);
  printf("APBCLK1: %lu %s\n", stm32_clk_frequency(&apb1_clk), mhz);
  printf("APBCLK2: %lu %s\n", stm32_clk_frequency(&apb2_clk), mhz);
  printf("UID96: %04x %04x %04x %04x %04x %04x\n", 
         uid96[0], uid96[1], uid96[2], uid96[3], uid96[4], uid96[5]);
  printf("PID16: %04x\n", pid16);
  printf("Heap start:  %08lx\n", (uint32_t)&__heap_start__);
  printf("Heap end:    %08lx\n", (uint32_t)&__heap_end__);
  printf("Stack start: %08lx\n", (uint32_t)&__stack_start__);
#endif
}

static void
u101_stm32l_set_address(rimeaddr_t *addr)
{
  uint16_t uid96[6];
  uint16_t pseudoid16;
  uint8_t *uidaddr;
  int i;

  stm32_id_uid96(uid96);
  pseudoid16 = crc16_data((unsigned char *)uid96, 12, 0);

  /* Assume the last 64 bits bits are unique (may very well not be the case) */
  uidaddr = (uint8_t *)&uid96[2];

  /* Set rime addr based on last bits of UID96*/
  for(i=0; i<RIMEADDR_CONF_SIZE; i++) {		
    addr->u8[i] = uidaddr[i];
  }

#if WITH_UIP6
  /* Construct link-local address based on rime address */
  memcpy(&uip_lladdr.addr, addr->u8, sizeof(uip_lladdr.addr));
#endif

  rimeaddr_set_node_addr(addr);

  printf("Rime address ");
  for(i = 0; i < sizeof(addr->u8); i++) {
    printf("%02x%s", addr->u8[i], ((i<sizeof(addr->u8)-1) ? ":" : "\n"));
  }
}

static void
u101_stm32l_print_processes(struct process * const processes[])
{
  printf("Starting:\n");
  while(*processes != NULL) {
    printf("    %s\n", (*processes)->name);
    processes++;
  }
  printf("\n");
}

static void
u101_stm32l_init(void)
{
  int r;
  int temp;
  rimeaddr_t addr;

  leds_init();
  clock_init();
  rtimer_init();
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();

  /* Radio needs SPI */
  uspi_master_init(&spim1);

  /* Init i2c as well */
  ui2c_master_init(&i2cm1);

  stcn75_init();
  r = stcn75_get_temperature_degc(&temp);
  if (0 != r) {
    printf("Couldn't get temp\n");
  } else {
    printf("temp: %d deg C\n", temp);
  }

  u101_stm32l_set_address(&addr);

  /* Init network stuff */
  r = NETSTACK_RADIO.init();
  if (0 != r) {
    printf("Couldn't initialize radio - halting");
    while (1)
      ;
  }

  rf230_set_channel(CHANNEL_802_15_4);
  rf230_set_pan_addr(IEEE802154_PANID, 0, (uint8_t *)&addr.u8);

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

    /* Flush USART output */
  dbg_drain();

  printf("Starting tcpip\n");
  process_start(&tcpip_process, NULL);

  printf("Tentative link-local IPv6 address ");
  {
    int i, a;
    for(a = 0; a < UIP_DS6_ADDR_NB; a++) {
      if (uip_ds6_if.addr_list[a].isused) {
        for(i = 0; i < 7; ++i) {
          printf("%02x%02x:",
                 uip_ds6_if.addr_list[a].ipaddr.u8[i * 2],
                 uip_ds6_if.addr_list[a].ipaddr.u8[i * 2 + 1]);
        }
        printf("%02x%02x\n",
               uip_ds6_if.addr_list[a].ipaddr.u8[14],
               uip_ds6_if.addr_list[a].ipaddr.u8[15]);
      }
    }
  }
  if(1) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }

  NETSTACK_MAC.on();
  NETSTACK_RADIO.on();

  u101_stm32l_print_processes(autostart_processes);
  autostart_start(autostart_processes);
  printf("Processes running\n");
}

extern struct process *process_list;

int
main()
{
  /* We probably want the debug uart at all times and we want it early */
  dbg_setup_uart_default(115200);

  /* Output some information initially */
  u101_stm32l_banner();

  /* Initialize everything */
  u101_stm32l_init();

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
