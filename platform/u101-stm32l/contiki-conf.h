#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

#define CCIF
#define CLIF

#define WITH_UIP 1
#define WITH_ASCII 1

#define CLOCK_CONF_SECOND 128
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

#define NETSTACK_CONF_NETWORK     rime_driver
#define RIMEADDR_CONF_SIZE        2

#define NETSTACK_CONF_MAC         nullmac_driver
#define NETSTACK_CONF_RDC         nullrdc_driver
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver
#define CHANNEL_802_15_4          26
#define RF230_CONF_AUTOACK        1
#define RF230_CONF_AUTORETRIES    0

/* uIP configuration */
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_BROADCAST       1
#define UIP_CONF_LOGGING 1
#define UIP_CONF_BUFFER_SIZE 1500

#define UIP_CONF_TCP_FORWARD 1



/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#define USB_EP1_SIZE 64
#define USB_EP2_SIZE 64

#define RAND_MAX 0x7fff

#define DBG_UART_NUM 3
#define DBG_UART_TXPORT B
#define DBG_UART_TXPIN 10
#define DBG_UART_RXPORT B
#define DBG_UART_RXPIN 11
#define DBG_UART_REMAP 0
#endif /* __CONTIKI_CONF_H__ */
