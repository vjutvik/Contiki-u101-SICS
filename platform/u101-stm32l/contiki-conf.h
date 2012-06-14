#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

#ifdef PLATFORM_CONF_H
#include PLATFORM_CONF_H
#else
#include "platform-conf.h"
#endif /* PLATFORM_CONF_H */

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

#if WITH_UIP6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK     sicslowpan_driver
#define NETSTACK_CONF_MAC         nullmac_driver 
#define NETSTACK_CONF_RDC         nullrdc_driver
#define NETSTACK_CONF_RADIO       rf230_driver
#define NETSTACK_CONF_FRAMER      framer_802154

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE      8
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0

#define CHANNEL_802_15_4          26

#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */
#define RIMEADDR_CONF_SIZE        2

#define NETSTACK_CONF_NETWORK     rime_driver
#define NETSTACK_CONF_MAC         nullmac_driver
#define NETSTACK_CONF_RDC         nullrdc_driver
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE      8

#define CHANNEL_802_15_4          26
#define RF230_CONF_AUTOACK        1
#define RF230_CONF_AUTORETRIES    0

#endif



#ifdef WITH_UIP6
#define RIMEADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 1  
#define UIP_CONF_IPV6_RPL               1

#define UIP_CONF_DS6_NBR_NBU            5
#define UIP_CONF_DS6_ROUTE_NBU          5

#define UIP_CONF_ND6_SEND_RA            0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IPV6                   1
#define UIP_CONF_IPV6_QUEUE_PKT         0
#define UIP_CONF_IPV6_CHECKS            0
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_ICMP6                  1
#define UIP_CONF_NETIF_MAX_ADDRESSES    2
#define UIP_CONF_ND6_MAX_PREFIXES       2
#define UIP_CONF_ND6_MAX_NEIGHBORS      2
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#define UIP_CONF_BUFFER_SIZE            300
#define SICSLOWPAN_CONF_FRAG            1
#define SICSLOWPAN_CONF_MAXAGE          8

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC        1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2

#else /* WITH_UIP6 */


/* uIP configuration */
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_BROADCAST       1
#define UIP_CONF_LOGGING         1

#define UIP_CONF_TCP_FORWARD     1

#endif /* WITH_UIP6 */


/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#define USB_EP1_SIZE 64
#define USB_EP2_SIZE 64

#define DBG_UART_NUM 3
#define DBG_UART_TXPORT B
#define DBG_UART_TXPIN 10
#define DBG_UART_RXPORT B
#define DBG_UART_RXPIN 11
#define DBG_UART_REMAP 0

#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif

#endif /* __CONTIKI_CONF_H__ */
