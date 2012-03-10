#include <cdc-acm.h>
#include <cdc.h>
#include <usb-api.h>
#include <usb-core.h>
#include <stdio.h>
#include <string.h>
#include "slip.h"


#define DATA_IN 0x81
#define DATA_OUT 0x02
#define INTERRUPT_IN 0x83

PROCESS(usb_acm_process, "USB serial");

static uint8_t usb_ctrl_data_buffer[32];

static USBBuffer recv_buffer;
static uint8_t recv_data[128];

static USBBuffer xmit_buffer[3];
static uint8_t xmit_data[32];

int carrier = 0;

static void
init_recv_buffer()
{
  recv_buffer.next = NULL;
  recv_buffer.data = recv_data;
  recv_buffer.left = sizeof(recv_data);
  recv_buffer.flags = USB_BUFFER_SHORT_END | USB_BUFFER_NOTIFY;
}

void
usb_cdc_acm_carrier(int on)
{
  carrier = on;
}

static void 
cdc_write(char *buf, unsigned int len)
{
  if (len > sizeof(xmit_data)) {
    return;
  }
  //printf("w %u\n", len);
  while (usb_send_pending(DATA_IN))
    ;
  
  memcpy(xmit_data, buf, len);
  xmit_buffer[0].next = NULL;
  xmit_buffer[0].data = xmit_data;
  xmit_buffer[0].left = len;
  xmit_buffer[0].flags = USB_BUFFER_SHORT_END;
  usb_submit_xmit_buffer(DATA_IN, &xmit_buffer[0]);

}

void
slip_arch_writeb(unsigned char b)
{
  if (carrier)
    cdc_write(&b, 1);
}


/**
 */
void
slip_arch_init(unsigned long ubr)
{
  (void)ubr;
  usb_setup();
  usb_cdc_acm_setup();
  process_start(&usb_acm_process, NULL);
}


PROCESS_THREAD(usb_acm_process, ev , data)
{
  PROCESS_BEGIN();
  printf("Starting slip for u101\n");
  usb_set_ep_event_process(DATA_OUT, process_current);
  usb_set_global_event_process(process_current);
  
  while(1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_EXIT) break;
    if (ev == PROCESS_EVENT_POLL) {
      unsigned int events = usb_get_global_events();
      if (events) {
        if (events & USB_EVENT_CONFIG) {
          if (usb_get_current_configuration() != 0) {
            printf("Configured\n");
            usb_setup_bulk_endpoint(DATA_IN);
            usb_setup_bulk_endpoint(DATA_OUT);
            usb_setup_interrupt_endpoint(INTERRUPT_IN);
            init_recv_buffer();
            usb_submit_recv_buffer(DATA_OUT, &recv_buffer);
          } else {
            usb_disable_endpoint(DATA_IN);
            usb_disable_endpoint(DATA_OUT);
            usb_disable_endpoint(INTERRUPT_IN);
          }
        }
      }
      events = usb_get_ep_events(DATA_OUT);
      /* printf("Ev: %ld\n", events); */
      if (events & USB_EP_EVENT_NOTIFICATION) {
        int len;
        int i;

        len = sizeof(recv_data) - recv_buffer.left;
        //printf("r %u\n", len);

        for (i=0; i<len; i++) {
          int r;
          r = slip_input_byte((unsigned char)recv_data[i]);
          if (0 != r) {
            printf("s1\n");
          }
        }

        init_recv_buffer();
        usb_submit_recv_buffer(DATA_OUT, &recv_buffer);
      }

    }
  }
  PROCESS_END();
}

