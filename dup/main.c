/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "shell.h"
#include <xtimer.h>
#include "msg.h"
#include <at30ts74.h>
#include <mma7660.h>
#include "thread.h"
#include <string.h>
#include <periph/gpio.h>
#include "net/gnrc/tftp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/ipv6.h"


#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
extern void handle_input_line(const shell_command_t *command_list, char *line);
#define PACKET_DESTINATION "2001:470:1f04:10::2"
#define DESTINATION_PORT 4042
#define INTERVAL 5000000
void comms(void)
{
  uint32_t last_wakeup = xtimer_now();
  while(1)
  {
      xtimer_usleep_until(&last_wakeup, INTERVAL);
      uint8_t port[2];
      ipv6_addr_t addr;

      /* parse destination address */
      if (ipv6_addr_from_str(&addr, PACKET_DESTINATION) == NULL) {
          puts("Error: unable to parse destination address");
          break;
      }
      port[0] = (uint8_t)DESTINATION_PORT;
      port[1] = DESTINATION_PORT >> 8;

      gnrc_pktsnip_t *payload, *udp, *ip;

      char buf[80];
      sprintf(buf, "hello world %u\n", (unsigned int) xtimer_now());
      /* allocate payload */
      payload = gnrc_pktbuf_add(NULL, &buf[0], strlen(buf), GNRC_NETTYPE_UNDEF);
      if (payload == NULL) {
          puts("Error: unable to copy data to packet buffer");
          break;
      }
      /* allocate UDP header, set source port := destination port */
      udp = gnrc_udp_hdr_build(payload, port, 2, port, 2);
      if (udp == NULL) {
          puts("Error: unable to allocate UDP header");
          gnrc_pktbuf_release(payload);
          break;
      }
      /* allocate IPv6 header */
      ip = gnrc_ipv6_hdr_build(udp, NULL, 0, (uint8_t *)&addr, sizeof(addr));
      if (ip == NULL) {
          puts("Error: unable to allocate IPv6 header");
          gnrc_pktbuf_release(udp);
          break;
      }
      /* send packet */
      if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
          puts("Error: unable to locate UDP thread");
          gnrc_pktbuf_release(ip);
          break;
      }
  }
}
void sh(char *c)
{
  char lbuf[100];
  strcpy(lbuf, c);
  strcat(lbuf,"\n");
  handle_input_line(NULL, lbuf);
}
int main(void)
{
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    sh("ifconfig 7 set chan 25");
    sh("rpl init 7");
    comms();

    /* should be never reached */
    return 0;
}
