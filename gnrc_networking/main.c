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
#include "thread.h"
#include "net/ipv6/addr.h"
#include "net/gnrc/ipv6/netif.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/netopt.h"
#include "net/gnrc/pkt.h"
#include "net/gnrc/pktbuf.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/sixlowpan/netif.h"

extern int _netif_config(int argc, char **argv);
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
extern void handle_input_line(const shell_command_t *command_list, char *line);
extern int udp_cmd(int argc, char **argv);
extern void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay);
static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

int main(void)
{

    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    at30ts74_t tmp;
    at30ts74_init(&tmp, I2C_0, AT30TS74_ADDR, AT30TS74_12BIT);

    char lbuf[100];
    //strcpy(lbuf, "ifconfig 7 set addr_long 00:12:6d:05:01:00\n");
    //handle_input_line(NULL, lbuf);
    strcpy(lbuf, "ifconfig 7 set chan 25\n");
    handle_input_line(NULL, lbuf);
    strcpy(lbuf, "rpl init 7\n");
    handle_input_line(NULL, lbuf);

  //  puts("RIOT network stack example application");
    //char* cf[] = {"ifconfig", "5", "set","chan","24"};
  //  _netif_config(5,cf);
    /* start shell */
    //puts("All up, running the shell now");
    //char line_buf[SHELL_DEFAULT_BUFSIZE];
    //shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    while (1) {
      int res;
      int32_t result;
      result = 555555;
      res = at30ts74_read(&tmp, &result);
      int mai = result/10000;
      int frac = result%10000;
      char buf [50];
      sprintf(buf, "Temperature is %d.%04d C\n", mai, frac);
      xtimer_usleep(1000000);
      //send("ff02::2", "4000", buf, 1, 100000);
      send("2001:470:1f04:10::2", "4041", buf, 1, 100000);
    }
    /* should be never reached */
    return 0;
}
