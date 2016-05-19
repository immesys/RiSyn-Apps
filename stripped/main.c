/*
 * This program is an example that copies the structure of a real program
 * I have, but does not use the sensors, so can run on a samr21-xpro
 */

#include <stdio.h>
#include "shell.h"
#include <xtimer.h>
#include "msg.h"
#include "thread.h"
#include <string.h>
#include <periph/gpio.h>
#include "net/gnrc/tftp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/ipv6.h"

#define DNS_INTERVAL 30000000
extern int _netif_config(int argc, char **argv);
#define MAIN_QUEUE_SIZE     (8)
#define COMMS_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
static msg_t _comms_msg_queue[COMMS_QUEUE_SIZE];

extern void handle_input_line(const shell_command_t *command_list, char *line);

#define INTERVAL 10000
static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

typedef struct {
  char msg[8];
} __attribute__((packed)) pkt_t;
#define TXBUFSZ 64
#define TXBUFMASK (TXBUFSZ-1)
pkt_t txbuf[TXBUFSZ];
uint8_t wptr;
uint8_t rptr;

char samplethread [2000];
char commsthread [2000];

#define MSG_TYPE_NEW_PACKETS (0xc201)
#define MSG_TYPE_DNS (0xc202)
#define NETWORK_RTT_US 1000000
//#define PACKET_DESTINATION "2607:f140:400:a009:2ca3:9be1:84ed:cac2"
#define PACKET_DESTINATION "2001:470:1f04:5f2::2"
#define DESTINATION_PORT 4041

kernel_pid_t comms_pid;
msg_t dns_msg;
xtimer_t dns_timer;
msg_t dns_msg2;
xtimer_t dns_timer2;
gpio_t occupancy;

void create_dns_record(void)
{
  printf("Sending DNS alias\n");
  char *dst = PACKET_DESTINATION;
  char name[] = "dummy payload for reproducer\n";
  uint8_t port[2];
  ipv6_addr_t addr;

  /* parse destination address */
  if (ipv6_addr_from_str(&addr, dst) == NULL) {
      puts("Error: unable to parse destination address");
      return;
  }
  port[0] = (uint8_t)DESTINATION_PORT;
  port[1] = DESTINATION_PORT >> 8;

  gnrc_pktsnip_t *payload, *udp, *ip;
  /* allocate payload */
  payload = gnrc_pktbuf_add(NULL, &(name[0]), sizeof(name)-1, GNRC_NETTYPE_UNDEF);
  if (payload == NULL) {
      puts("Error: unable to copy data to packet buffer");
      return;
  }
  /* allocate UDP header, set source port := destination port */
  udp = gnrc_udp_hdr_build(payload, port, 2, port, 2);
  if (udp == NULL) {
      puts("Error: unable to allocate UDP header");
      gnrc_pktbuf_release(payload);
      return;
  }
  /* allocate IPv6 header */
  ip = gnrc_ipv6_hdr_build(udp, NULL, 0, (uint8_t *)&addr, sizeof(addr));
  if (ip == NULL) {
      puts("Error: unable to allocate IPv6 header");
      gnrc_pktbuf_release(udp);
      return;
  }
  /* send packet */
  if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
      puts("Error: unable to locate UDP thread");
      gnrc_pktbuf_release(ip);
      return;
  }
}
void *comms(void *arg)
{
  (void) arg;
  msg_t msg;
  msg_init_queue(_comms_msg_queue, COMMS_QUEUE_SIZE);

  gnrc_netreg_entry_t entry = { NULL, 4041, thread_getpid() };

  if (gnrc_netreg_register(GNRC_NETTYPE_UDP, &entry)) {
    puts("ERROR listening\n");
  }

  while(1)
  {
    if (xtimer_msg_receive_timeout(&msg, NETWORK_RTT_US) < 0)
      msg.type = MSG_TYPE_NEW_PACKETS; //default to resending old packet
    switch(msg.type)
    {
      case GNRC_NETAPI_MSG_TYPE_RCV:
      {
        #if 0
          gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)(msg.content.ptr);
          gnrc_pktsnip_t *tmp;
          LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_UDP);
          //udp_hdr_t *udp = (udp_hdr_t *)tmp->data;
          LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_IPV6);
          //ipv6_hdr_t *ip = (ipv6_hdr_t *)tmp->data;
          uint8_t *data = (uint8_t *)pkt->data;

          uint32_t echotag = ((uint32_t)data[0]) |
                             ((uint32_t)data[1] << 8) |
                             ((uint32_t)data[2] << 16) |
                             ((uint32_t)data[3] << 24);
          uint32_t ts = ((uint32_t)data[4]) |
                       ((uint32_t)data[5] << 8) |
                       ((uint32_t)data[6] << 16) |
                       ((uint32_t)data[7] << 24);
          now64 /= 100000; //Ticks are in 0.1S
          xtime_offset = ts - now64;
          if (txbuf[rptr].echotag == echotag) {
            txbuf[rptr].echotag = 0;
            rptr++;
            rptr &= TXBUFMASK;
          }
          gnrc_pktbuf_release((gnrc_pktsnip_t *) msg.content.ptr);
          //Deliberately do not break
          #endif
      }
      case MSG_TYPE_NEW_PACKETS:
      {
          uint8_t port[2];
          ipv6_addr_t addr;

          if (txbuf[rptr].msg[0] != 0)
          {

            /* parse destination address */
            if (ipv6_addr_from_str(&addr, PACKET_DESTINATION) == NULL) {
                puts("Error: unable to parse destination address");
                break;
            }
            port[0] = (uint8_t)DESTINATION_PORT;
            port[1] = DESTINATION_PORT >> 8;

            gnrc_pktsnip_t *payload, *udp, *ip;
            /* allocate payload */
            payload = gnrc_pktbuf_add(NULL, &(txbuf[rptr]), sizeof(pkt_t), GNRC_NETTYPE_UNDEF);
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
            txbuf[rptr].msg[0] = 0;
            rptr++;
            rptr &= TXBUFMASK;
          }
          break;
      }
      case MSG_TYPE_DNS:
      {
        create_dns_record();
        xtimer_set_msg(&dns_timer,DNS_INTERVAL, &dns_msg, thread_getpid());
        break;
      }
    }
  }

}
void *dosample(void *arg)
{
  (void) arg;
  uint32_t last_wakeup = xtimer_now();
  int count = 0;
  while (1) {
    xtimer_usleep_until(&last_wakeup, INTERVAL);
    count ++;
    if (count % 100 == 0) {
      if (((wptr + 1)&TXBUFMASK) != rptr) {
        //We have space
        pkt_t *p = &txbuf[wptr];
        strcpy(&p->msg[0], "hello\n");
        wptr ++;
        wptr &= TXBUFMASK;
        msg_t checkPackets;
        checkPackets.type = MSG_TYPE_NEW_PACKETS;
        msg_send(&checkPackets, comms_pid);
      }
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
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    dns_msg.type = MSG_TYPE_DNS;
    dns_msg2.type = MSG_TYPE_DNS;
    sh("ifconfig 7 set chan 25");
    sh("rpl init 7");
    comms_pid = thread_create(commsthread, sizeof(commsthread),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            comms, NULL, "comms");
    thread_create(samplethread, sizeof(samplethread),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            dosample, NULL, "sampling");
    xtimer_set_msg(&dns_timer2,5000000, &dns_msg2, comms_pid);
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
