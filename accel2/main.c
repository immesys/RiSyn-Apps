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
#define INTERVAL 10000


typedef struct {
  uint32_t echotag;
  uint8_t vx;
  uint8_t vy;
  uint8_t vz;
  uint8_t vax;
  uint8_t vay;
  uint8_t vaz;
  uint8_t temp[4];
  uint8_t tm[4];
  uint8_t occ;
  uint8_t trig;
} __attribute__((packed)) pkt_t;

#define ACCTRIG 0x01
#define OCCTRIG 0x02
#define TXBUFSZ 64
#define TXBUFMASK (TXBUFSZ-1)
pkt_t txbuf[TXBUFSZ];
uint8_t wptr;
uint8_t rptr;
uint32_t xtime_offset;
char samplethread [2000];
uint32_t nextEchoTag = 1;
#define MSG_TYPE_NEW_PACKETS (0xc201)
#define NETWORK_RTT_US 1000000
//#define PACKET_DESTINATION "2607:f140:400:a009:2ca3:9be1:84ed:cac2"
#define PACKET_DESTINATION "2001:470:1f04:10::2"
#define DESTINATION_PORT 4041
#define STR(x) #x
#define XSTR(x) STR(x)
kernel_pid_t comms_pid;
void comms(void)
{
  msg_t msg;

  gnrc_netreg_entry_t entry = { NULL, 4041, thread_getpid() };

  if (gnrc_netreg_register(GNRC_NETTYPE_UDP, &entry)) {
    puts("ERROR listening\n");
  }

  xtime_offset = 0;
  while(1)
  {
    if (xtimer_msg_receive_timeout(&msg, NETWORK_RTT_US) < 0)
      msg.type = MSG_TYPE_NEW_PACKETS; //default to resending old packet
    uint64_t now64 = xtimer_now64();
    switch(msg.type)
    {
      case GNRC_NETAPI_MSG_TYPE_RCV:
      {
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
      }
      case MSG_TYPE_NEW_PACKETS:
      {
          if (txbuf[rptr].echotag != 0)
          {
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
          }
          break;
      }
    }
  }

}
void *dosample(void *arg)
{
  (void) arg;
  mma7660_t acc;
  if (mma7660_init(&acc, I2C_0, MMA7660_ADDR) != 0) {
    printf("Failed to init ACC\n");
  } else {
    printf("Init acc ok\n");
  }
  if (mma7660_set_mode(&acc, 1, 0, 0, 0) != 0) {
    printf("Failed to set mode\n");
  } else {
    printf("Set mode ok\n");
  }
  if (mma7660_config_samplerate(&acc, MMA7660_SR_AM64, MMA7660_SR_AW32, 1) != 0) {
    printf("Failed to config SR\n");
  }
  uint32_t sample = 0;
  uint32_t last_wakeup = xtimer_now();
  uint8_t needSample = 0;
  int32_t vax = 0;
  int32_t vay = 0;
  int32_t vaz = 0;
  int32_t avgCount = 0;
#define MAX_AVG 10000
#define TRIG  4
  uint8_t haveTriggeredA = 0;
  uint8_t occ = 0;
  uint8_t trig = 0;
  while (1) {
    trig = 0;
    xtimer_usleep_until(&last_wakeup, INTERVAL);
    uint64_t now64 = xtimer_now64();
    now64 /= 10000;
    now64 += xtime_offset;
    sample++;
    int8_t ax = 15, ay = 15, az = 15;
    int8_t x = 15, y = 15, z = 15;
    if (sample % 500 == 0) {
        needSample = 1;
        haveTriggeredA = 0;
    }
    if (1)
    { //Sample accelerometer
      int rv = mma7660_read(&acc, &x, &y, &z);
      if (rv != 0) {
        printf("Failed to read accelerometer %d\n", rv);
      } else {
        if (avgCount >= MAX_AVG) {
            vax >>= 1;
            vay >>= 1;
            vaz >>= 1;
            avgCount >>= 1;
        }
        //printf("Sampled x=%d y=%d z=%d\n", x, y, z);
        vax += x;
        vay += y;
        vaz += z;
        avgCount += 1;
        //printf("V %" PRId32 " %" PRId32 " %" PRId32 " (%" PRIu32 ")\n", vax, vay, vaz, avgCount);
        ax = (int8_t)((vax+(avgCount>>1))/avgCount);
        ay = (int8_t)((vay+(avgCount>>1))/avgCount);
        az = (int8_t)((vaz+(avgCount>>1))/avgCount);
        //printf("Averages: x=%d y=%d z=%d\n", ax, ay, az);
        if ( (ax - x) < -TRIG || (ax - x) > TRIG ||
             (ay - y) < -TRIG || (ay - y) > TRIG ||
             (az - z) < -TRIG || (az - z) > TRIG)
        {
            //puts("WT\n");
            if (!haveTriggeredA) {
                puts("ACCTRIG\n");
                //puts("DIDTRIG\n");
                needSample = 1;
                haveTriggeredA = 1;
                sample=1;
                trig |= ACCTRIG;
            }
        }
      }
    }
    if (needSample)
    { //Sample temperature
      //printf("Sampled temperature: %" PRId32 "\n", tempresult);

      if (((wptr + 1)&TXBUFMASK) != rptr) {
        //We have space
        pkt_t *p = &txbuf[wptr];
        wptr ++;
        wptr &= TXBUFMASK;
        p->echotag = nextEchoTag;
        p->vax = ax;
        p->vay = ay;
        p->vaz = az;
        p->vx = x;
        p->vy = y;
        p->vz = z;
        p->occ = occ;
        p->trig = trig;
        //memcpy(&(p->temp[0]), &tempresult, 4);
        //printf("T %02x %02x %02x %02x\n", p->temp[3], p->temp[2], p->temp[1], p->temp[0]);
        //memcpy(&(p->tm[0]), &now64, 4);
        nextEchoTag++;
        msg_t checkPackets;
        checkPackets.type = MSG_TYPE_NEW_PACKETS;
        msg_send(&checkPackets, comms_pid);
        needSample = 0;
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

    sh("ifconfig 7 set chan 25");
    //char ip [80];
    //sprintf(ip, "ifconfig 7 add 2001:470:4889:110:212:6d03:0:%04x", SERIAL);
    //sh(ip);
    sh("ncache add 7 fe80::5844:3d5c:600e:5e32");
    sh("fibroute add :: via fe80::5844:3d5c:600e:5e32 dev 7");
    //sh("rpl init 7");
    comms_pid = thread_getpid();
    thread_create(samplethread, sizeof(samplethread),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            dosample, NULL, "sampling");

    comms();

    /* should be never reached */
    return 0;
}
