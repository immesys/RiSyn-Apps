#include <stdio.h>
#include <rtt_stdio.h>
#include "shell.h"
#include "thread.h"
#include "xtimer.h"
#include <string.h>

#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"

#include <at30ts74.h>
#include <mma7660.h>
#include <periph/gpio.h>

// 1 second, defined in us
#define INTERVAL (1000000U)
#define NETWORK_RTT_US 1000000

extern int _netif_config(int argc, char **argv);
extern void send(char *addr_str, char *port_str, char *data, uint16_t datalen);
extern void start_server(char *port_str);
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
extern void handle_input_line(const shell_command_t *command_list, char *line);

// listen for UDP packets
void server(void)
{
  msg_t msg;
  msg_t msg_queue[16];
  msg_init_queue(msg_queue, 16);
  uint32_t led_sleep;

  union value2 {int32_t number; uint8_t bytes[2];} message_num;
  union value4 {int32_t number; uint8_t bytes[4];} temperature_value ;
  int8_t x,y,z;


  gnrc_netreg_entry_t entry = { NULL, 4444, thread_getpid() };
  if (gnrc_netreg_register(GNRC_NETTYPE_UDP, &entry)) {
    printf("ERROR listening\n");
  }

  printf("start listening for packets\n");
  while(1) // server loop
  {
    //xtimer_msg_receive_timeout(&msg, NETWORK_RTT_US);
    msg_receive(&msg);

    switch(msg.type)
    {
        case GNRC_NETAPI_MSG_TYPE_RCV:
        {
          printf("Received a packet \n");
          gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)(msg.content.ptr);
          gnrc_pktsnip_t *tmp;
          LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_UDP);
          //udp_hdr_t *udp = (udp_hdr_t *)tmp->data;
          LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_IPV6);
          //ipv6_hdr_t *ip = (ipv6_hdr_t *)tmp->data;
          //
          char *bytes = (char *)pkt->data;
          message_num.bytes[0] = bytes[0];
          message_num.bytes[1] = bytes[1];
          temperature_value.bytes[0] = bytes[2];
          temperature_value.bytes[1] = bytes[3];
          temperature_value.bytes[2] = bytes[4];
          temperature_value.bytes[3] = bytes[5];
          x = (int8_t)bytes[6];
          y = (int8_t)bytes[7];
          z = (int8_t)bytes[8];

          printf("got msg! num %" PRId32 ", temperature %" PRId32, message_num.number, temperature_value.number);
          printf(" x: %i, y: %i, z: %i\n", x,y,z);

          gnrc_pktbuf_release((gnrc_pktsnip_t *) msg.content.ptr);

          if (z < 0)
          {
            LED_ON;
            led_sleep = xtimer_now();
            xtimer_periodic_wakeup(&led_sleep, 250000);
            LED_OFF;
          }
          else if (temperature_value.number > 280000)
          {
            led_sleep = xtimer_now();
            LED_ON;
            xtimer_periodic_wakeup(&led_sleep, 200000);
            LED_OFF;
            xtimer_periodic_wakeup(&led_sleep, 100000);
            LED_ON;
            xtimer_periodic_wakeup(&led_sleep, 200000);
            LED_OFF;
          }
          break;
        }
        default:
        {
          printf("Expected %d but got %d\n", GNRC_NETAPI_MSG_TYPE_RCV, msg.type);
        }
    }

  }
}

void client(void)
{
    // temperature sensor
    at30ts74_t tmp;
    at30ts74_init(&tmp, I2C_0, AT30TS74_ADDR, AT30TS74_12BIT);
    // accelerometer
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

    uint32_t last_wakeup;
    int res;
    last_wakeup = xtimer_now();
    union value2 {int32_t number; uint8_t bytes[2];} message_num;
    union value4 {int32_t number; uint8_t bytes[4];} temperature_value ;
    int8_t x,y,z;
    memset(&message_num, 0, sizeof(message_num));
    memset(&temperature_value, 0, sizeof(temperature_value));
    message_num.number = 0;
    temperature_value.number = 0;

    while(1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);

        char buf [10];

        // increment message number
        message_num.number++;
        // pack it
        buf[0] = message_num.bytes[0];
        buf[1] = message_num.bytes[1];

        // read temperature
        res = at30ts74_read(&tmp, &temperature_value.number);
        if (res == 0)
        {
            buf[2] = temperature_value.bytes[0];
            buf[3] = temperature_value.bytes[1];
            buf[4] = temperature_value.bytes[2];
            buf[5] = temperature_value.bytes[3];
        }
        else
        {
            buf[2] = 0xff;
            buf[3] = 0xff;
            buf[4] = 0xff;
            buf[5] = 0xff;
        }

        // read accelerometer
        if (mma7660_read(&acc, &x, &y, &z) != 0)
        {
            printf("Could not read accel\n");
        }
        printf("accel %d %d %d\n", x, y, z);
        // pack it
        buf[6] = (uint8_t)x;
        buf[7] = (uint8_t)y;
        buf[8] = (uint8_t)z;

        send("ff02::1", "4444", buf, 10);
        LED_ON;
        xtimer_periodic_wakeup(&last_wakeup, 500000);
        LED_OFF;
        printf("sending msg! num %" PRId32 ", temperature %" PRId32 "\n", message_num.number, temperature_value.number);
        //printf("Sending packet %d\n", message_num);
    }
}

int main(void)
{
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    server();
    //client();

    return 0;
}
