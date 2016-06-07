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
gpio_t gpout;

/*
 * The insole will do PWM
 */
void insole(void)
{
    msg_t m;
    uint32_t pwm_unit = 10000;
    uint32_t percent = 50;
    uint32_t last_wakeup = xtimer_now();
    int i = 0;
    while (1)
    {
        i++;
        printf("i %i %i\n", i, msg_avail());
        if (msg_try_receive(&m) > 0)
        {
            //printf("got value %" PRId32 "\n",m.content.value);
            percent = m.content.value;
        }
        else
        {
            percent = 50;
        }
        if (percent >= 100)
        {
            percent = 99;
        }
        else if (percent <= 0)
        {
            percent = 0;
        }
        if (percent > 0)
        {
            LED_ON;
            gpio_set(gpout);
            xtimer_usleep_until(&last_wakeup, percent * pwm_unit);
        }
        LED_OFF;
        gpio_clear(gpout);
        xtimer_usleep_until(&last_wakeup, (100 - percent) * pwm_unit);
    }
}

// this thread listens for incoming network packets that tell us how fast to PWM
void *network_thread(void *arg)
{
    printf("network thread started\n");
    union value4 {int32_t number; uint8_t bytes[4];} new_interval;
    kernel_pid_t main_pid = *(kernel_pid_t*)arg;
    msg_t msg;
    msg_t msg_queue[MAIN_QUEUE_SIZE];
    msg_init_queue(msg_queue, MAIN_QUEUE_SIZE);
    msg_t ipc_msg;
    gnrc_netreg_entry_t entry = { NULL, 4444, thread_getpid() };
    if (gnrc_netreg_register(GNRC_NETTYPE_UDP, &entry)) {
      //printf("ERROR listening\n");
    }
    while(1)
    {
        msg_receive(&msg);
        switch(msg.type)
        {
            case GNRC_NETAPI_MSG_TYPE_RCV:
            {
                gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)(msg.content.ptr);
                gnrc_pktsnip_t *tmp;
                LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_UDP);
                LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_IPV6);
                char *bytes = (char *)pkt->data;
                new_interval.bytes[0] = bytes[0];
                new_interval.bytes[1] = bytes[1];
                ipc_msg.content.value = new_interval.number;
                int rv = msg_try_send(&ipc_msg, main_pid); // send to insole thread
                gnrc_pktbuf_release((gnrc_pktsnip_t *) msg.content.ptr);
                printf("%i\n", rv);
            }
        }
    }
}

void *accel_thread(void *arg)
{
    msg_t msg_queue[MAIN_QUEUE_SIZE];
    msg_init_queue(msg_queue, MAIN_QUEUE_SIZE);
    kernel_pid_t main_pid = *(kernel_pid_t*)arg;
    printf("accel thread started\n");
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
    int8_t x,y,z;
    msg_t ipc_msg;

    union value4 {int32_t number; uint8_t bytes[4];} new_interval;
    uint32_t last_wakeup = xtimer_now();
    char buf [10];
    while (1) {
        if (mma7660_read(&acc, &x, &y, &z) != 0)
        {
            printf("Could not read accel\n");
        }
        printf("accel %d %d %d\n", x, y, z);
        if (z < 0)
        {
            z = 0;
            //z = -z;
        }

        if (z > 16)
        {
            z = 25;
        }
        ipc_msg.content.value = (z * 4);
        new_interval.number = z*4;
        msg_try_send(&ipc_msg, main_pid); // send to insole thread

        buf[0] = new_interval.bytes[0];
        buf[1] = new_interval.bytes[1];

        send("ff02::1", "4444", buf, 10);

        xtimer_usleep_until(&last_wakeup, 1000000);

    }
}

char network_thread_stack[THREAD_STACKSIZE_MAIN];

int main(void)
{
    //TODO: change this depending on what you are
    bool is_insole = true;

    gpout = GPIO_PIN(0, 18);
    gpio_init(gpout, GPIO_DIR_OUT, GPIO_NOPULL);

    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    char lbuf[100];
    strcpy(lbuf, "ifconfig 7 set chan 22\n");
    handle_input_line(NULL, lbuf);

    kernel_pid_t main_pid = thread_getpid();
    kernel_pid_t pid;
    if (is_insole) {
    pid = thread_create(network_thread_stack, sizeof(network_thread_stack),
                                     THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                                     network_thread, (void*)&main_pid, "network");
    } else {
    pid = thread_create(network_thread_stack, sizeof(network_thread_stack),
                                     THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                                     accel_thread, (void*)&main_pid, "network");
    }

    printf("Created network thread %" PRIkernel_pid "\n", pid);

    insole();

    return 0;
}
