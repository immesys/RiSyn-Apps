/*
 * Copyright (C) 2016 Michael Andersen <m.andersen@cs.berkeley.edu>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     rtt_stdio_examples
 * @{
 *
 * @file
 * @brief       example application using SEGGER RTT for STDIO
 *
 * @author      Michael Andersen <m.andersen@cs.berkeley.edu>
 *
 * @}
 */

#include <stdio.h>
#include "xtimer.h"
#include <rtt_stdio.h>

/* set interval to 1 second */
#define INTERVAL (1000000U)

int main(void)
{
    uint32_t last_wakeup = xtimer_now();
    char buf[80];
    //This enables stdin polling. Without this, reads from stdin simply
    //block forever. It is not enabled by default because it would waste
    //power (assuming I get LPM modes working properly).
    rtt_stdio_enable_stdin();

    while(1) {
        xtimer_usleep_until(&last_wakeup, INTERVAL);
        LED_TOGGLE;
        printf ("hello world. It is: %lu\n", xtimer_now());
        printf ("we read '%s'\n", gets(buf));
    }

    return 0;
}
