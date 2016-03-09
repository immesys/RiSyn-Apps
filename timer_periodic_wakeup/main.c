/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     xtimer_examples
 * @{
 *
 * @file
 * @brief       example application for setting a periodic wakeup
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdio.h>
#include "xtimer.h"
#include <rtt_stdio.h>
#include <at30ts74.h>

/* set interval to 1 second */
#define INTERVAL (100000U)

int main(void)
{
    uint32_t last_wakeup = xtimer_now();
    at30ts74_t tmp;
    at30ts74_init(&tmp, I2C_0, AT30TS74_ADDR, AT30TS74_12BIT);
    while(1) {
        xtimer_usleep_until(&last_wakeup, INTERVAL);
        LED_TOGGLE;
        int res;
        int32_t result;
        res = at30ts74_read(&tmp, &result);
        printf ("We got an rv of %d and a temperature of %" PRId32 "\n", res, result);
    }

    return 0;
}
