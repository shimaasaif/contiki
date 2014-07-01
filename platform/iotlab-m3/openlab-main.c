/*
 * This file is part of HiKoB Openlab.
 *
 * HiKoB Openlab is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, version 3.
 *
 * HiKoB Openlab is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with HiKoB Openlab. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) 2011,2012 HiKoB.
 */

/**
 * \file openlab-main.c
 *         Configuration for IoT-LAB M3
 *
 * \author
 *         Antoine Fraboulet <antoine.fraboulet.at.hikob.com>
 *         Gaëtan Harter <gaetan.harter.at.inria.fr>
 *
 */


#include <string.h>

#include "platform.h"
#define NO_DEBUG_HEADER
#define LOG_LEVEL LOG_DEBUG
#include "debug.h"


#include "contiki.h"
#include "lib/sensors.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"


#include "dev/light-sensor.h"
#include "dev/acc-mag-sensor.h"
#include "dev/pressure-sensor.h"
#include "dev/gyr-sensor.h"

// fake button sensor
#include "dev/button-sensor.h"

#include "iotlab_uid.h"

#include "contiki-net.h"

#ifndef SLIP_ARCH_CONF_ENABLE
#define SLIP_ARCH_CONF_ENABLE 0
#endif
#if SLIP_ARCH_CONF_ENABLE
#include "dev/slip.h"
#endif

int putchar(int c);
void xputc(char c);

#define PROCESS_CONF_NO_PROCESS_NAMES 0

#if RIMEADDR_SIZE != 2
#error "RIME address size should be set to 2"
#endif /*RIMEADDR_SIZE == 2*/

/*-----------------------------------------------------------------------------------*/
/*
 * IoT-LAB M3 platform, sensors definition
 *
 */

/** Sensors **/
const struct sensors_sensor *sensors[] = {
    &light_sensor, &acc_sensor, &mag_sensor, &pressure_sensor, &gyr_sensor,
    &button_sensor,  // fake button sensor
    0
};

unsigned char sensors_flags[(sizeof(sensors) / sizeof(struct sensors_sensor *))];

/*---------------------------------------------------------------------------*/
void uip_log(char *msg)
{
    log_printf("%s\n", msg);
}
/*---------------------------------------------------------------------------*/
static void set_node_addresses()
{
    uint16_t uid_16b;

    // Use iotlab_uid
    uid_16b = iotlab_uid();

    // Rime address
    rimeaddr_node_addr.u8[0] = 0xff & (uid_16b >> 8);
    rimeaddr_node_addr.u8[1] = 0xff & (uid_16b);

    /*
     * Address for maximum compression
     *
     * http://tools.ietf.org/html/rfc6282#page-6
     *
     * 10: 16 bits.  The first 112 bits of the address are elided.
     *     The value of the first 64 bits is the link-local prefix
     *     padded with zeros.  The following 64 bits are
     *     0000:00ff:fe00:XXXX, where XXXX are the 16 bits carried in-line.
     *
     * -------------------------------------------
     *
     * For 802.15.4, addresses are deducted from the uip_lladdr using the
     * following code:
     *     memcpy(ipaddr->u8 + 8, lladdr, UIP_LLADDR_LEN);
     *     ipaddr->u8[8] ^= 0x02;
     */
    uip_lladdr.addr[0] = 0x02;  // should be a 0 after calculating ip addr
    uip_lladdr.addr[1] = 0;
    uip_lladdr.addr[2] = 0;
    uip_lladdr.addr[3] = 0xff;
    uip_lladdr.addr[4] = 0xfe;
    uip_lladdr.addr[5] = 0;
    uip_lladdr.addr[6] = rimeaddr_node_addr.u8[0];
    uip_lladdr.addr[7] = rimeaddr_node_addr.u8[1];

    log_debug("Rime Addr: %02x:%02x",
            rimeaddr_node_addr.u8[0],
            rimeaddr_node_addr.u8[1]);
    log_debug("uip_lladdr: %02x%02x:%02x%02x:%02x%02x:%02x%02x",
            uip_lladdr.addr[0],
            uip_lladdr.addr[1],
            uip_lladdr.addr[2],
            uip_lladdr.addr[3],
            uip_lladdr.addr[4],
            uip_lladdr.addr[5],
            uip_lladdr.addr[6],
            uip_lladdr.addr[7]);

}
/*---------------------------------------------------------------------------*/
static void
print_processes(struct process * const processes[])
{
#if !PROCESS_CONF_NO_PROCESS_NAMES
    printf(" Starting");
    while(*processes != NULL)
    {
	printf(" '%s'", (*processes)->name);
	processes++;
    }
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */
    putchar('\n');
}
/*---------------------------------------------------------------------------*/
static void char_rx(handler_arg_t arg, uint8_t c)
{
    uart1_get_input_handler()(c);
}
/*---------------------------------------------------------------------------*/
int main()
{
    static uint32_t idle_count = 0;

    /*
     * OpenLab Platform init
     *
     */

    platform_init();

    /*
     * Contiki core
     *
     */

    clock_init();
    process_init();
    process_start(&etimer_process, NULL);
    ctimer_init();

    /*
     * Sensors
     */
    process_start(&sensors_process, NULL);

    /*
     * Network
     *
     */

    netstack_init();
    set_node_addresses();

#if UIP_CONF_IPV6
    process_start(&tcpip_process, NULL);


    #if VIZTOOL_CONF_ON
    process_start(&viztool_process, NULL);
    #endif

    #if (!UIP_CONF_IPV6_RPL)
    {
	uip_ipaddr_t ipaddr;

	uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    }
    #endif /* UIP_CONF_IPV6_RPL */
#endif /* UIP_CONF_IPV6 */

    /*
     * init serial line
     */
    serial_line_init();
    uart_set_rx_handler(uart_print, char_rx, NULL);

    /*
     * eventually init slip device
     * wich may override serial line
     */
#if SLIP_ARCH_CONF_ENABLE
#ifndef UIP_CONF_LLH_LEN
#error "LLH_LEN is not defined"
#elif UIP_CONF_LLH_LEN != 0
#error "LLH_LEN must be 0 to use slip interface"
#endif
    slip_arch_init(SLIP_ARCH_CONF_BAUDRATE);
#endif

    /*
     * Start
     *
     */

    print_processes(autostart_processes);
    autostart_start(autostart_processes);
    watchdog_start();

    while(1) {
        int r;
        do {
            watchdog_periodic();
            r = process_run();
        } while(r > 0);
        idle_count++;
    }

    return 0;
}
/*---------------------------------------------------------------------------*/
