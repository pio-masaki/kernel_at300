/*
 * Common stats definitions for clients of dongle
 * ports
 *
 * $Copyright Open Broadcom Corporation$
 *
 * $Id: dngl_stats.h,v 1.5 2008-06-02 16:56:20 $
 */

#ifndef _dngl_stats_h_
#define _dngl_stats_h_

typedef struct {
	unsigned long	rx_packets;		/* total packets received */
	unsigned long	tx_packets;		/* total packets transmitted */
	unsigned long	rx_bytes;		/* total bytes received */
	unsigned long	tx_bytes;		/* total bytes transmitted */
	unsigned long	rx_errors;		/* bad packets received */
	unsigned long	tx_errors;		/* packet transmit problems */
	unsigned long	rx_dropped;		/* packets dropped by dongle */
	unsigned long	tx_dropped;		/* packets dropped by dongle */
	unsigned long   multicast;      /* multicast packets received */
} dngl_stats_t;

#endif /* _dngl_stats_h_ */
