/*
 * From FreeBSD 2.2.7: Fundamental constants relating to ethernet.
 *
 * $Copyright Open Broadcom Corporation$
 *
 * $Id: ethernet.h 285437 2011-09-21 22:16:56Z $
 */

#ifndef _NET_ETHERNET_H_	  /* use native BSD ethernet.h when available */
#define _NET_ETHERNET_H_

#ifndef _TYPEDEFS_H_
#include "typedefs.h"
#endif

/* This marks the start of a packed structure section. */
#include <packed_section_start.h>


/*
 * The number of bytes in an ethernet (MAC) address.
 */
#define	ETHER_ADDR_LEN		6

/*
 * The number of bytes in the type field.
 */
#define	ETHER_TYPE_LEN		2

/*
 * The number of bytes in the trailing CRC field.
 */
#define	ETHER_CRC_LEN		4

/*
 * The length of the combined header.
 */
#define	ETHER_HDR_LEN		(ETHER_ADDR_LEN * 2 + ETHER_TYPE_LEN)

/*
 * The minimum packet length.
 */
#define	ETHER_MIN_LEN		64

/*
 * The minimum packet user data length.
 */
#define	ETHER_MIN_DATA		46

/*
 * The maximum packet length.
 */
#define	ETHER_MAX_LEN		1518

/*
 * The maximum packet user data length.
 */
#define	ETHER_MAX_DATA		1500

/* ether types */
#define ETHER_TYPE_MIN		0x0600		/* Anything less than MIN is a length */
#define	ETHER_TYPE_IP		0x0800		/* IP */
#define ETHER_TYPE_ARP		0x0806		/* ARP */
#define ETHER_TYPE_8021Q	0x8100		/* 802.1Q */
#define	ETHER_TYPE_BRCM		0x886c		/* Broadcom Corp. */
#define	ETHER_TYPE_802_1X	0x888e		/* 802.1x */
#define	ETHER_TYPE_802_1X_PREAUTH 0x88c7	/* 802.1x preauthentication */
#define ETHER_TYPE_WAI		0x88b4		/* WAI */
#define ETHER_TYPE_89_0D	0x890d		/* 89-0d frame for TDLS */


/* Broadcom subtype follows ethertype;  First 2 bytes are reserved; Next 2 are subtype; */
#define	ETHER_BRCM_SUBTYPE_LEN	4	/* Broadcom 4 byte subtype */
#define	ETHER_BRCM_CRAM		1	/* Broadcom subtype cram protocol */

/* ether header */
#define ETHER_DEST_OFFSET	(0 * ETHER_ADDR_LEN)	/* dest address offset */
#define ETHER_SRC_OFFSET	(1 * ETHER_ADDR_LEN)	/* src address offset */
#define ETHER_TYPE_OFFSET	(2 * ETHER_ADDR_LEN)	/* ether type offset */

/*
 * A macro to validate a length with
 */
#define	ETHER_IS_VALID_LEN(foo)	\
	((foo) >= ETHER_MIN_LEN && (foo) <= ETHER_MAX_LEN)

#define ETHER_FILL_MCAST_ADDR_FROM_IP(ea, mgrp_ip) {		\
		((uint8 *)ea)[0] = 0x01;			\
		((uint8 *)ea)[1] = 0x00;			\
		((uint8 *)ea)[2] = 0x5e;			\
		((uint8 *)ea)[3] = ((mgrp_ip) >> 16) & 0x7f;	\
		((uint8 *)ea)[4] = ((mgrp_ip) >>  8) & 0xff;	\
		((uint8 *)ea)[5] = ((mgrp_ip) >>  0) & 0xff;	\
}

#ifndef __INCif_etherh   /* Quick and ugly hack for VxWorks */
/*
 * Structure of a 10Mb/s Ethernet header.
 */
BWL_PRE_PACKED_STRUCT struct ether_header {
	uint8	ether_dhost[ETHER_ADDR_LEN];
	uint8	ether_shost[ETHER_ADDR_LEN];
	uint16	ether_type;
} BWL_POST_PACKED_STRUCT;

/*
 * Structure of a 48-bit Ethernet address.
 */
BWL_PRE_PACKED_STRUCT struct	ether_addr {
	uint8 octet[ETHER_ADDR_LEN];
} BWL_POST_PACKED_STRUCT;
#endif	/* !__INCif_etherh Quick and ugly hack for VxWorks */

/*
 * Takes a pointer, set, test, clear, toggle locally admininistered
 * address bit in the 48-bit Ethernet address.
 */
#define ETHER_SET_LOCALADDR(ea)	(((uint8 *)(ea))[0] = (((uint8 *)(ea))[0] | 2))
#define ETHER_IS_LOCALADDR(ea) 	(((uint8 *)(ea))[0] & 2)
#define ETHER_CLR_LOCALADDR(ea)	(((uint8 *)(ea))[0] = (((uint8 *)(ea))[0] & 0xd))
#define ETHER_TOGGLE_LOCALADDR(ea)	(((uint8 *)(ea))[0] = (((uint8 *)(ea))[0] ^ 2))

/* Takes a pointer, marks unicast address bit in the MAC address */
#define ETHER_SET_UNICAST(ea)	(((uint8 *)(ea))[0] = (((uint8 *)(ea))[0] & ~1))

/*
 * Takes a pointer, returns true if a 48-bit multicast address
 * (including broadcast, since it is all ones)
 */
#define ETHER_ISMULTI(ea) (((const uint8 *)(ea))[0] & 1)


/* compare two ethernet addresses - assumes the pointers can be referenced as shorts */
#define	ether_cmp(a, b)	(!(((short*)a)[0] == ((short*)b)[0]) | \
			 !(((short*)a)[1] == ((short*)b)[1]) | \
			 !(((short*)a)[2] == ((short*)b)[2]))

/* copy an ethernet address - assumes the pointers can be referenced as shorts */
#define	ether_copy(s, d) { \
		((short*)d)[0] = ((short*)s)[0]; \
		((short*)d)[1] = ((short*)s)[1]; \
		((short*)d)[2] = ((short*)s)[2]; }


static const struct ether_addr ether_bcast = {{255, 255, 255, 255, 255, 255}};
static const struct ether_addr ether_null = {{0, 0, 0, 0, 0, 0}};

#define ETHER_ISBCAST(ea)	((((uint8 *)(ea))[0] &		\
	                          ((uint8 *)(ea))[1] &		\
				  ((uint8 *)(ea))[2] &		\
				  ((uint8 *)(ea))[3] &		\
				  ((uint8 *)(ea))[4] &		\
				  ((uint8 *)(ea))[5]) == 0xff)
#define ETHER_ISNULLADDR(ea)	((((uint8 *)(ea))[0] |		\
				  ((uint8 *)(ea))[1] |		\
				  ((uint8 *)(ea))[2] |		\
				  ((uint8 *)(ea))[3] |		\
				  ((uint8 *)(ea))[4] |		\
				  ((uint8 *)(ea))[5]) == 0)


#define ETHER_MOVE_HDR(d, s) \
do { \
	struct ether_header t; \
	t = *(struct ether_header *)(s); \
	*(struct ether_header *)(d) = t; \
} while (0)

/* This marks the end of a packed structure section. */
#include <packed_section_end.h>

#endif /* _NET_ETHERNET_H_ */
