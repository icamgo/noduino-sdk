/* Copyright Statement:
 *
 * (C) 2005-2016  MediaTek Inc. All rights reserved.
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. ("MediaTek") and/or its licensors.
 * Without the prior written permission of MediaTek and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 * You may only use, reproduce, modify, or distribute (as applicable) MediaTek Software
 * if you have agreed to and been bound by the applicable license agreement with
 * MediaTek ("License Agreement") and been granted explicit permission to do so within
 * the License Agreement ("Permitted User").  If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES
 * ARE PROVIDED TO RECEIVER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */

/**
 * @file
 *
 * Neighbor discovery and stateless address autoconfiguration for IPv6.
 * Aims to be compliant with RFC 4861 (Neighbor discovery) and RFC 4862
 * (Address autoconfiguration).
 */
/*
 * Copyright (c) 2010 Inico Technologies Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Ivan Delamer <delamer@inicotech.com>
 *
 *
 * Please coordinate changes and requests with Ivan Delamer
 * <delamer@inicotech.com>
 */

#ifndef LWIP_HDR_ND6_H
#define LWIP_HDR_ND6_H

#include "lwip/opt.h"

#if LWIP_IPV6  /* don't build if not configured for use in lwipopts.h */

#include "lwip/pbuf.h"
#include "lwip/ip6.h"
#include "lwip/ip6_addr.h"
#include "lwip/netif.h"

#define LWIP_ND6_NUM_AUTOCFG_PREFIXES 3
#define ND6_AUTOCFG_PREFIX_LEN  64
#define ND6_AUTOCFG_INTERFACE_LEN       (128 - ND6_AUTOCFG_PREFIX_LEN)
#define LWIP_ND6_PTR_SOLICITATION_INTERVAL (4000)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ND6_SEND_FLAG_MULTICAST_DEST = 0x01,
    ND6_SEND_FLAG_ALLNODES_DEST = 0x02,
    ND6_SEND_FLAG_NS_NO_OPTION = 0x04,

    /* Source address indication flags: */
    ND6_SEND_FLAG_SRC_LINK_LOCAL = 0x10,
    ND6_SEND_FLAG_SRC_ADDR1 = 0x20,
    ND6_SEND_FLAG_SRC_ADDR2 = 0x40,
    ND6_SEND_FLAG_SRC_ANY_ADDR = 0x80,
    ND6_SEND_FLAG_SRC_MARK_ALL = 0xF0
}nd6_send_flag_enum;

typedef enum
{
    IPV6_AUTOCONFIG_DISENABLED = 0,
    IPV6_AUTOCONFIG_ENABLED = 1,
    IPV6_AUTOCONFIG_STOP = 2,
    IPV6_AUTOCONFIG_ALL
} ip6_autoconfig_enabled_enum;


/* Struct for tables. */
struct nd6_neighbor_cache_entry {
  ip6_addr_t next_hop_address;
  struct netif * netif;
  u8_t lladdr[NETIF_MAX_HWADDR_LEN];
  /*u32_t pmtu;*/
#if LWIP_ND6_QUEUEING
  /** Pointer to queue of pending outgoing packets on this entry. */
  struct nd6_q_entry *q;
#else /* LWIP_ND6_QUEUEING */
  /** Pointer to a single pending outgoing packet on this entry. */
  struct pbuf *q;
#endif /* LWIP_ND6_QUEUEING */
  u8_t state;
  u8_t isrouter;
  union {
    u32_t reachable_time;
    u32_t delay_time;
    u32_t probes_sent;
    u32_t stale_time;
  } counter;
  u32_t retrans_interval;
  nd6_send_flag_enum src_flag;   /* source flag: any, linklocal, g1, g2 */
};

struct nd6_destination_cache_entry {
  ip6_addr_t destination_addr;
  ip6_addr_t next_hop_addr;
  u16_t pmtu;
  u32_t age;
};

struct nd6_prefix_list_entry {
  ip6_addr_t prefix;
  u8_t prefix_len;
  struct netif * netif;
  u32_t invalidation_timer;
  u32_t preferred_timer;
#if LWIP_IPV6_AUTOCONFIG
  u8_t flags;
#define ND6_PREFIX_AUTOCONFIG_AUTONOMOUS 0x01
#define ND6_PREFIX_AUTOCONFIG_ADDRESS_GENERATED 0x02
#define ND6_PREFIX_AUTOCONFIG_ADDRESS_DUPLICATE 0x04
#define ND6_PREFIX_AUTOCONFIG_ADDRESS_DEPRECATED 0x08

#endif /* LWIP_IPV6_AUTOCONFIG */
};

struct nd6_router_list_entry {
  struct nd6_neighbor_cache_entry * neighbor_entry;
  u32_t invalidation_timer;
  u8_t flags;
};


enum nd6_neighbor_cache_entry_state {
  ND6_NO_ENTRY = 0,
  ND6_INCOMPLETE,
  ND6_REACHABLE,
  ND6_STALE,
  ND6_DELAY,
  ND6_PROBE
};


typedef enum
{
    ND6_ERR_OK = 0,
    ND6_ERR_INVALID_PARAM = -1,
    ND6_ERR_VIOLATE_PROTOCAL = -2,
    ND6_ERR_MISC_ERR = -3,

    ND6_ERR_END = 0xFF
}nd6_err_enum;


#if LWIP_ND6_QUEUEING
/** struct for queueing outgoing packets for unknown address
  * defined here to be accessed by memp.h
  */
struct nd6_q_entry {
  struct nd6_q_entry *next;
  struct pbuf *p;
};
#endif /* LWIP_ND6_QUEUEING */

/** Neighbor solicitation message header. */
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct ns_header {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FIELD(u32_t reserved);
  PACK_STRUCT_FLD_S(ip6_addr_p_t target_address);
  /* Options follow. */
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Neighbor advertisement message header. */
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct na_header {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FLD_8(u8_t flags);
  PACK_STRUCT_FLD_8(u8_t reserved[3]);
  PACK_STRUCT_FLD_S(ip6_addr_p_t target_address);
  /* Options follow. */
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif
#define ND6_FLAG_ROUTER      (0x80)
#define ND6_FLAG_SOLICITED   (0x40)
#define ND6_FLAG_OVERRIDE    (0x20)
#define ND6_FLAG_NO_OPTION   (0x10)

/** Router solicitation message header. */
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct rs_header {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FIELD(u32_t reserved);
  /* Options follow. */
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#define ND6_RS_COUNT    (3)
#define ND6_RS_INTERVAL (4000)

/** Router advertisement message header. */
#define ND6_RA_FLAG_MANAGED_ADDR_CONFIG (0x80)
#define ND6_RA_FLAG_OTHER_CONFIG (0x40)
#define ND6_RA_FLAG_HOME_AGENT (0x20)
#define ND6_RA_PREFERENCE_MASK (0x18)
#define ND6_RA_PREFERENCE_HIGH (0x08)
#define ND6_RA_PREFERENCE_MEDIUM (0x00)
#define ND6_RA_PREFERENCE_LOW (0x18)
#define ND6_RA_PREFERENCE_DISABLED (0x10)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct ra_header {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FLD_8(u8_t current_hop_limit);
  PACK_STRUCT_FLD_8(u8_t flags);
  PACK_STRUCT_FIELD(u16_t router_lifetime);
  PACK_STRUCT_FIELD(u32_t reachable_time);
  PACK_STRUCT_FIELD(u32_t retrans_timer);
  /* Options follow. */
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Redirect message header. */
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct redirect_header {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FIELD(u32_t reserved);
  PACK_STRUCT_FLD_S(ip6_addr_p_t target_address);
  PACK_STRUCT_FLD_S(ip6_addr_p_t destination_address);
  /* Options follow. */
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Link-layer address option. */
#define ND6_OPTION_TYPE_SOURCE_LLADDR (0x01)
#define ND6_OPTION_TYPE_TARGET_LLADDR (0x02)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct lladdr_option {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t length);
  PACK_STRUCT_FLD_8(u8_t addr[NETIF_MAX_HWADDR_LEN]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Prefix information option. */
#define ND6_OPTION_TYPE_PREFIX_INFO (0x03)
#define ND6_PREFIX_FLAG_ON_LINK (0x80)
#define ND6_PREFIX_FLAG_AUTONOMOUS (0x40)
#define ND6_PREFIX_FLAG_ROUTER_ADDRESS (0x20)
#define ND6_PREFIX_FLAG_SITE_PREFIX (0x10)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct prefix_option {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t length);
  PACK_STRUCT_FLD_8(u8_t prefix_length);
  PACK_STRUCT_FLD_8(u8_t flags);
  PACK_STRUCT_FIELD(u32_t valid_lifetime);
  PACK_STRUCT_FIELD(u32_t preferred_lifetime);
  PACK_STRUCT_FLD_8(u8_t reserved2[3]);
  PACK_STRUCT_FLD_8(u8_t site_prefix_length);
  PACK_STRUCT_FLD_S(ip6_addr_p_t prefix);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Redirected header option. */
#define ND6_OPTION_TYPE_REDIR_HDR (0x04)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct redirected_header_option {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t length);
  PACK_STRUCT_FLD_8(u8_t reserved[6]);
  /* Portion of redirected packet follows. */
  /* PACK_STRUCT_FLD_8(u8_t redirected[8]); */
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** MTU option. */
#define ND6_OPTION_TYPE_MTU (0x05)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct mtu_option {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t length);
  PACK_STRUCT_FIELD(u16_t reserved);
  PACK_STRUCT_FIELD(u32_t mtu);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Route information option. */
#define ND6_OPTION_TYPE_ROUTE_INFO (24)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct route_option {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t length);
  PACK_STRUCT_FLD_8(u8_t prefix_length);
  PACK_STRUCT_FLD_8(u8_t preference);
  PACK_STRUCT_FIELD(u32_t route_lifetime);
  PACK_STRUCT_FLD_S(ip6_addr_p_t prefix);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Recursive DNS Server Option. */
#define ND6_OPTION_TYPE_RDNSS (25)
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct rdnss_option {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t length);
  PACK_STRUCT_FIELD(u16_t reserved);
  PACK_STRUCT_FIELD(u32_t lifetime);
  PACK_STRUCT_FLD_S(ip6_addr_p_t rdnss_address[1]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#define SIZEOF_RDNSS_OPTION_BASE 8 /* size without addresses */

/* the possible states of an IP address */
#define IP6_ADDRESS_STATE_INVALID     (0)
#define IP6_ADDRESS_STATE_VALID       (0x4)
#define IP6_ADDRESS_STATE_PREFERRED   (0x5) /* includes valid */
#define IP6_ADDRESS_STATE_DEPRECATED  (0x6) /* includes valid */
#define IP6_ADDRESS_STATE_TENTATIV    (0x8)

/** 1 second period */
#define ND6_TMR_INTERVAL 1000

#define ND6_TMR_DALAY 5
#define ND6_TMR_TOLERATE_BIAS   (50) /* Milliseconde */

#define ND6_ABS(a)  ((a) >= 0 ? (a) : -(a))

/* Router tables. */
/* TODO make these static? and entries accessible through API? */
extern struct nd6_neighbor_cache_entry neighbor_cache[];
extern struct nd6_destination_cache_entry destination_cache[];
extern struct nd6_prefix_list_entry prefix_list[];
extern struct nd6_router_list_entry default_router_list[];

/* Default values, can be updated by a RA message. */
extern u32_t reachable_time;
extern u32_t retrans_timer;

void nd6_tmr(void);
void nd6_input(struct pbuf *p, struct netif *inp);
void nd6_icmp_ptb_input(struct pbuf *p, struct netif *inp);
s8_t nd6_get_next_hop_entry(const ip6_addr_t * ip6addr, struct netif * netif);
s8_t nd6_select_router(const ip6_addr_t * ip6addr, struct netif * netif);
u16_t nd6_get_destination_mtu(const ip6_addr_t * ip6addr, struct netif * netif);
err_t nd6_queue_packet(s8_t neighbor_index, struct pbuf * p);
#if LWIP_ND6_TCP_REACHABILITY_HINTS
void nd6_reachability_hint(const ip6_addr_t * ip6addr);
#endif /* LWIP_ND6_TCP_REACHABILITY_HINTS */
s8_t nd6_find_prefix_by_netif(struct netif* inp);
void nd6_reset_prefix(s8_t index, struct netif* inp);
s8_t nd6_find_autocfg_prefix_by_netif(struct netif* inp);
void nd6_reset_autocfg_prefix(s8_t index, struct netif* inp);
void nd6_clear_onlink_prefix_by_netif(struct netif* inp);
void nd6_clear_autocfg_prefix_by_netif(struct netif* inp);
void nd6_reset_cache(void);
extern u8_t nd6_get_cur_hop_limit(void);
extern s8_t nd6_gen_ns_src_flag(u8_t nce_idx);
extern void nd6_snd_ns_immediately(u8_t nce_idx);

#ifdef __cplusplus
}
#endif

#endif /* LWIP_IPV6 */

#endif /* LWIP_HDR_ND6_H */
