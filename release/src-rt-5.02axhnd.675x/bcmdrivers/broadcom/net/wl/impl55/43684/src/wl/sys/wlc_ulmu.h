/*
 * 802.11ax uplink MU scheduler and scheduler statistics module
 *
 * Copyright 2019 Broadcom
 *
 * This program is the proprietary software of Broadcom and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 *
 * 1. This program, including its structure, sequence and organization,
 * constitutes the valuable trade secrets of Broadcom, and you shall use
 * all reasonable efforts to protect the confidentiality thereof, and to
 * use this information only in connection with your use of Broadcom
 * integrated circuit products.
 *
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
 * "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES,
 * REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR
 * OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY
 * DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
 * NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES,
 * ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR
 * CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING
 * OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 *
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL
 * BROADCOM OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL,
 * SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR
 * IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN
 * IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii)
 * ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF
 * OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY
 * NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *
 * <<Broadcom-WL-IPTag/Proprietary:>>
 *
 * $Id:$
 */

#ifndef _wlc_ulmu_h_
#define _wlc_ulmu_h_

#include <wlc_types.h>

/* ulmu ul policy definition */
#define ULMU_POLICY_DISABLE		0
#define ULMU_POLICY_BASIC		1
#define ULMU_POLICY_MAX			1
#define ULMU_POLICY_AUTO		(-1)

#define ULMU_USRCNT_MAX			32

#define ULMU_OFDMA_TRSSI_MAP			110
#define ULMU_OFDMA_TRSSI_MIN			20  /* -90 dBm */
#define ULMU_OFDMA_TRSSI_MAX			90  /* -20 dBm */
#define ULMU_OFDMA_TRSSI_INIT			80  /* -30 dBm */

#define ULMU_TRIG_FIFO			5

/* attach/detach */
wlc_ulmu_info_t *wlc_ulmu_attach(wlc_info_t *wlc);
void wlc_ulmu_detach(wlc_ulmu_info_t *ulmu);

/* Driver UL trigger interface */

#define ULMU_PACKET_TRIGGER		0
#define ULMU_TWT_TRIGGER		1

#define ULMU_QOSNULL_LIMIT		6
#define ULMU_TIMEOUT_LIMIT		7
#define ULMU_CB_THRESHOLD		100
#define ULMU_CB_TEST_THRESHOLD	80

#define ULMU_STATUS_INPROGRESS	0
#define ULMU_STATUS_THRESHOLD	1
#define ULMU_STATUS_COMPLETE	2
#define ULMU_STATUS_QOSNULL		10
#define ULMU_STATUS_TIMEOUT		20
#define ULMU_STATUS_TRIGCNT		30
#define	ULMU_STATUS_WATCHDOG	40
#define ULMU_STATUS_UNKNOWN		((uint32) (-1))

typedef struct packet_trigger_info {
	uint32 trigger_bytes;
	int (*callback_function) (wlc_info_t *wlc, struct scb *scb,
		void *arg, uint32 status_code, uint32 bytes_consumed);
	void *callback_parameter;
	uint32 callback_reporting_threshold;
	bool multi_callback;
	uint32 qos_null_threshold;
	uint32 failed_request_threshold;
	uint32 watchdog_timeout;
} packet_trigger_info_t;

/* Placeholder for TWT trigger data */
typedef struct twt_trigger_info {
	uint32 trigger_bytes;
} twt_trigger_info_t;

typedef struct wlc_ulmu_trigger_info {
	union {
		packet_trigger_info_t packet_trigger;
		twt_trigger_info_t twt_trigger;
	} trigger_type;
} wlc_ulmu_trigger_info_t;

extern int wlc_ulmu_drv_trigger_request(wlc_info_t *wlc, struct scb *scb,
	int trigger_type, wlc_ulmu_trigger_info_t *trigger_info);

/* UL OFDMA */
extern bool wlc_ulmu_admit_clients(wlc_info_t *wlc, scb_t *scb, bool admit);

extern void wlc_ulmu_ul_nss_upd(wlc_ulmu_info_t *ulmu, scb_t* scb, uint8 tx_nss);

extern int wlc_ulmu_stats_upd(wlc_ulmu_info_t *ulmu, struct scb *scb,
	tx_status_t *txs);
#if defined(BCMDBG) || defined(BCMDBG_DUMP) || defined(UL_RU_STATS_DUMP)
extern int wlc_ulmu_rustats_upd(wlc_ulmu_info_t *ulmu, struct scb *scb,
	tx_status_t *txs);
#endif /* defined(BCMDBG) || defined(BCMDBG_DUMP) || defined(UL_RU_STATS_DUMP) */
extern void wlc_ulmu_fburst_set(wlc_ulmu_info_t *ulmu, bool enable);
extern void wlc_ulmu_max_trig_usrs_set(wlc_ulmu_info_t *ulmu, uint16 val);
extern void wlc_ulmu_maxn_uplimit_set(wlc_ulmu_info_t *ulmu, uint16 val);
#if defined(WL11AX) && defined(WL_PSMX)
extern void wlc_ulmu_chanspec_upd(wlc_info_t *wlc);
extern void wlc_ulmu_oper_state_upd(wlc_ulmu_info_t* ulmu, scb_t *scb, uint8 state);
#else
#define wlc_ulmu_chanspec_upd(a) do {} while (0)
#define wlc_ulmu_oper_state_upd(a, b, c) do {} while (0)
#endif /* defined(WL11AX) && defined(WL_PSMX) */
extern int wlc_ulmu_reclaim_utxd(wlc_info_t *wlc, tx_status_t *txs);
extern bool wlc_ulmu_del_usr(wlc_ulmu_info_t *ulmu, scb_t *scb, bool is_bss_up);
#endif /* _wlc_ulmu_h_ */
