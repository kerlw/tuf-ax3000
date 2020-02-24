/*
 * wlc_taf_cmn.h
 *
 * This module contains the common definitions for the taf transmit module.
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
 *
 * <<Broadcom-WL-IPTag/Proprietary:>>
 *
 * $Id$
 *
 */

#if !defined(__wlc_taf_cmn_h__)
#define __wlc_taf_cmn_h__

#include <wlc_types.h>
#include <bcmwifi_rspec.h>
#include <wlc.h>
#include <wlc_scb_ratesel.h>
#include <wlc_airtime.h>
#include <wlc_scb.h>

#ifdef PKTQ_LOG
//#define TAF_PKTQ_LOG
#endif // endif

#ifdef TAF_PKTQ_LOG
#include <wlioctl.h>
#endif // endif

/* define scheduler methods support */
#define WLTAF_IAS

#ifndef WLCNTSCB
#error TAF requires WLCNTSCB
#endif // endif

/* define source data support */
#ifdef WLNAR
#define TAF_ENABLE_NAR          1
#else
#define TAF_ENABLE_NAR          0
#endif // endif

#ifdef WLAMPDU
#define TAF_ENABLE_AMPDU        1
#else
#define TAF_ENABLE_AMPDU        0
#endif // endif

#ifdef WL_MU_TX
#define TAF_ENABLE_MU_TX        TAF_ENABLE_AMPDU
#else
#define TAF_ENABLE_MU_TX        0
#endif // endif

#if TAF_ENABLE_MU_TX
#include <wlc_mutx.h>
#define TAF_MAX_MU              4
#endif // endif

#ifdef WLSQS
#define TAF_ENABLE_SQS_PULL     1
#else
#define TAF_ENABLE_SQS_PULL     0
#endif // endif

#ifdef BCMDBG
#define TAF_DBG
#define TAF_DEBUG_VERBOSE
#endif // endif

#if TAF_ENABLE_SQS_PULL
#include <wlc_sqs.h>

#define TAF_DEFAULT_UNIFIED_TID ALLPRIO
#define TAF_TID(_tid, ti)       TAF_DEFAULT_UNIFIED_TID
#endif /* TAF_ENABLE_SQS_PULL */

#if TAF_ENABLE_NAR
#include <wlc_nar.h>
#endif /* TAF_ENABLE_NAR */

//#define TAF_KEEP_FREED_CUBBY

#ifndef TAF_DEFAULT_UNIFIED_TID
#define TAF_DEFAULT_UNIFIED_TID ALLPRIO
#endif // endif

#ifndef TAF_TID
#define TAF_TID(_tid, ti)      (TAF_UNITED_ORDER(ti) ? TAF_DEFAULT_UNIFIED_TID : _tid)
#endif // endif

#ifdef WLAMPDU_PRECEDENCE

#define TAF_MAXPRIO		WLC_PREC_COUNT
#define TAF_NUMPRIO(s)		TAF_MAXPRIO
typedef uint16			taf_traffic_map_t;

#else /* WLAMPDU_PRECEDENCE */

#if TAF_ENABLE_NAR
#define TAF_MAXPRIO		NUMPRIO
#define TAF_NUMPRIO(s)		NUMPRIO
typedef uint8			taf_traffic_map_t;
#else /* TAF_ENABLE_NAR */
#define TAF_MAXPRIO		NUMPRIO
#define TAF_NUMPRIO(s)		NUMPRIO
typedef uint8			taf_traffic_map_t;
#endif /* TAF_ENABLE_NAR */

#endif /* WLAMPDU_PRECEDENCE */

#define TAF_NUM_TIDSTATE	NUMPRIO

#ifdef TAF_DEBUG_VERBOSE
#define _taf_stringify(x...)    #x
#define taf_stringify(x...)     _taf_stringify(x)
#define taf_line_num            taf_stringify(__LINE__)

extern const char* taf_assert_fail;
extern bool taf_trace;
#ifdef DONGLEBUILD
extern bool taf_attach_complete;
#else
#define taf_attach_complete     TRUE
#endif // endif

extern void taf_memtrace_dump(void);

#define TAF_ASSERT(exp)         if (!(exp)) { \
					taf_memtrace_dump(); \
					WL_PRINT(("%s '"taf_stringify(exp)"' %s:" \
						taf_line_num"\n", \
						taf_assert_fail, __FUNCTION__)); \
					OSL_DELAY(400000); \
				} \
			        if (taf_attach_complete) { ASSERT(exp); }

#define TAF_TRACE		if (taf_trace) { WL_PRINT(("trace: %s:"taf_line_num"\n", \
					__FUNCTION__)); }

#else /* TAF_DEBUG_VERBOSE */
#define TAF_TRACE
#define TAF_ASSERT(exp)         ASSERT(exp)
#endif /* TAF_DEBUG_VERBOSE */

typedef struct taf_scb_cubby taf_scb_cubby_t;

/* Max count of total TAF scb cubbies. When reached, reuse the oldest one. Must be >= MAXSCB */
#define TAF_SCB_CNT_MAX         MAXSCB
#define TAF_SCORE_MAX           0xFFFFFFFF
#define TAF_SCORE_MIN           0

#define TAF_WD_STALL_RESET_LIMIT  60  /* 60 times WD 'samples' no pkts in flow, do hard reset */

/* order of scheduler priority */
typedef enum {
#ifdef WLTAF_IAS
#if TAF_ENABLE_SQS_PULL
	TAF_VIRTUAL_MARKUP = -1,
	FIRST_IAS_SCHEDULER = TAF_VIRTUAL_MARKUP,
	TAF_SCHED_FIRST_INDEX = FIRST_IAS_SCHEDULER,
#endif /* TAF_ENABLE_SQS_PULL */
	TAF_EBOS,
	TAF_SCHEDULER_START = TAF_EBOS,
#if !TAF_ENABLE_SQS_PULL
	FIRST_IAS_SCHEDULER = TAF_EBOS,
	TAF_SCHED_FIRST_INDEX = FIRST_IAS_SCHEDULER,
#endif /* !TAF_ENABLE_SQS_PULL */
	TAF_PSEUDO_RR,
	TAF_ATOS,
	TAF_ATOS2,
	LAST_IAS_SCHEDULER = TAF_ATOS2,
#ifndef WLTAF_SCHEDULER_AVAILABLE
#define WLTAF_SCHEDULER_AVAILABLE
#endif // endif
#endif /* WLTAF_IAS */
#ifndef WLTAF_SCHEDULER_AVAILABLE
	TAF_SCHED_FIRST_INDEX,
#endif // endif
	TAF_SCHED_LAST_INDEX,
	NUM_TAF_SCHEDULERS = TAF_SCHED_LAST_INDEX - TAF_SCHED_FIRST_INDEX,
	TAF_UNDEFINED = NUM_TAF_SCHEDULERS,
#ifndef WLTAF_SCHEDULER_AVAILABLE
	TAF_SCHEDULER_START = TAF_UNDEFINED,
	TAF_DEFAULT_SCHEDULER = TAF_UNDEFINED
#endif // endif
#if defined(WLTAF_IAS)
	TAF_DEFAULT_SCHEDULER = TAF_ATOS
#endif // endif
} taf_scheduler_kind;

typedef enum {
#ifdef WLTAF_IAS
	TAF_SCHEDULER_IAS_METHOD,
#endif // endif
	NUM_TAF_SCHEDULER_METHOD_GROUPS,
	TAF_GROUP_UNDEFINED
} taf_scheduler_method_group;

typedef enum {
	TAF_ORDERED_PULL_SCHEDULING,
	TAF_SINGLE_PUSH_SCHEDULING,
	TAF_SCHEME_UNDEFINED
} taf_scheduling_scheme;

typedef enum {
	TAF_LIST_DO_NOT_SCORE,
	TAF_LIST_SCORE_MINIMUM,
	TAF_LIST_SCORE_MAXIMUM,
	TAF_LIST_SCORE_FIXED_INIT_MINIMUM,
	TAF_LIST_SCORE_FIXED_INIT_MAXIMUM
} taf_list_scoring_order_t;

typedef enum {
	TAF_FIRST_REAL_SOURCE,
	/* the order of sources here implicitly defines the ordering priority of that source */
#if TAF_ENABLE_NAR
	/* TAF_SOURCE_NAR are real data packets */
	TAF_SOURCE_NAR = TAF_FIRST_REAL_SOURCE,
#endif // endif
#if TAF_ENABLE_AMPDU
#if TAF_ENABLE_NAR
	/* TAF_SOURCE_AMPDU are real data packets */
	TAF_SOURCE_AMPDU,
#else
	TAF_SOURCE_AMPDU = TAF_FIRST_REAL_SOURCE,
#endif /* TAF_ENABLE_NAR */
#endif /* TAF_ENABLE_AMPDU */
#if TAF_ENABLE_AMPDU || TAF_ENABLE_NAR
	TAF_NUM_REAL_SOURCES,
#else
	TAF_NUM_REAL_SOURCES = TAF_FIRST_REAL_SOURCE,
#endif // endif
#if TAF_ENABLE_SQS_PULL
	/* TAF_SOURCE_HOST_SQS are virtual data packets */
	TAF_SOURCE_HOST_SQS = TAF_NUM_REAL_SOURCES,
	TAF_NUM_SCHED_SOURCES,
#else
	TAF_NUM_SCHED_SOURCES = TAF_NUM_REAL_SOURCES,
#endif // endif

	TAF_SOURCE_UNDEFINED = TAF_NUM_SCHED_SOURCES
} taf_source_type_t;

#define TAF_SOURCE_IS_REAL(s_idx)    ((s_idx) >= TAF_FIRST_REAL_SOURCE && \
					(s_idx) < TAF_NUM_REAL_SOURCES)

#define TAF_SOURCE_IS_VIRTUAL(s_idx) (!TAF_SOURCE_IS_REAL(s_idx))

#if TAF_ENABLE_NAR
#define TAF_SOURCE_IS_NAR(s_idx)     ((s_idx) == TAF_SOURCE_NAR)
#else
#define TAF_SOURCE_IS_NAR(s_idx)     FALSE
#endif // endif

#if TAF_ENABLE_AMPDU
#define TAF_SOURCE_IS_AMPDU(s_idx)   ((s_idx) == TAF_SOURCE_AMPDU)
#else
#define TAF_SOURCE_IS_AMPDU(s_idx)   FALSE
#endif // endif

#if TAF_ENABLE_SQS_PULL
#define TAF_SOURCE_IS_SQS(s_idx)     ((s_idx) == TAF_SOURCE_HOST_SQS)
#else
#define TAF_SOURCE_IS_SQS(s_idx)     FALSE
#endif // endif

#define TAF_MAX_PKT_INDEX            7

typedef enum {
	TAF_ORDER_TID_SCB,
	TAF_ORDER_TID_PARALLEL,
	TAF_ORDER_NUM_OPTIONS
} taf_schedule_order_t;

typedef struct taf_list {
	struct taf_list *next;
	struct taf_list *prev;
	union {
		taf_scb_cubby_t *scb_taf;
	};
} taf_list_t;

typedef enum {
	TAF_RSPEC_SU_DL,
	TAF_RSPEC_SU_DL_INSTANT,
#if TAF_ENABLE_MU_TX
	TAF_RSPEC_MU_DL_INSTANT,
#endif // endif
	NUM_TAF_RSPECS
} taf_rspec_index_t;

typedef union {
	struct {
		uint32 d[16];
	} reserved;
#ifdef WLTAF_IAS
	struct  {
		struct {
			ratespec_t rspec[NUM_TAF_RSPECS];
			uint32     byte_rate[NUM_TAF_RSPECS];
			uint32     pkt_rate[NUM_TAF_RSPECS];
			bool       use[TAF_NUM_SCHED_SOURCES];
			uint8      max_nss;
			uint8      max_bw;
			uint32     timestamp;
			uint32     relative_score;
		} data;
#ifdef TAF_DBG
		struct {
			uint32  ps_enter;
#if !TAF_ENABLE_SQS_PULL
			uint32  skip_ps;
#endif // endif
		} debug;
#endif /* TAF_DBG */
	} ias;
#endif /* WLTAF_IAS */
} taf_scheduler_scb_stats_t;

typedef void* taf_scheduler_tid_info_t;

typedef enum {
#if TAF_ENABLE_MU_TX
	TAF_TECH_DL_HEMUMIMO,
	TAF_TECH_DL_VHMUMIMO,
	TAF_TECH_DL_OFDMA,
	//TAF_TECH_UL_OFDMA,
#endif /* TAF_ENABLE_MU_TX */
	TAF_NUM_MU_TECH_TYPES,
	TAF_TECH_DL_SU = TAF_NUM_MU_TECH_TYPES,
	TAF_NUM_TECH_TYPES,
	TAF_TECH_INVALID = TAF_NUM_TECH_TYPES
} taf_tech_type_t;

#define TAF_TECH_DL_HEMUMIMO_MASK   (1 << TAF_TECH_DL_HEMUMIMO)
#define TAF_TECH_DL_VHMUMIMO_MASK   (1 << TAF_TECH_DL_VHMUMIMO)
#define TAF_TECH_DL_OFDMA_MASK      (1 << TAF_TECH_DL_OFDMA)
#define TAF_TECH_DL_SU_MASK         (1 << TAF_TECH_DL_SU)

#define TAF_TECH_DL_MIMO_MASK       (TAF_TECH_DL_HEMUMIMO_MASK | TAF_TECH_DL_VHMUMIMO_MASK)
#define TAF_TECH_DL_MU_MASK         (TAF_TECH_DL_MIMO_MASK | TAF_TECH_DL_OFDMA_MASK)

#if TAF_ENABLE_MU_TX
#define TAF_TECH_MASK_IS_MUMIMO(tech)  (((tech) & TAF_TECH_DL_MIMO_MASK) != 0)
#define TAF_TECH_MASK_IS_OFDMA(tech)   (((tech) & TAF_TECH_DL_OFDMA_MASK) != 0)
#define TAF_TECH_MASK_IS_SU_ONLY(tech) (((tech) & (TAF_TECH_DL_MIMO_MASK | \
							TAF_TECH_DL_OFDMA_MASK)) == 0)

typedef struct {
	taf_traffic_map_t         enable;
	void*                     scb_h;
} taf_mu_tech_t;

#else

#define TAF_TECH_MASK_IS_MUMIMO(tech)    FALSE
#define TAF_TECH_MASK_IS_OFDMA(tech)     FALSE
#define TAF_TECH_MASK_IS_SU_ONLY(tech)   TRUE
#endif /* TAF_ENABLE_MU_TX */

typedef struct {
	taf_list_t                list;
	taf_link_state_t          linkstate[TAF_NUM_SCHED_SOURCES][TAF_MAXPRIO];
	taf_traffic_map_t         tid_enabled;
	taf_scheduler_tid_info_t  tid_info[TAF_MAXPRIO];
	uint8                     ps_mode;
#if TAF_ENABLE_AMPDU
	uint16                    max_pdu;
#endif // endif
	struct {
		taf_traffic_map_t         map[TAF_NUM_SCHED_SOURCES]; /* bitmask for TID per src */
		taf_traffic_map_t         available; /* global bitmask for all srcs per TID */
		uint16                    count[TAF_NUM_SCHED_SOURCES][TAF_MAXPRIO];
		uint32                    est_units[TAF_MAXPRIO];
		uint32                    est_units_total;
	} traffic;
#if TAF_ENABLE_MU_TX
	uint32                    mu_enable_mask;
	taf_mu_tech_t             mu_tech[TAF_NUM_MU_TECH_TYPES];
	uint8                     mu_type[TAF_MAXPRIO];
#endif // endif
#if TAF_ENABLE_SQS_PULL
	uint32                    pkt_pull_dequeue;
	uint16                    pkt_pull_request[TAF_MAXPRIO];
	taf_traffic_map_t         pkt_pull_map;
#endif /* TAF_ENABLE_SQS_PULL */
	int32                     released_units_limit;
	taf_scheduler_scb_stats_t scb_stats;
} taf_scheduler_info_t;

typedef struct {
	bool             used;
	void *           scbh;
	void *           tidh;
} taf_sched_handle_t;

typedef enum {
	TAF_SCHEDULE_REAL_PACKETS = 0,
	TAF_SCHEDULE_VIRTUAL_PACKETS = TAF_SQS_TRIGGER_TID,
	TAF_MARKUP_REAL_PACKETS = TAF_SQS_V2R_COMPLETE_TID
} taf_schedule_state_t;

typedef struct  {
	taf_scheduler_kind       type;
	taf_schedule_state_t     op_state;
	taf_scheduler_public_t   public;
	int                      tid;
	struct scb*              scb;
	uint32                   actual_release;
	uint32                   actual_release_n;
	uint32                   virtual_release;
	uint32                   pending_release;
	bool                     status_release_fail;
	bool                     legacy_release;
} taf_release_context_t;

typedef struct {
	bool  (*scheduler_fn)     (wlc_taf_info_t *, taf_release_context_t *, void *);
	int   (*watchdog_fn)      (void *);
	int   (*dump_fn)          (void *, struct bcmstrbuf *);
	bool  (*linkstate_fn)     (void *, struct scb *, int, taf_source_type_t, taf_link_state_t);
	int   (*scbstate_fn)      (void *, struct scb *, void *, taf_scb_state_t);
	bool  (*bssstate_fn)      (void *, wlc_bsscfg_t *, void *, taf_bss_state_t);
	bool  (*rateoverride_fn)  (void *, ratespec_t, wlcband_t *);
	int   (*iovar_fn)         (void *, const char *, wl_taf_define_t *, struct bcmstrbuf *);
	bool  (*txstat_fn)        (void *, taf_scb_cubby_t *, int, void *, taf_txpkt_state_t);
	void  (*schedstate_fn)    (void *, taf_scb_cubby_t *, int, int, taf_source_type_t,
		taf_sched_state_t);
#ifdef TAF_PKTQ_LOG
	uint32 (*dpstats_log_fn)  (void *, taf_scb_cubby_t *, mac_log_counters_v06_t*, bool,
		uint32, uint32, uint32);
#endif // endif
} taf_scheduler_fn_t;

typedef struct taf_method_info {
	taf_scheduler_kind         type;
	const char*                name;
#ifdef TAF_DBG
	const char*                dump_name;
#endif // endif
	taf_scheduling_scheme      scheme;
	taf_list_scoring_order_t   ordering;
	taf_scheduler_method_group group;
	taf_scheduler_fn_t         funcs;
	struct wlc_taf_info*       taf_info;
	taf_list_t*                list;
	taf_traffic_map_t*         ready_to_schedule;
	uint32                     counter;
	uint32                     score_init;

	union {
		void*                        reserved;
#ifdef WLTAF_IAS
		struct taf_ias_method_info*  ias;
#endif // endif
	};
} taf_method_info_t;

/*
 * Per SCB data, malloced, to which a pointer is stored in the SCB cubby.
 *
 * This struct is malloc'ed when initializing the scb, but it is not free with the scb!
 * It will be kept in the "pool" list to save the preferences assigned to the STA, and
 * reused if the STA associates again to the AP.
 */
struct taf_scb_cubby {
	struct taf_scb_cubby* next;        /* Next cubby for TAF list. Must be first */
	struct scb *          scb;        /* Back pointer */
	uint32                score;      /* score for scheduler ordering: constant for EBOS and
	                                   * dynamic otherwise
					   */
	uint32                force;      /* Force sending traffic */
	uint32                timestamp;  /* misc timestamp */
	taf_method_info_t*    method;     /* Which scheduling method owns the SCB */
	struct ether_addr     ea;         /* Copy of MAC to find back reassociated STA */
	taf_scheduler_info_t  info;       /* keep in last place */
};

typedef struct {
	void*  (*scb_h_fn)       (void *, struct scb *);
	void*  (*tid_h_fn)       (void *, int);
	uint16 (*pktqlen_fn)     (void *, void *);
	bool   (*release_fn)     (void *, void *, void *, bool, taf_scheduler_public_t *);
} taf_support_funcs_t;

#if TAF_ENABLE_MU_TX
typedef struct {
	void*  (*scb_h_fn)       (void *, struct scb *);
} taf_tech_funcs_t;
#endif /* TAF_ENABLE_MU_TX */

/* TAF per interface context */
struct wlc_taf_info {
	wlc_info_t           *wlc;       /* Back link to wlc */
	bool                 enabled;    /* On/Off switch */
	bool                 bypass;     /* taf bypass (soft taf disable) */
	int                  scb_handle; /* Offset for scb cubby */
	taf_scheduler_kind   default_scheduler;
	uint32               default_score;
	taf_scb_cubby_t      *head;      /* Ordered list of associated STAs */
#ifdef TAF_KEEP_FREED_CUBBY
	taf_scb_cubby_t      *pool;      /* Backup for disassociated STAs cubbies */
#endif // endif
	uint32               scheduler_index;
	uint32               super;
	uint32               scb_cnt;    /* Total count of allocated cubbies */
	void*                scheduler_context[NUM_TAF_SCHEDULERS];
	void*                group_context[NUM_TAF_SCHEDULER_METHOD_GROUPS];
	uint8                group_use_count[NUM_TAF_SCHEDULER_METHOD_GROUPS];
	int8                 scheduler_nest;
	void**               source_handle_p[TAF_NUM_SCHED_SOURCES];
	uint32               use_sampled_rate_sel;
#if TAF_ENABLE_MU_TX
	uint32               mu;
	void**               tech_handle_p[TAF_NUM_MU_TECH_TYPES];
	taf_tech_funcs_t     tech_fn[TAF_NUM_MU_TECH_TYPES];
#endif // endif
	taf_support_funcs_t  funcs[TAF_NUM_SCHED_SOURCES];

	uint32               force_time;
	uint32               release_count;
	uint16               watchdog_data_stall;
	uint16               watchdog_data_stall_limit;
	taf_schedule_order_t ordering;
#if !TAF_ENABLE_SQS_PULL
	taf_schedule_order_t pending_ordering;
#endif // endif
	taf_schedule_state_t op_state;
	uint32               last_scheduler_run;
#if TAF_ENABLE_SQS_PULL
	uint32               eops_rqst;
	uint32               total_pull_requests;
#ifdef TAF_DBG
	uint32               virtual_complete_time;
#endif /* TAF_DBG */
#endif /* TAF_ENABLE_SQS_PULL */
};

#if TAF_ENABLE_SQS_PULL
#define TAF_UNITED_ORDER(_taf)      (TRUE)
#define TAF_PARALLEL_ORDER(_taf)    (FALSE)
#else
#define TAF_UNITED_ORDER(_taf)      ((_taf)->ordering == TAF_ORDER_TID_SCB)
#define TAF_PARALLEL_ORDER(_taf)    ((_taf)->ordering == TAF_ORDER_TID_PARALLEL)
#endif /* TAF_ENABLE_SQS_PULL */

#define TAF_METHOD_GROUP_INFO(_m)  (((_m)->taf_info)->group_context[(_m)->group])
#define TAF_CUBBY_TIDINFO(_s, _t)  ((_s)->info.tid_info[(_t)])

#ifdef WLTAF_IAS
#define TAF_TYPE_IS_IAS(type)  (type >= FIRST_IAS_SCHEDULER && type <= LAST_IAS_SCHEDULER)
#else
#define TAF_TYPE_IS_IAS(type)  (FALSE)
#endif // endif

#define SCB_TAF_CUBBY_PTR(info, scb)    ((taf_scb_cubby_t **)(SCB_CUBBY((scb), (info)->scb_handle)))
#define SCB_TAF_CUBBY(taf_info, scb)    (*SCB_TAF_CUBBY_PTR(taf_info, scb))

#define TAF_WLCT(_T)                    ((_T)->wlc)
#define TAF_WLCM(_M)                    ((_M) ? TAF_WLCT((_M)->taf_info) : NULL)

#define TAF_ETHERS(s)                   ETHER_TO_MACF((s)->ea)
#define TAF_ETHERC(cub)                 TAF_ETHERS((cub)->scb)

#ifdef TAF_DBG
#define WL_TAFM(_M, format, ...)	WL_TAFF(TAF_WLCM(_M), "(%s) "format, \
						TAF_SCHED_NAME(_M), ##__VA_ARGS__)

#define WL_TAFT(_T, ...)		WL_TAFF(TAF_WLCT(_T), ##__VA_ARGS__)
#define WL_TAFW(_W, ...)		WL_TAFF((_W), ##__VA_ARGS__)
#else
#define WL_TAFM(_M, format, ...)
#define WL_TAFT(_T, ...)
#define WL_TAFW(_W, ...)
#endif /* TAF_DBG */

#define TAF_TIME_FORCE_DEFAULT  0

#define TAF_IOVAR_OK_SET        (BCME_OK + 1)
#define TAF_IOVAR_OK_GET        (BCME_OK + 2)

#ifdef WLTAF_IAS
void* BCMATTACHFN(wlc_taf_ias_method_attach)(wlc_taf_info_t *taf_info, taf_scheduler_kind type);
int   BCMATTACHFN(wlc_taf_ias_method_detach)(void* context);
#endif /* WLTAF_IAS */

extern int
wlc_taf_param(const char** cmd, uint32* param, uint32 min, uint32 max, struct bcmstrbuf* b);

extern const char* wlc_taf_ordering_name[TAF_ORDER_NUM_OPTIONS];
extern const char* taf_undefined_string;
extern const char* taf_tx_sources[TAF_NUM_SCHED_SOURCES + 1];

#ifdef TAF_DBG
extern const char* taf_rel_complete_text[NUM_TAF_REL_COMPLETE_TYPES];
extern const char* taf_link_states_text[NUM_TAF_LINKSTATE_TYPES];
extern const char* taf_scb_states_text[NUM_TAF_SCBSTATE_TYPES];
extern const char* taf_txpkt_status_text[TAF_NUM_TXPKT_STATUS_TYPES];
#if TAF_ENABLE_MU_TX
extern const char* taf_mutech_text[TAF_NUM_TECH_TYPES];
#endif /* TAF_ENABLE_MU_TX */
#endif /* TAF_DBG */

extern void BCMFASTPATH
wlc_taf_sort_list(taf_list_t ** source_head, taf_list_scoring_order_t ordering);

void BCMFASTPATH wlc_taf_list_add(taf_list_t** head, taf_list_t* item);
void BCMFASTPATH wlc_taf_list_remove(taf_list_t** head, taf_list_t* item);
void BCMFASTPATH wlc_taf_list_insert(taf_list_t* parent, taf_list_t* item);

taf_source_type_t wlc_taf_get_source_index(const void * data);

bool wlc_taf_marked_up(wlc_taf_info_t *taf_info);

//extern bool wlc_taf_rawfb(wlc_taf_info_t* taf_info);
//extern uint32 wlc_taf_schedule_period(wlc_taf_info_t* taf_info, int tid);

#define TAF_SCHED_NAME(method)    taf_get_sched_name(method)
#define TAF_DUMP_NAME(method)     taf_get_dump_name(method)
#define TAF_SOURCE_NAME(s_index)  ((s_index >= 0 && s_index < TAF_NUM_SCHED_SOURCES) ? \
					(const char*)taf_tx_sources[s_index] : \
					(const char*)taf_tx_sources[TAF_SOURCE_UNDEFINED])

static INLINE BCMFASTPATH uint32 taf_timestamp(wlc_info_t* wlc)
{
	if (wlc->pub->up) {
		uint32 _t_timestamp = R_REG(wlc->osh, D11_TSFTimerLow(wlc));
		/* '0 time' is used some places to init state logic, small error here to avoid
		 * rare issue
		 */
		return _t_timestamp ? _t_timestamp : 1;
	}
	return 0;
}

static INLINE BCMFASTPATH
taf_method_info_t* taf_get_method_info(wlc_taf_info_t* taf_info, taf_scheduler_kind sched)
{
	if ((sched >= TAF_SCHED_FIRST_INDEX) && (sched < TAF_SCHED_LAST_INDEX)) {
		int index = (int)sched - (int)TAF_SCHED_FIRST_INDEX;
		return (taf_method_info_t*)(taf_info->scheduler_context[index]);
	}
	return NULL;
}

static INLINE ratespec_t BCMFASTPATH
taf_get_rate(wlc_taf_info_t *taf_info, struct scb *scb, taf_rspec_index_t rindex)
{
	wlc_info_t* wlc = TAF_WLCT(taf_info);

	switch (rindex) {
		case TAF_RSPEC_SU_DL:
			return wlc_scb_ratesel_get_primary(wlc, scb, NULL);
			break;
		case TAF_RSPEC_SU_DL_INSTANT:
#if TAF_ENABLE_MU_TX
		case TAF_RSPEC_MU_DL_INSTANT:
#endif // endif
			return wlc_scb_ratesel_get_opstats(wlc->wrsi, scb, AC_BE,
				RTSTAT_GET_TXS | ((rindex == TAF_RSPEC_SU_DL_INSTANT ?
				RTSTAT_GET_TYPE_SU : RTSTAT_GET_TYPE_MU) << RTSTAT_GET_TYPE_SHIFT));
			break;
		default:
			TAF_ASSERT(0);
	}
	return 0;
}

static INLINE void BCMFASTPATH
taf_rate_to_taf_units(wlc_taf_info_t *taf_info, taf_scb_cubby_t* scb_taf, uint32 tech)
{
	taf_rspec_index_t rindex;
	ratespec_t rspec;
	taf_scheduler_scb_stats_t* scb_stats = &scb_taf->info.scb_stats;

#if TAF_ENABLE_MU_TX
	tech &= taf_info->use_sampled_rate_sel & (taf_info->mu | TAF_TECH_DL_SU_MASK);
#else
	tech &= taf_info->use_sampled_rate_sel & TAF_TECH_DL_SU_MASK;
#endif // endif

	for (rindex = TAF_RSPEC_SU_DL; rindex < NUM_TAF_RSPECS; rindex++) {

		if (rindex == TAF_RSPEC_SU_DL_INSTANT && ((tech & TAF_TECH_DL_SU_MASK) == 0)) {
			continue;
		}
#if TAF_ENABLE_MU_TX
		if (rindex == TAF_RSPEC_MU_DL_INSTANT && ((tech & TAF_TECH_DL_MU_MASK) == 0)) {
			continue;
		}
#endif // endif
		rspec = taf_get_rate(taf_info, scb_taf->scb, rindex);

		if (scb_stats->ias.data.rspec[rindex] != rspec) {

			if (rindex == TAF_RSPEC_SU_DL) {
				scb_taf->info.scb_stats.ias.data.max_nss = wlc_ratespec_nss(rspec);
				scb_taf->info.scb_stats.ias.data.max_bw = wlc_ratespec_bw(rspec);
			}

			scb_stats->ias.data.rspec[rindex] = rspec;

			scb_stats->ias.data.byte_rate[rindex] = wlc_airtime_payload_time_us(0,
				rspec, TAF_PKTBYTES_COEFF);

			scb_stats->ias.data.pkt_rate[rindex] =
				scb_stats->ias.data.byte_rate[rindex] *
				wlc_airtime_dot11hdrsize(scb_taf->scb->wsec);
		}
	}
}

static INLINE const char* taf_get_sched_name(taf_method_info_t* method)
{
	if (method && method->name) {
		return method->name;
	}
	return taf_undefined_string;
}

static INLINE const char* taf_get_dump_name(taf_method_info_t* method)
{
#ifdef TAF_DBG
	if (method && method->dump_name) {
		return method->dump_name;
	}
#endif /* TAF_DBG */
	return taf_undefined_string;
}

static INLINE bool taf_is_bypass(wlc_taf_info_t *taf_info)
{
	bool is_bypass = TRUE;

	if (taf_info && !taf_info->bypass) {
		return FALSE;
	}
	return is_bypass;
}

static INLINE bool taf_enabled(wlc_taf_info_t* taf_info)
{
	bool result = (taf_info != NULL) ? taf_info->enabled : FALSE;
	return result;
}

static INLINE bool taf_in_use(wlc_taf_info_t* taf_info)
{
	bool result = taf_enabled(taf_info) && !taf_is_bypass(taf_info);
	return result;
}

static INLINE uint32 BCMFASTPATH taf_nss(taf_scb_cubby_t *scb_taf)
{
	uint32 nss = scb_taf->info.scb_stats.ias.data.max_nss;
	TAF_ASSERT(nss > 0);
	return nss;
}

static INLINE uint32 BCMFASTPATH taf_bw(taf_scb_cubby_t *scb_taf)
{
	uint32 bw = scb_taf->info.scb_stats.ias.data.max_bw;
	TAF_ASSERT(bw > 0);
	return bw;
}

#endif /* __wlc_taf_cmn_h__ */
