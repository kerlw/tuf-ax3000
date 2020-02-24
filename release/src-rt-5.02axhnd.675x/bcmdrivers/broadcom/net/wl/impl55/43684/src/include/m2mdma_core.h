/*
 * BCM43XX M2M DMA core hardware definitions.
 *
 * Copyright (C) 2019, Broadcom. All Rights Reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * <<Broadcom-WL-IPTag/Proprietary:>>
 *
 * $Id:m2mdma _core.h 421139 2013-08-30 17:56:15Z kiranm $
 */

#ifndef	_M2MDMA_CORE_H
#define	_M2MDMA_CORE_H
#include <sbhnddma.h>
/* cpp contortions to concatenate w/arg prescan */
#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif // endif

/* dma regs to control the flow between host2dev and dev2host  */
typedef struct m2m_devdmaregs {
	dma64regs_t	tx;
	uint32 		PAD[2];
	dma64regs_t	rx;
	uint32 		PAD[2];
} m2m_devdmaregs_t;

typedef struct dmaintregs {
	uint32 intstatus;
	uint32 intmask;
} dmaintregs_t;

/* rx header */
typedef volatile struct {
	uint16 len;
	uint16 flags;
} m2md_rxh_t;

/*
 * Software counters (first part matches hardware counters)
 */

typedef volatile struct {
	uint32 rxdescuflo;	/* receive descriptor underflows */
	uint32 rxfifooflo;	/* receive fifo overflows */
	uint32 txfifouflo;	/* transmit fifo underflows */
	uint32 runt;		/* runt (too short) frames recv'd from bus */
	uint32 badlen;		/* frame's rxh len does not match its hw tag len */
	uint32 badcksum;	/* frame's hw tag chksum doesn't agree with len value */
	uint32 seqbreak;	/* break in sequence # space from one rx frame to the next */
	uint32 rxfcrc;		/* frame rx header indicates crc error */
	uint32 rxfwoos;		/* frame rx header indicates write out of sync */
	uint32 rxfwft;		/* frame rx header indicates write frame termination */
	uint32 rxfabort;	/* frame rx header indicates frame aborted */
	uint32 woosint;		/* write out of sync interrupt */
	uint32 roosint;		/* read out of sync interrupt */
	uint32 rftermint;	/* read frame terminate interrupt */
	uint32 wftermint;	/* write frame terminate interrupt */
} m2md_cnt_t;

/* SB side: M2M DMA core registers */
typedef struct m2mregs {
	uint32 control;		/* Core control 0x0 */
	uint32 capabilities;	/* Core capabilities 0x4 */
	uint32 intcontrol;	/* Interrupt control 0x8 */
	uint32 PAD[5];
	uint32 intstatus;	/* Interrupt Status  0x20 */
	uint32 PAD[3];
	dmaintregs_t intregs[8]; /* 0x30 - 0x6c */
	uint32 PAD[36];
	uint32 intrxlazy[8];	/* 0x100 - 0x11c */
	uint32 PAD[48];
	uint32 clockctlstatus;  /* 0x1e0 */
	uint32 workaround;	/* 0x1e4 */
	uint32 powercontrol;	/* 0x1e8 */
	uint32 PAD[5];
	m2m_devdmaregs_t dmaregs[8]; /* 0x200 - 0x3f4 */
} m2md_regs_t;

/** m2md_regs_t::capabilities */
#define M2MD_CAPABILITIES_CHANNELCNT_NBITS 4
#define M2MD_CAPABILITIES_CHANNELCNT_SHIFT 0
#define M2MD_CAPABILITIES_CHANNELCNT_MASK \
	BCM_MASK(M2MD_CAPABILITIES_CHANNELCNT)

#define M2MD_CAPABILITIES_MAXBURSTLEN_NBITS 3
#define M2MD_CAPABILITIES_MAXBURSTLEN_SHIFT 4
#define M2MD_CAPABILITIES_MAXBURSTLEN_MASK \
	BCM_MASK(M2MD_CAPABILITIES_MAXBURSTLEN)

#define M2MD_CAPABILITIES_MAXREADSOUTSTANDING_NBITS 3
#define M2MD_CAPABILITIES_MAXREADSOUTSTANDING_SHIFT 7
#define M2MD_CAPABILITIES_MAXREADSOUTSTANDING_MASK \
	BCM_MASK(M2MD_CAPABILITIES_MAXREADSOUTSTANDING)

#define M2MD_CAPABILITIES_SM2MCNT_NBITS 4
#define M2MD_CAPABILITIES_SM2MCNT_SHIFT 10
#define M2MD_CAPABILITIES_SM2MCNT_MASK \
	BCM_MASK(M2MD_CAPABILITIES_SM2MCNT)

/** dmaintregs_t intstatus, intmask for traditional and simple m2m channels */
/* Ch# <IntStatus,IntMask> DescErr (DE) */
#define M2MD_CH_DE_NBITS        1
#define M2MD_CH_DE_SHIFT        10
#define M2MD_CH_DE_MASK         BCM_MASK(M2MD_CH_DE)
/* Ch# <IntStatus,IntMask> DataErr (DA) */
#define M2MD_CH_DA_NBITS        1
#define M2MD_CH_DA_SHIFT        11
#define M2MD_CH_DA_MASK         BCM_MASK(M2MD_CH_DA)
/* Ch# <IntStatus,IntMask> DescProtoErr (DP) */
#define M2MD_CH_DP_NBITS        1
#define M2MD_CH_DP_SHIFT        12
#define M2MD_CH_DP_MASK         BCM_MASK(M2MD_CH_DP)
/* Ch# <IntStatus,IntMask> RcvDescUf (RU) */
#define M2MD_CH_RU_NBITS        1
#define M2MD_CH_RU_SHIFT        13
#define M2MD_CH_RU_MASK         BCM_MASK(M2MD_CH_RU)
/* Ch# <IntStatus,IntMask> RcvInt (RI) */
#define M2MD_CH_RI_NBITS        1
#define M2MD_CH_RI_SHIFT        16
#define M2MD_CH_RI_MASK         BCM_MASK(M2MD_CH_RI)
/* Ch# <IntStatus,IntMask> XmtInt (XI) */
#define M2MD_CH_XI_NBITS        1
#define M2MD_CH_XI_SHIFT        24
#define M2MD_CH_XI_MASK         BCM_MASK(M2MD_CH_XI)

#endif	/* _M2MDMA_CORE_H */
