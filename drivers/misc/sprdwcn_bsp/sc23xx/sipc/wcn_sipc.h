/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MDBG_SIPC_H__
#define __MDBG_SIPC_H__
#define _WCN_TXRX_H
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sipc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

#include "bus_common.h"
#include "mdbg_type.h"

#define MDBG_SIPC_WCN_DST		3
#define MDBG_SIPC_MAX_DATA_LEN	4096
#define MDBG_SIPC_LOG_SIZE		(8*1024)
#define MDBG_SIPC_ASSERT_SIZE		1024
#define MDBG_SIPC_LOOPCHECK_SIZE		128
#define MDBG_SIPC_AT_CMD_SIZE		128

#define MDBG_SIPC_BUFID_DEFAULT		0
#define MDBG_SIPC_BUFID_ATCMD		5

#define WCN_SIPC_CHN_TX_NUM 16
#define WCN_SIPC_CHN_RX_NUM 16

#define MDBG_SIPC_LOG_TXBUF_SIZE		0x8000
#define MDBG_SIPC_LOG_RXBUF_SIZE		0x30000

enum mdbg_spipe_num {
	MDBG_SPIPE_ATCMD = 0,
	MDBG_SPIPE_LOOPCHECK,
	MDBG_SPIPE_ASSERT,
	MDBG_SPIPE_LOG,
	MDBG_SPIPE_NUM
};

enum mdbg_sipc_channel {
	MDBG_SIPC_CHANNEL_LOG = 5,
	MDBG_SIPC_CHANNEL_ATCMD = 4,
	MDBG_SIPC_CHANNEL_ASSERT = 12,
	MDBG_SIPC_CHANNEL_LOOPCHECK = 11,
};

struct mdbg_spipe {
	uint8_t			stype;
	uint8_t			dst;
	uint8_t			channel;
	uint8_t			bufid;
	uint32_t			len;
	uint32_t			bufnum;
	uint32_t			txbufsize;
	uint32_t			rxbufsize;
};

struct wcn_sipc_node_t {
	struct list_head node;
	int chn;
	int num;
	struct mbuf_t *mbuf_head;
	struct mbuf_t *mbuf_tail;
};

struct wcn_sipc_info_t {
	struct list_head tx_head;
	struct list_head rx_head;
	spinlock_t tx_spinlock;
	spinlock_t rx_spinlock;
	struct tasklet_struct tx_task;
};

int mdbg_sipc_type_write(void *buf, unsigned int len, unsigned int subtype);
int mdbg_sipc_cb_register(unsigned int subtype, void *func);
int mdbg_sipc_init(void);
void  mdbg_sipc_destroy(void);
#endif
