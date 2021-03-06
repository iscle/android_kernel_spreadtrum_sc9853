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


#ifndef _MDBG_COMMON_H
#define _MDBG_COMMON_H
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

#include "mdbg_type.h"

#define MDBG_SDIO_PACKER_TYPE		3

enum {
	MDBG_SUBTYPE_RING	= 0,
	MDBG_SUBTYPE_LOOPCHECK,
	MDBG_SUBTYPE_AT,
	MDBG_SUBTYPE_ASSERT,
};

#define MDBG_CACHE_FLAG_VALUE	(0xcdcddcdc)

#define MDBG_SHARE_MEMORY_SIZE		(3*1024*1024)
#define MDBG_CP_IRAM_DATA_NUM		8192
#define DUMP_IRAM_START_ADDR		0x10000000
#define DUMP_GNSS_IRAM_START_ADDR	0X18004000

#define DUMP_STR_TYPE_DATA		0
#define DUMP_STR_TYPE_REG		1

#define WCN_CP2_STATUS_DUMP_REG	0x6a6b6c6d

/* wcn registers start */
#define DUMP_REG_BTWF_INTC_ADDR		0x40010000
#define DUMP_REG_BTWF_INTC_LEN		0x38

#define DUMP_REG_BTWF_SYSTEM_TIMER_ADDR		0x40020000
#define DUMP_REG_BTWF_SYSTEM_TIMER_LEN		0x10

#define DUMP_REG_BTWF_TIMER0_ADDR		0x40030000
#define DUMP_REG_BTWF_TIMER0_LEN		0x20

#define DUMP_REG_BTWF_TIMER1_ADDR		0x40030020
#define DUMP_REG_BTWF_TIMER1_LEN		0x20

#define DUMP_REG_BTWF_TIMER2_ADDR		0x40030040
#define DUMP_REG_BTWF_TIMER2_LEN		0x20

#define DUMP_REG_BTWF_WATCHDOG_ADDR		0x40040000
#define DUMP_REG_BTWF_WATCHDOG_LEN		0x24

#define DUMP_REG_BTWF_CTRL_ADDR		0x40060000
#define DUMP_REG_BTWF_CTRL_LEN		0x300

#define DUMP_REG_BTWF_DMA_CTRL_ADDR		0x60200000
#define DUMP_REG_BTWF_DMA_CTRL_LEN		0x3000

#define DUMP_REG_BTWF_AHB_CTRL_ADDR		0x60300000
#define DUMP_REG_BTWF_AHB_CTRL_LEN		0x400

#define DUMP_REG_COM_AHB_CTRL_ADDR		0xd0010000
#define DUMP_REG_COM_AHB_CTRL_LEN		0xb4

#define DUMP_REG_MANU_CLK_CTRL_ADDR		0xd0020800
#define DUMP_REG_MANU_CLK_CTRL_LEN		0xc

#define DUMP_REG_WIFI_ADDR		0x70000000
#define DUMP_REG_WIFI_LEN		0x10000

#define DUMP_REG_FM_ADDR		0x400b0000
#define DUMP_REG_FM_LEN			0x850

#define DUMP_REG_BT_CMD_ADDR	0x60700000
#define DUMP_REG_BT_CMD_LEN		0x400

#define DUMP_REG_BT_ADDR		0x60740000
#define DUMP_REG_BT_LEN			0xa400

/* wcn registers end*/

/* ap aon registers start*/
#define DUMP_REG_PMU_SLEEP_CTRL		0x402B00CC
#define DUMP_REG_PMU_SLEEP_STATUS		0x402B00D4
#define DUMP_REG_PMU_PD_WCN_SYS_CFG		0x402B0100
#define DUMP_REG_PMU_PD_WIFI_WRAP_CFG		0x402B0104
#define DUMP_REG_PMU_WCN_SYS_DSLP_ENA		0x402B0244
#define DUMP_REG_PMU_WIFI_WRAP_DSLP_ENA		0x402B0248
#define DUMP_REG_AON_APB_WCN_SYS_CFG2		0x402E057C
/* ap aon registers end*/

#define MDBG_RX_RING_SIZE		(2*1024*1024)
#define DUMP_PACKET_SIZE	(1024)
#define DUMP_WAIT_TIMEOUT	(100)
#define DUMP_WAIT_COUNT	(40)

#define SMP_HEADERFLAG 0X7E7E7E7E
#define SMP_RESERVEDFLAG 0X5A5A
#define SMP_DSP_CHANNEL_NUM 0X88
#define SMP_DSP_TYPE 0X9D
#define SMP_DSP_DUMP_TYPE 0X32

#define SYSNC_CODE_LEN 0X4
#define CHKSUM_LEN 0X2
#define ARMLOG_HEAD 9

#define SMP_HEAD_STR "at+smphead="

struct ring_rx_data {
	unsigned char		*addr;
	unsigned int		len;
	unsigned int		fifo_id;
	struct list_head	entry;
};

struct ring_device {
	struct mdbg_ring_t	*ring;
	struct wake_lock	rw_wake_lock;
	spinlock_t		rw_lock;
	struct mutex mdbg_read_mutex;
	struct list_head        rx_head;
	struct tasklet_struct   rx_task;
};

struct sme_head_tag {
	unsigned int seq_num;
	unsigned short len;
	unsigned char type;
	unsigned char subtype;
};

struct smp_head {
	unsigned int sync_code;
	unsigned short length;
	unsigned char channel_num;
	unsigned char packet_type;
	unsigned short reserved;
	unsigned short check_sum;
};

enum smp_diag_subtype_t {
	NORMAL_INFO = 0X0,
	DUMP_MEM_DATA,
	DUMP_MEM_END,
};

int mdbg_comm_init(void);
void mdbg_comm_remove(void);
long int mdbg_send(char *buf, long int len, unsigned int subtype);
long int mdbg_receive(void *buf, long int len);
int mdbg_dump_mem(void);
int mdbg_pt_common_reg(unsigned int subtype, void *func);
long mdbg_content_len(void);
void mdbg_clear_log(void);

int mdbg_read_release(unsigned int fifo_id);
unsigned int wcn_get_carddump_status(void);
unsigned int wcn_set_carddump_status(unsigned int flag);

unsigned long long mdbg_get_rx_total_cnt(void);
void wcn_hold_cpu(void);
void mdbg_hold_cpu(void);
bool mdbg_get_download_status(void);
int mdbg_get_module_status(void);
int mdbg_get_module_status_changed(void);
int mdbg_snap_shoot_iram(void *buf);
#endif
