/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <soc/sprd/wcn_bus.h>

#include "wcn_integrate.h"
#include "wcn_sipc.h"
#include "wcn_txrx.h"

static struct wcn_sipc_info_t *sipc_info;
static int g_dump_status;

/* stype:dst:channel:bufid */
static struct mdbg_spipe g_mdbg_spipe[MDBG_SPIPE_NUM] = {
	{
		.stype = MDBG_SPIPE_ATCMD,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_ATCMD,
		.bufid = MDBG_SIPC_BUFID_ATCMD,
		.len = MDBG_SIPC_AT_CMD_SIZE,
		.bufnum = 16,
		.txbufsize = 0x1000,
		.rxbufsize = 0x1000,
	},
	{
		.stype = MDBG_SPIPE_LOOPCHECK,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_LOOPCHECK,
		.bufid = MDBG_SIPC_BUFID_DEFAULT,
		.len = MDBG_SIPC_LOOPCHECK_SIZE,
		.bufnum = 1,
		.txbufsize = 0x400,
		.rxbufsize = 0x400,
	},
	{
		.stype = MDBG_SPIPE_ASSERT,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_ASSERT,
		.bufid = MDBG_SIPC_BUFID_DEFAULT,
		.len = MDBG_SIPC_ASSERT_SIZE,
		.bufnum = 1,
		.txbufsize = 0x400,
		.rxbufsize = 0x400,
	},
	{
		.stype = MDBG_SPIPE_LOG,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_LOG,
		.bufid = MDBG_SIPC_BUFID_DEFAULT,
		.len = MDBG_SIPC_LOG_SIZE,
		.bufnum = 1,
		.txbufsize = MDBG_SIPC_LOG_TXBUF_SIZE,
		.rxbufsize = MDBG_SIPC_LOG_RXBUF_SIZE,
	},
};

static inline struct wcn_sipc_info_t *wcn_sipc_get_info(void)
{
	return sipc_info;
}

static u32 wcn_sipc_spipe_to_channel(int spipe_channel)
{
	u32 channel = CHN_MAX_NUM;

	switch (spipe_channel) {
	case MDBG_SIPC_CHANNEL_ATCMD:
		channel = WCN_AT_RX;
		break;
	case MDBG_SIPC_CHANNEL_LOOPCHECK:
		channel = WCN_LOOPCHECK_RX;
		break;
	case MDBG_SIPC_CHANNEL_ASSERT:
		channel = WCN_ASSERT_RX;
		break;
	case MDBG_SIPC_CHANNEL_LOG:
		channel = WCN_RING_RX;
		break;
	default:
		break;
	}

	return channel;
}

static struct mdbg_spipe *wcn_sipc_get_chn_spipe(int channel)
{
	struct mdbg_spipe *p_spipe = NULL;

	if (channel > WCN_RING_RX) {
		WCN_ERR("Received channel is invalid(channel=%d)\n", channel);
		return NULL;
	}

	switch (channel) {
	case WCN_AT_TX:
		p_spipe = &g_mdbg_spipe[MDBG_SPIPE_ATCMD];
		break;
	case WCN_LOOPCHECK_RX:
		p_spipe = &g_mdbg_spipe[MDBG_SPIPE_LOOPCHECK];
		break;
	case WCN_AT_RX:
		p_spipe = &g_mdbg_spipe[MDBG_SPIPE_ATCMD];
		break;
	case WCN_ASSERT_RX:
		p_spipe = &g_mdbg_spipe[MDBG_SPIPE_ASSERT];
		break;
	case WCN_RING_RX:
		p_spipe = &g_mdbg_spipe[MDBG_SPIPE_LOG];
		break;
	default:
		break;
	}

	return p_spipe;
}


static int wcn_sipc_write(int chn, void *buf, int count)
{
	int cnt = -ENODEV;
	struct mdbg_spipe *p_spipe;

	p_spipe = wcn_sipc_get_chn_spipe(chn);
	if (p_spipe) {
		WCN_INFO("sipc write channel %d count=%d\n",
					p_spipe->channel, count);
		cnt = sbuf_write(p_spipe->dst, p_spipe->channel,
				 p_spipe->bufid, buf, count, -1);
	}

	return cnt;
}

static int wcn_sipc_rx_handle(struct  mdbg_spipe *p_spipe, void *buf, int len)
{
	int ret = -1;
	struct mbuf_t *mbuf_node;
	struct mchn_ops_t *wcn_sipc_ops = NULL;
	struct wcn_sipc_node_t *rx_info;
	struct wcn_sipc_info_t *chn_info = wcn_sipc_get_info();

	mbuf_node = kmalloc(sizeof(struct mbuf_t), GFP_KERNEL);
	if (!mbuf_node) {
		ret =  -ENOMEM;
		goto err1;
	}

	rx_info = kmalloc(sizeof(struct wcn_sipc_node_t), GFP_KERNEL);
	if (!rx_info) {
		ret = -ENOMEM;
		goto err2;
	}

	spin_lock_bh(&chn_info->rx_spinlock);
	mbuf_node->buf = (char *)buf;
	mbuf_node->len = len;
	mbuf_node->next = NULL;

	rx_info->mbuf_head = mbuf_node;
	rx_info->mbuf_tail = mbuf_node;
	rx_info->num = 1;
	rx_info->chn = wcn_sipc_spipe_to_channel(p_spipe->channel);
	list_add_tail(&rx_info->node, &chn_info->rx_head);
	spin_unlock_bh(&chn_info->rx_spinlock);

	wcn_sipc_ops = chn_ops(rx_info->chn);
	if (wcn_sipc_ops)
		wcn_sipc_ops->pop_link(rx_info->chn, rx_info->mbuf_head,
				       rx_info->mbuf_tail, rx_info->num);
	else {
		WCN_ERR("no rx ops,chn:%d\n", rx_info->chn);
		kfree(buf);
		kfree(mbuf_node);
		kfree(rx_info);
	}

	return ret;
err2:
	kfree(mbuf_node);
err1:
	kfree(buf);
	WCN_ERR("rx error chn:%d, ret:%d\n",
		p_spipe->channel, ret);
	WARN_ON(1);

	return ret;
}


static void wcn_sipc_handler (int event, void *data)
{
	struct  mdbg_spipe *p_spipe = data;
	int cnt = 0;
	unsigned char *buf;
	uint8_t stype;
	struct bus_puh_t *puh = NULL;

	if (!p_spipe)
		return;
	stype = p_spipe->stype;
	if (stype >= MDBG_SPIPE_NUM)
		return;

	switch (event) {
	case SBUF_NOTIFY_WRITE:
		break;
	case SBUF_NOTIFY_READ:
		buf = kzalloc(p_spipe->len + PUB_HEAD_RSV, GFP_KERNEL);
		if (!buf)
			return;
		cnt = sbuf_read(p_spipe->dst, p_spipe->channel, p_spipe->bufid,
				(void *)(buf + PUB_HEAD_RSV), p_spipe->len, 0);
		puh = (struct bus_puh_t *)buf;
		puh->len = cnt;
		WCN_DEBUG("sipc read channel %d count=%d\n",
					p_spipe->channel, cnt);
		if (cnt > 0)
			wcn_sipc_rx_handle(p_spipe, buf, cnt);
		else {
			kfree(buf);
			WCN_ERR("read cnt invalid!cnt %d\n", cnt);
		}
		break;
	default:
		WCN_ERR("received event is invalid(event=%d)\n", event);
	}
}

static int wcn_sipc_chn_init(struct mchn_ops_t *ops)
{
	int ret = 0;
	struct mdbg_spipe *p_spipe;

	p_spipe = wcn_sipc_get_chn_spipe(ops->channel);
	if (!p_spipe)
		return -1;
	if (p_spipe->stype != MDBG_SPIPE_ATCMD) {
		ret = sbuf_create(p_spipe->dst, p_spipe->channel,
				  p_spipe->bufnum,
				  p_spipe->txbufsize,
				  p_spipe->rxbufsize);
		if (ret < 0) {
			WCN_ERR("chn:%d sbuf fail!\n",
				ops->channel);
			return -1;
		}
	}

	ret = bus_chn_init(ops, HW_TYPE_SIPC);
	if (ret)
		return ret;

	ret = sbuf_register_notifier(p_spipe->dst,
				     p_spipe->channel,
				     p_spipe->bufid,
				     wcn_sipc_handler,
				     p_spipe);
	WCN_INFO("chn:%d sbuf success!\n", ops->channel);

	return ret;
}

static int wcn_sipc_chn_deinit(struct mchn_ops_t *ops)
{
	struct mdbg_spipe *p_spipe;
	int ret = 0;

	p_spipe = wcn_sipc_get_chn_spipe(ops->channel);
	if (p_spipe) {
		sbuf_destroy(p_spipe->dst, p_spipe->channel);
		WCN_INFO("chn:%d  success!\n",
			p_spipe->channel);

		ret = bus_chn_deinit(ops);
	}

	return ret;
}

static int wcn_sipc_buf_list_alloc(int chn, struct mbuf_t **head,
			       struct mbuf_t **tail, int *num)
{
	return buf_list_alloc(chn, head, tail, num);
}

static int wcn_sipc_buf_list_free(int chn, struct mbuf_t *head,
			      struct mbuf_t *tail, int num)
{
	return buf_list_free(chn, head, tail, num);
}

static void  wcn_sipc_tx_task(unsigned long data)
{
	struct wcn_sipc_info_t *chn_info = wcn_sipc_get_info();
	struct mchn_ops_t *wcn_sipc_ops = NULL;
	struct wcn_sipc_node_t *tx_info, *tx_tmp;
	struct mbuf_t *mbuf;
	int i;

	spin_lock_bh(&chn_info->tx_spinlock);
	if (list_empty(&chn_info->tx_head)) {
		WCN_ERR("tasklet something err\n");
		spin_unlock_bh(&chn_info->tx_spinlock);
		return;
	}
	list_for_each_entry_safe(tx_info, tx_tmp, &chn_info->tx_head, node) {
		list_del(&tx_info->node);
		if (!list_empty(&chn_info->tx_head))
			tasklet_schedule(&chn_info->tx_task);

		mbuf = tx_info->mbuf_head;
		for (i = 0; i < tx_info->num; i++, mbuf = mbuf->next) {
			WCN_DEBUG("tx_info->chn %d  mbuf->len %d\n",
				tx_info->chn, mbuf->len);
			wcn_sipc_write(tx_info->chn,
				      (void *)(mbuf->buf + PUB_HEAD_RSV),
				      mbuf->len);
		}

		wcn_sipc_ops = chn_ops(tx_info->chn);
		if (wcn_sipc_ops)
			wcn_sipc_ops->pop_link(tx_info->chn,
					       tx_info->mbuf_head,
					       tx_info->mbuf_tail,
					       tx_info->num);
		else
			WCN_ERR("no tx ops chn:%d\n",
				tx_info->chn);
		kfree(tx_info);
	}
	spin_unlock_bh(&chn_info->tx_spinlock);
}

static int wcn_sipc_list_push(int chn, struct mbuf_t *head,
			  struct mbuf_t *tail, int num)
{
	struct wcn_sipc_node_t *rx_info, *rx_tmp;
	struct wcn_sipc_node_t *tx_info;
	struct wcn_sipc_info_t *chn_info = wcn_sipc_get_info();
	struct mchn_ops_t *wcn_sipc_ops = NULL;
	struct mbuf_t *mbuf_node = NULL, *mbuf_tmp = NULL;
	int i;

	wcn_sipc_ops = chn_ops(chn);
	WCN_DEBUG("chn %d\n", chn);

	if (wcn_sipc_ops->inout == WCNBUS_TX) {
		tx_info = kmalloc(sizeof(struct wcn_sipc_node_t), GFP_KERNEL);
		if (!tx_info)
			return -ENOMEM;

		spin_lock_bh(&chn_info->tx_spinlock);
		tx_info->chn = chn;
		tx_info->mbuf_head = head;
		tx_info->mbuf_tail = tail;
		tx_info->num = num;
		list_add_tail(&tx_info->node, &chn_info->tx_head);
		tasklet_schedule(&chn_info->tx_task);
		spin_unlock_bh(&chn_info->tx_spinlock);
	} else if (wcn_sipc_ops->inout == WCNBUS_RX) {
		spin_lock_bh(&chn_info->rx_spinlock);
		list_for_each_entry_safe(rx_info, rx_tmp,
					 &chn_info->rx_head, node) {
			if (rx_info->mbuf_head == head) {
				mbuf_node = head;
				for (i = 0; i < num; i++) {
					if (mbuf_node) {
						mbuf_tmp = mbuf_node->next;
						kfree(mbuf_node->buf);
						kfree(mbuf_node);
					}
					mbuf_node = mbuf_tmp;
				}
				list_del(&rx_info->node);
				kfree(rx_info);
				spin_unlock_bh(&chn_info->rx_spinlock);
				return 0;
			}
		}
		kfree(head->buf);
		kfree(head);
		spin_unlock_bh(&chn_info->rx_spinlock);
	}

	return 0;
}

static inline unsigned int wcn_sipc_get_dump_status(void)
{
	return g_dump_status;
}

static inline void wcn_sipc_set_dump_status(unsigned int flag)
{
	g_dump_status = flag;
}

static unsigned long long wcn_sipc_get_rx_total_cnt(void)
{
	return wcn_get_cp2_comm_rx_count();
}

static int wcn_sipc_module_init(void)
{
	unsigned long data = 0;

	sipc_info = kzalloc(sizeof(struct wcn_sipc_info_t), GFP_KERNEL);
	if (!sipc_info)
		return -ENOMEM;

	INIT_LIST_HEAD(&sipc_info->tx_head);
	INIT_LIST_HEAD(&sipc_info->rx_head);
	spin_lock_init(&sipc_info->tx_spinlock);
	spin_lock_init(&sipc_info->rx_spinlock);

	tasklet_init(&sipc_info->tx_task, wcn_sipc_tx_task,
		data);
	WCN_INFO("module init success\n");

	return 0;
}

static struct sprdwcn_bus_ops wcn_sipc_bus_ops = {
	.chn_init = wcn_sipc_chn_init,
	.chn_deinit = wcn_sipc_chn_deinit,
	.list_alloc = wcn_sipc_buf_list_alloc,
	.list_free = wcn_sipc_buf_list_free,

	.push_list = wcn_sipc_list_push,
	.get_carddump_status = wcn_sipc_get_dump_status,
	.set_carddump_status = wcn_sipc_set_dump_status,
	.get_rx_total_cnt = wcn_sipc_get_rx_total_cnt,

};

void module_bus_init(void)
{
	wcn_sipc_module_init();

	module_ops_register(&wcn_sipc_bus_ops);
}
EXPORT_SYMBOL(module_bus_init);

