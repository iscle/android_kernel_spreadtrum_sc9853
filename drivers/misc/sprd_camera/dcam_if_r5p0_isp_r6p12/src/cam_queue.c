/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>

#include "cam_common.h"
#include "cam_queue.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "cam_queue: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int sprd_cam_buf_queue_init(struct cam_buf_queue *queue, int queue_type,
		int node_size, int node_num, char *q_str)
{
	int ret = 0;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	memset(queue, 0x00, sizeof(*queue));
	queue->node = vzalloc(node_size * node_num + strlen(q_str) + 1);
	if (queue->node) {
		queue->write = &queue->node[0];
		queue->read = &queue->node[0];
		queue->node_size = node_size;
		queue->node_num = node_num;
		queue->q_str = queue->node + (node_size * node_num);
		strcpy(queue->q_str, q_str);
		queue->type = queue_type;
		if (queue_type != CAM_QUEUE_NO_LOCK)
			spin_lock_init(&queue->lock);
		pr_info("buf queue (%s) init ok\n", queue->q_str);
		ret = 0;
	} else {
		pr_err("buf queue (%s) init fail\n", queue->q_str);
		ret = -ENOMEM;
	}

	return ret;
}

int sprd_cam_buf_queue_write(struct cam_buf_queue *queue, void *node)
{
	char *ori_node;
	char *last_node;
	unsigned long flags = 0;

	if (queue == NULL || queue->node == NULL
		|| node == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type & CAM_QUEUE_W_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	ori_node = queue->write;
	last_node = queue->node + queue->node_size * (queue->node_num - 1);
	memcpy(queue->write, node, queue->node_size);
	queue->write += queue->node_size;
	queue->wcnt++;
	if (queue->write > last_node)
		queue->write = &queue->node[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		queue->wcnt--;
		pr_info("warning, buf queue (%s) is full\n", queue->q_str);
	}
	if (queue->type & CAM_QUEUE_W_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);
	pr_debug("buf queue (%s) w ok\n", queue->q_str);

	return 0;
}

int sprd_cam_buf_queue_read(struct cam_buf_queue *queue, void *node)
{
	int ret = 0;
	int flag = 0;
	unsigned long flags = 0;
	char *last_node;

	if (queue == NULL || queue->node == NULL
		|| node == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type & CAM_QUEUE_R_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		flag = 1;
		last_node = queue->node +
			queue->node_size * (queue->node_num - 1);

		memcpy(node, queue->read, queue->node_size);
		queue->read += queue->node_size;
		queue->rcnt++;
		if (queue->read > last_node)
			queue->read = &queue->node[0];
		pr_debug("buf queue (%s) r ok\n", queue->q_str);
	}
	if (!flag) {
		ret = -EAGAIN;
		pr_debug("buf queue (%s) is empty!\n", queue->q_str);
	}
	if (queue->type & CAM_QUEUE_R_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}


int sprd_cam_buf_queue_clear(struct cam_buf_queue *queue)
{
	int ret = 0;
	unsigned long flags = 0;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->node) {
		if (queue->type != CAM_QUEUE_NO_LOCK)
			spin_lock_irqsave(&queue->lock, flags);
		queue->write = &queue->node[0];
		queue->read = &queue->node[0];
		memset(queue->node, 0, queue->node_size * queue->node_num);
		if (queue->type != CAM_QUEUE_NO_LOCK)
			spin_unlock_irqrestore(&queue->lock, flags);
		ret = 0;
	} else {
		pr_err("buf queue is not validl!\n");
		ret = -ENOMEM;
	}

	return ret;
}

int sprd_cam_buf_queue_get_wnode(struct cam_buf_queue *queue,
				 void **node)
{
	unsigned long flags = 0;

	if (queue == NULL || queue->node == NULL
		|| node == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type & CAM_QUEUE_W_LOCK)
		spin_lock_irqsave(&queue->lock, flags);

	*node = queue->write;

	if (queue->type & CAM_QUEUE_W_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return 0;
}

int sprd_cam_buf_queue_wnode_inc(struct cam_buf_queue *queue)
{
	char *ori_node;
	char *last_node;
	unsigned long flags = 0;

	if (queue == NULL || queue->node == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type & CAM_QUEUE_W_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	ori_node = queue->write;
	last_node = queue->node + queue->node_size * (queue->node_num - 1);
	queue->write += queue->node_size;
	queue->wcnt++;
	if (queue->write > last_node)
		queue->write = &queue->node[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		queue->wcnt--;
		pr_info("warning, buf queue (%s) is full\n", queue->q_str);
	}
	if (queue->type & CAM_QUEUE_W_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);
	pr_debug("buf queue (%s) w ok\n", queue->q_str);
	return 0;
}

int sprd_cam_buf_queue_get_rnode(struct cam_buf_queue *queue,
				 void **node)
{
	int ret = 0;
	unsigned long flags = 0;

	if (queue == NULL || queue->node == NULL
		|| node == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type & CAM_QUEUE_R_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		*node = queue->read;
		pr_debug("buf queue (%s) r ok\n", queue->q_str);
	} else {
		ret = -EAGAIN;
		pr_debug("buf queue (%s) is empty!\n", queue->q_str);
	}
	if (queue->type & CAM_QUEUE_R_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

int sprd_cam_buf_queue_rnode_inc(struct cam_buf_queue *queue)
{
	int ret = 0;
	int flag = 0;
	unsigned long flags = 0;
	char *last_node;

	if (queue == NULL || queue->node == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type & CAM_QUEUE_R_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		flag = 1;
		last_node = queue->node +
			queue->node_size * (queue->node_num - 1);

		queue->read += queue->node_size;
		queue->rcnt++;
		if (queue->read > last_node)
			queue->read = &queue->node[0];
		pr_debug("buf queue (%s) r ok\n", queue->q_str);
	}
	if (!flag) {
		ret = -EAGAIN;
		pr_info("buf queue (%s) is empty!\n", queue->q_str);
	}
	if (queue->type & CAM_QUEUE_R_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}


int sprd_cam_buf_queue_deinit(struct cam_buf_queue *queue)
{
	unsigned long flags = 0;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	if (queue->node) {
		vfree(queue->node);
		queue->node = NULL;
		queue->write = NULL;
		queue->read = NULL;
	}
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);
	return 0;
}

uint32_t sprd_cam_buf_queue_cur_nodes(struct cam_buf_queue *queue)
{
	unsigned long flags = 0;
	uint32_t node_num = 0;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	if (queue->write > queue->read)
		node_num = (queue->write - queue->read) / queue->node_size - 1;
	else if (queue->write < queue->read)
		node_num = queue->node_num -
		(queue->read - queue->write) / queue->node_size
		- 1;
	else
		node_num = 0;
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return node_num;
}

int sprd_cam_frm_queue_init(struct cam_frm_queue *queue, int queue_type,
		int node_size, int node_num, char *q_str)
{
	int ret = 0;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	memset(queue, 0x00, sizeof(*queue));
	queue->frm_array = vzalloc(node_size * node_num + strlen(q_str) + 1);
	if (queue->frm_array) {
		queue->node_size = node_size;
		queue->node_num = node_num;
		queue->q_str = queue->frm_array + (node_size * node_num);
		strcpy(queue->q_str, q_str);
		queue->type = queue_type;
		if (queue_type != CAM_QUEUE_NO_LOCK)
			spin_lock_init(&queue->lock);
		pr_info("frm queue (%s) init ok\n", queue->q_str);
		ret = 0;
	} else {
		pr_err("frm queue (%s) init fail\n", queue->q_str);
		ret = -ENOMEM;
	}

	return ret;
}

int sprd_cam_frm_queue_deinit(struct cam_frm_queue *queue)
{
	unsigned long flags = 0;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);
	if (queue->frm_array) {
		vfree(queue->frm_array);
		queue->frm_array = NULL;
	}
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);
	return 0;

}

int sprd_cam_frm_enqueue(struct cam_frm_queue *queue,
			void *node)
{
	unsigned long flags = 0;

	if (queue == NULL || queue->frm_array == NULL
		|| node == NULL) {
		pr_err("fail to get valid parm %p, %p\n", queue, node);
		return -1;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);

	if (queue->valid_cnt >= queue->node_num) {
		pr_info("frm queue (%s) over flow\n", queue->q_str);
		if (queue->type != CAM_QUEUE_NO_LOCK)
			spin_unlock_irqrestore(&queue->lock, flags);
		return -1;
	}

	memcpy(queue->frm_array + queue->node_size * queue->valid_cnt, node,
			queue->node_size);
	queue->valid_cnt++;
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return 0;
}

int sprd_cam_frm_dequeue(struct cam_frm_queue *queue,
			void *node)
{
	uint32_t i = 0;
	unsigned long flags = 0;

	if (queue == NULL || queue->frm_array == NULL
		|| node == NULL) {
		pr_err("deq, invalid parm %p, %p\n", queue, node);
		return -1;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);

	if (queue->valid_cnt == 0) {
		pr_err("frm queue (%s) under flow\n", queue->q_str);
		if (queue->type != CAM_QUEUE_NO_LOCK)
			spin_unlock_irqrestore(&queue->lock, flags);
		return -1;
	}

	memcpy(node, queue->frm_array, queue->node_size);
	queue->valid_cnt--;
	for (i = 0; i < queue->valid_cnt; i++) {
		memcpy(queue->frm_array + queue->node_size * i,
				queue->frm_array + queue->node_size * (i + 1),
				queue->node_size);
	}
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return 0;
}

int sprd_cam_frm_queue_cur_nodes(struct cam_frm_queue *queue)
{
	unsigned long flags = 0;
	uint32_t valid_cnt;

	if (queue == NULL || queue->frm_array == NULL) {
		pr_err("deq, invalid parm %p\n", queue);
		return -1;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);

	valid_cnt = queue->valid_cnt;
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return valid_cnt;
}

int sprd_cam_frm_get_firstnode(struct cam_frm_queue *queue,
			void **node)
{
	unsigned long flags = 0;

	if (queue == NULL || queue->frm_array == NULL
		|| node == NULL) {
		pr_err("deq, invalid parm %p, %p\n", queue, node);
		return -1;
	}

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);

	if (queue->valid_cnt == 0) {
		pr_err("frm queue (%s) under flow\n", queue->q_str);
		if (queue->type != CAM_QUEUE_NO_LOCK)
			spin_unlock_irqrestore(&queue->lock, flags);
		return -1;
	}

	*node = queue->frm_array;
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);

	return 0;
}

void sprd_cam_frm_queue_clear(struct cam_frm_queue *queue)
{
	unsigned long flags = 0;

	if (queue == NULL) {
		pr_err("fail to get valid heap %p\n", queue);
		return;
	}
	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_lock_irqsave(&queue->lock, flags);

	memset((void *)queue->frm_array, 0,
		queue->node_size * queue->node_num);
	queue->valid_cnt = 0;

	if (queue->type != CAM_QUEUE_NO_LOCK)
		spin_unlock_irqrestore(&queue->lock, flags);
}
