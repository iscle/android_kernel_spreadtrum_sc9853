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

#ifndef _CAM_QUEUE_H_
#define _CAM_QUEUE_H_

#include <linux/spinlock.h>

enum {
	CAM_QUEUE_NO_LOCK = 0,
	CAM_QUEUE_R_LOCK,
	CAM_QUEUE_W_LOCK,
	CAM_QUEUE_RW_LOCK,
};

/*genenal queue for camera system, when queue full, nodes will not be assert*/
struct cam_buf_queue {
	uint32_t type;
	char	*node;
	char	*write;
	char	*read;
	uint32_t wcnt;
	uint32_t rcnt;
	uint32_t node_num;
	uint32_t node_size;
	spinlock_t lock;
	char *q_str;/*debug string, can be NULL*/
};

int sprd_cam_buf_queue_init(struct cam_buf_queue *queue, int queue_type,
		int node_size, int node_num, char *q_str);
int sprd_cam_buf_queue_write(struct cam_buf_queue *queue, void *node);
int sprd_cam_buf_queue_read(struct cam_buf_queue *queue, void *node);
int sprd_cam_buf_queue_clear(struct cam_buf_queue *queue);
int sprd_cam_buf_queue_deinit(struct cam_buf_queue *queue);
uint32_t sprd_cam_buf_queue_cur_nodes(struct cam_buf_queue *queue);
int sprd_cam_buf_queue_get_wnode(struct cam_buf_queue *queue,
				 void **node);
int sprd_cam_buf_queue_wnode_inc(struct cam_buf_queue *queue);
int sprd_cam_buf_queue_get_rnode(struct cam_buf_queue *queue,
				 void **node);
int sprd_cam_buf_queue_rnode_inc(struct cam_buf_queue *queue);

/*genenal queue for camera system, when queue full, nodes will not be assert*/
struct cam_frm_queue {
	uint32_t valid_cnt;
	char *frm_array;
	uint32_t node_size;
	uint32_t node_num;
	int type;
	spinlock_t lock;
	char *q_str;/*debug string, can be NULL*/
};

int sprd_cam_frm_queue_init(struct cam_frm_queue *queue, int queue_type,
		int node_size, int node_num, char *q_str);
int sprd_cam_frm_enqueue(struct cam_frm_queue *queue,
			void *node);
int sprd_cam_frm_dequeue(struct cam_frm_queue *queue,
			void *node);
int sprd_cam_frm_queue_cur_nodes(struct cam_frm_queue *queue);
int sprd_cam_frm_get_firstnode(struct cam_frm_queue *queue,
			void **node);
void sprd_cam_frm_queue_clear(struct cam_frm_queue *queue);
int sprd_cam_frm_queue_deinit(struct cam_frm_queue *queue);

#endif /* _CAM_QUEUE_H_ */
