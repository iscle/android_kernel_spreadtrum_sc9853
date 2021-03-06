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

#ifndef _CAM_STATISTIC_H_
#define _CAM_STATISTIC_H_

#include <video/sprd_isp_r6p12.h>

#include "cam_buf.h"
#include "cam_queue.h"

#define ISP_PDAF_INIT_FLAG             0
#define ISP_PDAF_START_FLAG            (1 << ISP_PDAF_START_ID)
#define ISP_PDAF_LEFT_DONE_FLAG        (1 << ISP_PDAF_LEFT_DONE_ID)
#define ISP_PDAF_RIGHT_DONE_FLAG       (1 << ISP_PDAF_RIGHT_DONE_ID)

enum isp_pdaf_flag_id {
	ISP_PDAF_START_ID = 0,
	ISP_PDAF_LEFT_DONE_ID = 1,
	ISP_PDAF_RIGHT_DONE_ID = 2,
	ISP_PDAF_MAX_ID
};

/*all statitics queue node definitions*/
struct cam_statis_buf {
	uint32_t buf_size;
	int buf_property;
	unsigned long phy_addr;
	unsigned long vir_addr;
	unsigned long addr_offset;
	unsigned long kaddr[2];
	unsigned long mfd;
	uint32_t frame_id;
	struct cam_buf_info buf_info;
};

/*all statitics queues attached with dcam/isp*/
struct cam_statis_module {
	struct cam_frm_queue aem_statis_frm_queue;
	struct cam_frm_queue afl_statis_frm_queue;
	struct cam_frm_queue afm_statis_frm_queue;
	struct cam_frm_queue nr3_statis_frm_queue;
	struct cam_frm_queue pdaf_statis_frm_queue;
	struct cam_statis_buf aem_buf_reserved;
	struct cam_statis_buf afl_buf_reserved;
	struct cam_statis_buf afm_buf_reserved;
	struct cam_statis_buf nr3_buf_reserved;
	struct cam_statis_buf pdaf_buf_reserved;
	struct cam_buf_queue aem_statis_queue;
	struct cam_buf_queue afl_statis_queue;
	struct cam_buf_queue afm_statis_queue;
	struct cam_buf_queue nr3_statis_queue;
	struct cam_buf_queue pdaf_statis_queue;
	struct cam_statis_buf img_statis_buf;
	uint32_t pdaf_status;
	uint32_t pdaf_type;
	uint32_t aem_status;
};

int sprd_cam_init_statis_queue(struct cam_statis_module *statis_module);
void sprd_cam_clear_statis_queue(struct cam_statis_module *statis_module);
void sprd_cam_deinit_statis_queue(struct cam_statis_module *statis_module);
int sprd_cam_cfg_statis_buf(struct device *dev,
	struct cam_statis_module *statis_module,
	struct isp_statis_buf_input *parm);
int sprd_cam_set_statis_addr(struct device *dev,
	struct cam_statis_module *statis_module,
	struct isp_statis_buf_input *parm);
int sprd_cam_set_statis_buf(int idx, struct cam_statis_module *statis_module);
int sprd_cam_set_next_statis_buf(int idx,
	struct cam_statis_module *statis_module,
	enum isp_3a_block_id block_index);
int sprd_aem_fake(int idx);

#endif
