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

#ifndef _CAM_COMMON_H_
#define _CAM_COMMON_H_

#include <linux/types.h>
#include <video/sprd_img.h>

#include "cam_buf.h"
#include "isp_3dnr_drv.h"

/* #define CAM_DEBUG */

#ifdef CAM_DEBUG
#define CAM_TRACE                      pr_info
#define DCAM_TRACE                     pr_info
#else
#define CAM_TRACE                      pr_debug
#define DCAM_TRACE                     pr_debug
#endif

#define CAM_LOWEST_ADDR                0x800
#define CAM_ADDR_INVALID(addr) \
	((unsigned long)(addr) < CAM_LOWEST_ADDR)
#define CAM_YUV_ADDR_INVALID(y, u, v) \
	(CAM_ADDR_INVALID(y) && \
	 CAM_ADDR_INVALID(u) && \
	 CAM_ADDR_INVALID(v))

#define ISP_PATH1_LINE_BUF_LENGTH      2592
#define ISP_PATH2_LINE_BUF_LENGTH      2304
#define ISP_PATH3_LINE_BUF_LENGTH      4656

#define DCAM_BUF_QUEUE_LENGTH          24
#define DCAM_FRM_QUEUE_LENGTH          5
#define CAMERA_QUEUE_LENGTH            96

#define ISP_STATISTICS_QUEUE_LEN       8

#define ZOOM_RATIO_DEFAULT             1000

enum camera_path_id {
	CAMERA_FULL_PATH = 0,
	CAMERA_BIN_PATH,
	CAMERA_PRE_PATH,
	CAMERA_CAP_PATH,
	CAMERA_VID_PATH,
	CAMERA_PDAF_PATH,
	CAMERA_VCH2_PATH,
	CAMERA_MAX_PATH,
};

enum cam_data_endian {
	CAM_ENDIAN_LITTLE = 0,
	CAM_ENDIAN_BIG,
	CAM_ENDIAN_HALFBIG,
	CAM_ENDIAN_HALFLITTLE,
	CAM_ENDIAN_MAX
};

struct camera_size {
	uint32_t w;
	uint32_t h;
};

struct camera_rect {
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
};

struct camera_addr {
	uint32_t yaddr;
	uint32_t uaddr;
	uint32_t vaddr;
	uint32_t yaddr_vir;
	uint32_t uaddr_vir;
	uint32_t vaddr_vir;
	uint32_t mfd_y;
	uint32_t mfd_u;
	uint32_t mfd_v;
	uint32_t reserved;
	struct device *p_dev;/*for iommu map*/
	uint32_t type;
	struct ion_client *client[3];/*for ion alloc buffer*/
	struct ion_handle *handle[3];
	unsigned long iova[3];
	uint32_t state;
	uint32_t num; /*valid buf num, max 3*/
};

struct camera_endian_sel {
	uint32_t y_endian;
	uint32_t uv_endian;
	uint32_t reserved0;
	uint32_t reserved1;
};

struct camera_path_dec {
	uint32_t x_factor;
	uint32_t y_factor;
	uint32_t reserved[2];
};

struct camera_frame {
	uint32_t type;
	uint32_t lock;/*reserved*/
	uint32_t flags;
	uint32_t fid;
	uint32_t width;
	uint32_t height;
	uint32_t yaddr;
	uint32_t uaddr;
	uint32_t vaddr;
	uint32_t yaddr_vir;
	uint32_t uaddr_vir;
	uint32_t vaddr_vir;
	atomic_t usr_cnt;
	unsigned long phy_addr;/*3A static buf*/
	unsigned long vir_addr;/*3A static buf*/
	uint32_t kaddr[2];/*3A static buf*/
	uint32_t addr_offset;/*3A static buf*/
	uint32_t buf_size;
	uint32_t irq_type;
	uint32_t irq_property;/*isp moddule*/
	uint32_t frame_id;/*isp moddule*/
	uint32_t zoom_ratio;
	struct cam_buf_info buf_info;
	struct frame_mv mv;
	unsigned long timestamp;
};

struct camera_get_path_id {
	uint32_t fourcc;
	uint32_t sn_fmt;
	uint32_t is_path_work[CAMERA_MAX_PATH];
	uint32_t need_isp_tool;
	uint32_t need_isp;
	struct camera_size input_size;
	struct camera_rect input_trim;
	struct camera_size output_size;
};

struct cam_path_info {
	uint32_t line_buf;
	uint32_t support_yuv;
	uint32_t support_raw;
	uint32_t support_jpeg;
	uint32_t support_scaling;
	uint32_t support_trim;
	uint32_t is_scaleing_path;
	uint32_t is_slow_motion;
};

struct cam_path_capability {
	uint32_t count;
	uint32_t support_3dnr_mode;
	struct cam_path_info path_info[CAMERA_MAX_PATH];
};

extern struct platform_device *s_dcam_pdev;
extern struct platform_device *s_isp_pdev;

typedef int (*cam_isr_func) (struct camera_frame *frame, void *u_data);

int cam_get_timestamp(struct timeval *tv);

int dcam_set_frame_addr(void **isp_dev_handle,
	enum camera_path_id path_idx, struct camera_frame *frame_addr);

int dcam_calc_raw10_line_bytes(uint16_t isloose, uint16_t width);

#endif /* _CAM_COMMON_H_ */
