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

#ifndef _CAM_BUF_H_
#define _CAM_BUF_H_

#include <linux/types.h>
#include <linux/sprd_iommu.h>

enum {
	CAM_BUF_USER_TYPE,/*app alloc/free*/
	CAM_BUF_KERNEL_TYPE,/*kernel alloc/free*/
	CAM_BUF_SWAP_TYPE,/*dcam/isp swap*/
};

enum {
	CAM_BUF_STATE_MAPPING_DCAM = 0x01,
	CAM_BUF_STATE_MAPPING_ISP = 0x02,
	CAM_BUF_STATE_MAPPING = 0x03,
};

struct cam_buf_info {
	struct device *dev;
	uint32_t type;
	uint32_t num; /*valid buf num,max 3*/
	uint32_t state;
	size_t size[3];
	uint32_t offset[3];
	unsigned long iova[3];
	uint32_t mfd[3];
	void *buf[3];
	struct dma_buf *dmabuf_p[3];
	struct ion_client *client[3];/*for ion alloc buffer*/
	struct ion_handle *handle[3];
	uint32_t *kaddr[3];
};

static inline uint32_t cam_buf_is_valid(struct cam_buf_info *buf_info)
{
	if ((buf_info->type == CAM_BUF_USER_TYPE && buf_info->mfd[0] != 0)
	|| (buf_info->client[0] != NULL && buf_info->handle[0] != NULL))
		return 1;

	pr_err("cam buf is not valid!\n");
	pr_err("type mfd client handle %d %x %p %p",
		buf_info->type, buf_info->mfd[0],
		buf_info->client[0], buf_info->handle[0]);
	return 0;
}

static inline uint32_t cam_buf_is_equal(struct cam_buf_info *buf_info1,
	struct cam_buf_info *buf_info2)
{
	if (buf_info1->type == CAM_BUF_USER_TYPE
		&& buf_info1->mfd[0] == buf_info2->mfd[0]
		&& buf_info1->offset[0] == buf_info2->offset[0])
		return 1;
	else if (buf_info1->type != CAM_BUF_USER_TYPE
		&& buf_info1->client[0] == buf_info2->client[0]
		&& buf_info1->handle[0] == buf_info2->handle[0])
		return 1;

	return 0;
}

int cam_buf_get_sg_table(struct cam_buf_info *info);
int cam_buf_put_sg_table(void);
int cam_buf_check_addr(struct cam_buf_info *info);
int cam_buf_map_addr(struct cam_buf_info *info);
int cam_buf_unmap_addr(struct cam_buf_info *info);
int cam_buf_unmap_addr_with_id(struct cam_buf_info *info,
	enum sprd_iommu_chtype ctype, uint32_t cid);

int cam_buf_alloc_k(struct cam_buf_info *buf_info, struct device *dev,
	size_t size, uint32_t num, uint32_t type);
int cam_buf_free_k(struct cam_buf_info *info);

#endif /* _CAM_BUF_H_ */
