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

#ifndef _ISP_BUF_HEADER_
#define _ISP_BUF_HEADER_

#include "isp_drv.h"

extern spinlock_t isp_mod_lock;

void isp_frm_clear(struct isp_pipe_dev *dev, enum isp_path_index path_index);
int isp_cfg_ctx_buf_iommu_map(struct isp_cfg_ctx_desc *cfg_ctx);
#endif
