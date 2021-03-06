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

#include <linux/types.h>
#include <linux/sprd_ion.h>
#include <linux/sprd_iommu.h>

#include "cam_statistic.h"
#include "isp_drv.h"

#define ION
#ifdef ION
#include "ion.h"
#include "ion_priv.h"
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_STATICS: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int sprd_cam_init_statis_queue(struct cam_statis_module *module)
{

	int ret = ISP_RTN_SUCCESS;

	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	ret = sprd_cam_buf_queue_init(&module->aem_statis_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "ae buf_queue");
	ret = sprd_cam_buf_queue_init(&module->afl_statis_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "afl buf_queue");
	ret = sprd_cam_buf_queue_init(&module->afm_statis_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "afm buf_queue");
	ret = sprd_cam_buf_queue_init(
		&module->nr3_statis_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "nr3 buf_queue");
	ret = sprd_cam_buf_queue_init(
		&module->pdaf_statis_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "pdaf buf_queue");

	ret = sprd_cam_frm_queue_init(&module->aem_statis_frm_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "ae frm_queue");
	ret = sprd_cam_frm_queue_init(&module->afl_statis_frm_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "afl frm_queue");
	ret = sprd_cam_frm_queue_init(&module->afm_statis_frm_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "afm frm_queue");
	ret = sprd_cam_frm_queue_init(
		&module->nr3_statis_frm_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct cam_statis_buf),
		ISP_STATISTICS_QUEUE_LEN, "nr3 frm_queue");
	ret = sprd_cam_frm_queue_init(
		&module->pdaf_statis_frm_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct camera_frame),
		ISP_STATISTICS_QUEUE_LEN, "pdaf frm_queue");

	CAM_TRACE("statis buf init ok.\n");
	return ret;
}

void sprd_cam_clear_statis_queue(struct cam_statis_module *module)
{
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	sprd_cam_buf_queue_clear(&module->aem_statis_queue);
	sprd_cam_buf_queue_clear(&module->afl_statis_queue);
	sprd_cam_buf_queue_clear(&module->afm_statis_queue);
	sprd_cam_buf_queue_clear(&module->nr3_statis_queue);
	sprd_cam_buf_queue_clear(&module->pdaf_statis_queue);

	sprd_cam_frm_queue_clear(&module->aem_statis_frm_queue);
	sprd_cam_frm_queue_clear(&module->afl_statis_frm_queue);
	sprd_cam_frm_queue_clear(&module->afm_statis_frm_queue);
	sprd_cam_frm_queue_clear(&module->nr3_statis_frm_queue);
	sprd_cam_frm_queue_clear(&module->pdaf_statis_frm_queue);

	pr_info("statis buf clear ok.\n");
}


void sprd_cam_deinit_statis_queue(struct cam_statis_module *module)
{
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	sprd_cam_buf_queue_deinit(&module->aem_statis_queue);
	sprd_cam_buf_queue_deinit(&module->afl_statis_queue);
	sprd_cam_buf_queue_deinit(&module->afm_statis_queue);
	sprd_cam_buf_queue_deinit(&module->nr3_statis_queue);
	sprd_cam_buf_queue_deinit(&module->pdaf_statis_queue);

	sprd_cam_frm_queue_deinit(&module->aem_statis_frm_queue);
	sprd_cam_frm_queue_deinit(&module->afl_statis_frm_queue);
	sprd_cam_frm_queue_deinit(&module->afm_statis_frm_queue);
	sprd_cam_frm_queue_deinit(&module->nr3_statis_frm_queue);
	sprd_cam_frm_queue_deinit(&module->pdaf_statis_frm_queue);

	cam_buf_unmap_addr(&module->img_statis_buf.buf_info);
	memset(&module->img_statis_buf.buf_info, 0x00,
		sizeof(module->img_statis_buf.buf_info));
	CAM_TRACE("statis buf deinit ok.\n");
}

int sprd_cam_cfg_statis_buf(struct device *dev,
	struct cam_statis_module *module,
	struct isp_statis_buf_input *parm)
{
	int ret = ISP_RTN_SUCCESS;
	int cnt = 0;
	uint32_t aem_iova_addr = 0, aem_vir_addr = 0;
	unsigned long aem_kaddr = 0;
	uint32_t afl_iova_addr = 0, afl_vir_addr = 0;
	unsigned long afl_kaddr = 0;
	uint32_t afm_iova_addr = 0, afm_vir_addr = 0;
	unsigned long afm_kaddr = 0;
	uint32_t nr3_iova_addr = 0, nr3_vir_addr = 0;
	unsigned long nr3_kaddr = 0;
	uint32_t pdaf_iova_addr = 0, pdaf_vir_addr = 0;
	unsigned long pdaf_kaddr = 0;
	uint32_t addr_offset = 0;
	struct cam_statis_buf frm_statis;
	struct cam_statis_buf aem_frm_statis;
	struct cam_statis_buf afl_frm_statis;
	struct cam_statis_buf afm_frm_statis;
	struct cam_statis_buf nr3_frm_statis;
	struct cam_statis_buf pdaf_frm_statis;
	size_t statis_mem_size = 0;

	if (!dev || !module || !parm) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	CAM_TRACE("cfg statis buf in.\n");
	memset((void *)&module->img_statis_buf, 0,
		sizeof(module->img_statis_buf));
	memset((void *)&frm_statis, 0x00, sizeof(frm_statis));
	memset((void *)&aem_frm_statis, 0x00, sizeof(aem_frm_statis));
	memset((void *)&afl_frm_statis, 0x00, sizeof(afl_frm_statis));
	memset((void *)&afm_frm_statis, 0x00, sizeof(afm_frm_statis));
	memset((void *)&pdaf_frm_statis, 0x00, sizeof(pdaf_frm_statis));
	memset((void *)&nr3_frm_statis, 0x00, sizeof(nr3_frm_statis));
	frm_statis.phy_addr = parm->phy_addr;
	frm_statis.vir_addr = parm->vir_addr;
	frm_statis.buf_size = parm->buf_size;
	frm_statis.buf_property = parm->buf_property;

	frm_statis.buf_info.dev = dev;
	frm_statis.buf_info.mfd[0] = parm->mfd;
	/*mapping iommu buffer*/
	ret = cam_buf_get_sg_table(&frm_statis.buf_info);
	if (ret) {
		pr_err("fail to cfg statis buf addr\n");
		ret = -1;
		return ret;
	}

	ret = cam_buf_map_addr(&frm_statis.buf_info);
	memcpy(&module->img_statis_buf, &frm_statis,
		sizeof(struct cam_statis_buf));
	aem_iova_addr = frm_statis.buf_info.iova[0];
	aem_vir_addr = parm->vir_addr;
#ifdef CONFIG_64BIT
	aem_kaddr = (unsigned long)parm->kaddr[0]
		| ((unsigned long)parm->kaddr[1] << 32);
#else
	aem_kaddr = (unsigned long)parm->kaddr[0];
#endif
	CAM_TRACE("kaddr[0]=%d, kaddr[1]= %d, aem:%lu.\n",
		parm->kaddr[0],
		parm->kaddr[1],
		aem_kaddr);
	statis_mem_size = frm_statis.buf_info.size[0];
	/*split the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_AEM_STATIS_BUF_NUM; cnt++) {
		aem_frm_statis.phy_addr = aem_iova_addr;
		aem_frm_statis.vir_addr = aem_vir_addr;
		aem_frm_statis.kaddr[0] = aem_kaddr;
	#ifdef CONFIG_64BIT
		aem_frm_statis.kaddr[1] = aem_kaddr >> 32;
	#endif
		aem_frm_statis.addr_offset = addr_offset;
		aem_frm_statis.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
		aem_frm_statis.buf_size = ISP_AEM_STATIS_BUF_SIZE;
		aem_frm_statis.buf_property = ISP_AEM_BLOCK;
		aem_frm_statis.buf_info.dev = dev;

		ret = sprd_cam_buf_queue_write(&module->aem_statis_queue,
					&aem_frm_statis);
		aem_iova_addr += ISP_AEM_STATIS_BUF_SIZE;
		aem_vir_addr += ISP_AEM_STATIS_BUF_SIZE;
		aem_kaddr += ISP_AEM_STATIS_BUF_SIZE;
		addr_offset += ISP_AEM_STATIS_BUF_SIZE;
	}
	/*init reserved aem statis buf*/
	module->aem_buf_reserved.phy_addr = aem_iova_addr;
	module->aem_buf_reserved.vir_addr = aem_vir_addr;
	module->aem_buf_reserved.kaddr[0] = aem_kaddr;
#ifdef CONFIG_64BIT
	module->aem_buf_reserved.kaddr[1] = aem_kaddr >> 32;
#endif
	module->aem_buf_reserved.addr_offset = addr_offset;
	module->aem_buf_reserved.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
	module->aem_buf_reserved.buf_size = ISP_AEM_STATIS_BUF_SIZE;
	module->aem_buf_reserved.buf_property = ISP_AEM_BLOCK;
	module->aem_buf_reserved.buf_info.dev = dev;

	/*afl statis buf cfg*/
	afl_iova_addr = aem_iova_addr + ISP_AEM_STATIS_BUF_SIZE;
	afl_vir_addr = aem_vir_addr + ISP_AEM_STATIS_BUF_SIZE;
	afl_kaddr = aem_kaddr + ISP_AEM_STATIS_BUF_SIZE;
	addr_offset += ISP_AEM_STATIS_BUF_SIZE;
	/*split the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_AFL_STATIS_BUF_NUM; cnt++) {
		afl_frm_statis.phy_addr = afl_iova_addr;
		afl_frm_statis.vir_addr = afl_vir_addr;
		afl_frm_statis.kaddr[0] = afl_kaddr;
	#ifdef CONFIG_64BIT
		afl_frm_statis.kaddr[1] = afl_kaddr >> 32;
	#endif
		afl_frm_statis.addr_offset = addr_offset;
		afl_frm_statis.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
		afl_frm_statis.buf_size = ISP_AFL_STATIS_BUF_SIZE;
		afl_frm_statis.buf_property = ISP_AFL_BLOCK;
		afl_frm_statis.buf_info.dev = dev;
		ret = sprd_cam_buf_queue_write(&module->afl_statis_queue,
					&afl_frm_statis);
		afl_iova_addr += ISP_AFL_STATIS_BUF_SIZE;
		afl_vir_addr += ISP_AFL_STATIS_BUF_SIZE;
		afl_kaddr += ISP_AFL_STATIS_BUF_SIZE;
		addr_offset += ISP_AFL_STATIS_BUF_SIZE;
	}
	/*init reserved afl statis buf*/
	module->afl_buf_reserved.phy_addr = afl_iova_addr;
	module->afl_buf_reserved.vir_addr = afl_vir_addr;
	module->afl_buf_reserved.kaddr[0] = afl_kaddr;
#ifdef CONFIG_64BIT
	module->afl_buf_reserved.kaddr[1] = afl_kaddr >> 32;
#endif
	module->afl_buf_reserved.addr_offset = addr_offset;
	module->afl_buf_reserved.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
	module->afl_buf_reserved.buf_size = ISP_AFL_STATIS_BUF_SIZE;
	module->afl_buf_reserved.buf_property = ISP_AFL_BLOCK;
	module->afl_buf_reserved.buf_info.dev = dev;

	/*afm statis buf cfg*/
	afm_iova_addr = afl_iova_addr + ISP_AFL_STATIS_BUF_SIZE;
	afm_vir_addr = afl_vir_addr + ISP_AFL_STATIS_BUF_SIZE;
	afm_kaddr = afl_kaddr + ISP_AFL_STATIS_BUF_SIZE;
	addr_offset += ISP_AFL_STATIS_BUF_SIZE;
	/*slip the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_AFM_STATIS_BUF_NUM; cnt++) {
		afm_frm_statis.phy_addr = afm_iova_addr;
		afm_frm_statis.vir_addr = afm_vir_addr;
		afm_frm_statis.kaddr[0] = afm_kaddr;
	#ifdef CONFIG_64BIT
		afm_frm_statis.kaddr[1] = afm_kaddr >> 32;
	#endif
		afm_frm_statis.addr_offset = addr_offset;
		afm_frm_statis.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
		afm_frm_statis.buf_size = ISP_AFM_STATIS_BUF_SIZE;
		afm_frm_statis.buf_property = ISP_AFM_BLOCK;
		afm_frm_statis.buf_info.dev = dev;
		ret = sprd_cam_buf_queue_write(&module->afm_statis_queue,
					&afm_frm_statis);
		afm_iova_addr += ISP_AFM_STATIS_BUF_SIZE;
		afm_vir_addr += ISP_AFM_STATIS_BUF_SIZE;
		afm_kaddr += ISP_AFM_STATIS_BUF_SIZE;
		addr_offset += ISP_AFM_STATIS_BUF_SIZE;
	}
	/*init reserved afl statis buf*/
	module->afm_buf_reserved.phy_addr = afm_iova_addr;
	module->afm_buf_reserved.vir_addr = afm_vir_addr;
	module->afm_buf_reserved.kaddr[0] = afm_kaddr;
#ifdef CONFIG_64BIT
	module->afm_buf_reserved.kaddr[1] = afm_kaddr >> 32;
#endif
	module->afm_buf_reserved.addr_offset = addr_offset;
	module->afm_buf_reserved.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
	module->afm_buf_reserved.buf_size = ISP_AFM_STATIS_BUF_SIZE;
	module->afm_buf_reserved.buf_property = ISP_AFM_BLOCK;
	module->afm_buf_reserved.buf_info.dev = dev;

	/*pdaf statis buf cfg*/
	pdaf_iova_addr = afm_iova_addr + ISP_AFM_STATIS_BUF_SIZE;
	pdaf_vir_addr = afm_vir_addr + ISP_AFM_STATIS_BUF_SIZE;
	pdaf_kaddr = afm_kaddr + ISP_AFM_STATIS_BUF_SIZE;
	addr_offset  += ISP_AFM_STATIS_BUF_SIZE;
	/*slip the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_PDAF_STATIS_BUF_NUM; cnt++) {
		pdaf_frm_statis.phy_addr = pdaf_iova_addr;
		pdaf_frm_statis.vir_addr = pdaf_vir_addr;
		pdaf_frm_statis.kaddr[0] = pdaf_kaddr;
	#ifdef CONFIG_64BIT
		pdaf_frm_statis.kaddr[1] = pdaf_kaddr >> 32;
	#endif
		pdaf_frm_statis.addr_offset = addr_offset;
		pdaf_frm_statis.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
		pdaf_frm_statis.buf_size = ISP_PDAF_STATIS_BUF_SIZE;
		pdaf_frm_statis.buf_property = ISP_PDAF_BLOCK;
		pdaf_frm_statis.buf_info.dev = dev;
		ret = sprd_cam_buf_queue_write(&module->pdaf_statis_queue,
					&pdaf_frm_statis);
		pdaf_iova_addr += ISP_PDAF_STATIS_BUF_SIZE;
		pdaf_vir_addr += ISP_PDAF_STATIS_BUF_SIZE;
		pdaf_kaddr += ISP_PDAF_STATIS_BUF_SIZE;
		addr_offset += ISP_PDAF_STATIS_BUF_SIZE;
	}
	/*init reserved pdaf statis buf*/
	module->pdaf_buf_reserved.phy_addr = pdaf_iova_addr;
	module->pdaf_buf_reserved.vir_addr = pdaf_vir_addr;
	module->pdaf_buf_reserved.kaddr[0] = pdaf_kaddr;
#ifdef CONFIG_64BIT
	module->pdaf_buf_reserved.kaddr[1] = pdaf_kaddr >> 32;
#endif
	module->pdaf_buf_reserved.addr_offset = addr_offset;
	module->pdaf_buf_reserved.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
	module->pdaf_buf_reserved.buf_size = ISP_PDAF_STATIS_BUF_SIZE;
	module->pdaf_buf_reserved.buf_property = ISP_PDAF_BLOCK;
	module->pdaf_buf_reserved.buf_info.dev = dev;

	/*nr3 statis buf cfg*/
	nr3_iova_addr = pdaf_iova_addr + ISP_PDAF_STATIS_BUF_SIZE;
	nr3_vir_addr = pdaf_vir_addr + ISP_PDAF_STATIS_BUF_SIZE;
	nr3_kaddr = pdaf_kaddr + ISP_PDAF_STATIS_BUF_SIZE;
	addr_offset += ISP_PDAF_STATIS_BUF_SIZE;
	/*slip the big buffer to some little buffer*/
	for (cnt = 0; cnt < ISP_BINNING_STATIS_BUF_NUM; cnt++) {
		nr3_frm_statis.phy_addr = nr3_iova_addr;
		nr3_frm_statis.vir_addr = nr3_vir_addr;
		nr3_frm_statis.kaddr[0] = nr3_kaddr;
	#ifdef CONFIG_64BIT
		nr3_frm_statis.kaddr[1] = nr3_kaddr >> 32;
	#endif
		nr3_frm_statis.addr_offset = addr_offset;
		nr3_frm_statis.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
		nr3_frm_statis.buf_size = ISP_BINNING_STATIS_BUF_SIZE;
		nr3_frm_statis.buf_property = ISP_NR3_BLOCK;
		nr3_frm_statis.buf_info.dev = dev;
		ret = sprd_cam_buf_queue_write(&module->nr3_statis_queue,
					&nr3_frm_statis);
		nr3_iova_addr += ISP_BINNING_STATIS_BUF_SIZE;
		nr3_vir_addr += ISP_BINNING_STATIS_BUF_SIZE;
		nr3_kaddr += ISP_BINNING_STATIS_BUF_SIZE;
		addr_offset += ISP_BINNING_STATIS_BUF_SIZE;
	}
	/*init reserved nr3 statis buf*/
	module->nr3_buf_reserved.phy_addr = nr3_iova_addr;
	module->nr3_buf_reserved.vir_addr = nr3_vir_addr;
	module->nr3_buf_reserved.kaddr[0] = nr3_kaddr;
	module->nr3_buf_reserved.kaddr[1] = nr3_kaddr;
	module->nr3_buf_reserved.addr_offset = addr_offset;
	module->nr3_buf_reserved.buf_info.mfd[0] = frm_statis.buf_info.mfd[0];
	module->nr3_buf_reserved.buf_size = ISP_BINNING_STATIS_BUF_SIZE;
	module->nr3_buf_reserved.buf_property = ISP_NR3_BLOCK;
	module->nr3_buf_reserved.buf_info.dev = dev;

	CAM_TRACE("cfg statis buf out.\n");
	return ret;
}

int sprd_cam_set_statis_addr(struct device *dev,
	struct cam_statis_module *module,
	struct isp_statis_buf_input *parm)
{
	int ret = 0;
	struct cam_statis_buf frm_statis;
	struct cam_statis_buf *statis_buf_reserved = NULL;
	struct cam_buf_queue *statis_queue = NULL;

	if (!dev || !module || !parm) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	switch (parm->buf_property) {
	case ISP_AEM_BLOCK:
		statis_queue = &module->aem_statis_queue;
		statis_buf_reserved = &module->aem_buf_reserved;
		break;
	case ISP_AFL_BLOCK:
		statis_queue = &module->afl_statis_queue;
		statis_buf_reserved = &module->afl_buf_reserved;
		break;
	case ISP_AFM_BLOCK:
		statis_queue = &module->afm_statis_queue;
		statis_buf_reserved = &module->afm_buf_reserved;
		break;
	case ISP_NR3_BLOCK:
		statis_queue = &module->nr3_statis_queue;
		statis_buf_reserved = &module->nr3_buf_reserved;
		break;
	case ISP_PDAF_BLOCK:
		statis_queue = &module->pdaf_statis_queue;
		statis_buf_reserved = &module->pdaf_buf_reserved;
		break;
	default:
		pr_err("fail to get statis block %d\n", parm->buf_property);
		return -EFAULT;
	}

	/*config statis buf*/
	if (parm->is_statis_buf_reserved == 1) {
		statis_buf_reserved->phy_addr = parm->phy_addr;
		statis_buf_reserved->vir_addr = parm->vir_addr;
		statis_buf_reserved->addr_offset = parm->addr_offset;
		statis_buf_reserved->kaddr[0] = parm->kaddr[0];
		statis_buf_reserved->kaddr[1] = parm->kaddr[1];
		statis_buf_reserved->buf_size = parm->buf_size;
		statis_buf_reserved->buf_property =
			parm->buf_property;
		statis_buf_reserved->buf_info.dev = dev;
		statis_buf_reserved->buf_info.mfd[0] = parm->reserved[0];
	} else {
		memset((void *)&frm_statis, 0x00, sizeof(frm_statis));
		frm_statis.phy_addr = parm->phy_addr;
		frm_statis.vir_addr = parm->vir_addr;
		frm_statis.addr_offset = parm->addr_offset;
		frm_statis.kaddr[0] = parm->kaddr[0];
		frm_statis.kaddr[1] = parm->kaddr[1];
		frm_statis.buf_size = parm->buf_size;
		frm_statis.buf_property = parm->buf_property;

		frm_statis.buf_info.dev = dev;
		frm_statis.buf_info.mfd[0] = parm->reserved[0];
		/*when the statis is running, we need not map again*/
		ret = sprd_cam_buf_queue_write(statis_queue, &frm_statis);
	}

	CAM_TRACE("set statis buf addr done.\n");
	return ret;
}

int sprd_cam_set_statis_buf(int idx, struct cam_statis_module *module)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;

	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

#if 0
	sprd_aem_fake(idx);
#endif

	rtn = sprd_cam_set_next_statis_buf(idx, module,
				      ISP_AEM_BLOCK);
	if (rtn) {
		pr_err("fail to set next AEM statis buf\n");
		return -(rtn);
	}
	dcam_force_copy(idx, AEM_COPY);
	rtn = sprd_cam_set_next_statis_buf(idx, module,
				      ISP_AFL_BLOCK);
	if (rtn) {
		pr_err("fail to set next AFL statis buf\n");
		return -(rtn);
	}
	dcam_force_copy(idx, AEM_COPY);
	rtn = sprd_cam_set_next_statis_buf(idx, module,
				ISP_AFM_BLOCK);
	if (rtn) {
		pr_err("fail to set next AFM statis buf\n");
		return -(rtn);
	}
	dcam_force_copy(idx, BIN_COPY);
	rtn = sprd_cam_set_next_statis_buf(idx, module,
				ISP_NR3_BLOCK);
	if (rtn) {
		pr_err("fail to set next nr3 statis buf\n");
		return -(rtn);
	}
	dcam_force_copy(idx, BIN_COPY);

	rtn = sprd_cam_set_next_statis_buf(idx, module,
				ISP_PDAF_BLOCK);
	if (rtn) {
		pr_err("fail to set next pdaf statis buf\n");
		return -(rtn);
	}
	dcam_force_copy(idx, PDAF_COPY);
	dcam_force_copy(idx, VCH2_COPY);

	CAM_TRACE("set statis buf done.\n");
	return rtn;
}
