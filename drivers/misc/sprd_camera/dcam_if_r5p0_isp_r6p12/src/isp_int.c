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

#include <linux/interrupt.h>

#include "isp_int.h"
#include "isp_buf.h"
#include "isp_path.h"
#include "isp_block.h"
#include "cam_common.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_INT: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static isp_isr_func p_user_func[ISP_ID_MAX][ISP_IMG_MAX];
static void *p_user_data[ISP_ID_MAX][ISP_IMG_MAX];
static spinlock_t isp_irq0_lock[ISP_MAX_COUNT];
static spinlock_t isp_irq1_lock[ISP_MAX_COUNT];

static const uint32_t isp_irq_p0[] = {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_STORE_DONE_VID,
	ISP_INT_NR3_ALL_DONE,
	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_FMCU_CONFIG_DONE,
	ISP_INT_FMCU_CMD_ERROR,
};

static const uint32_t isp_irq_p1[] = {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_STORE_DONE_VID,
	ISP_INT_NR3_ALL_DONE,
	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_FMCU_CONFIG_DONE,
	ISP_INT_FMCU_CMD_ERROR,
};

static const uint32_t isp_irq_c0[] = {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_NR3_ALL_DONE,
	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_FMCU_CONFIG_DONE,
	ISP_INT_FMCU_CMD_ERROR,
};

static const uint32_t isp_irq_c1[] = {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_NR3_ALL_DONE,
	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_FMCU_CONFIG_DONE,
	ISP_INT_FMCU_CMD_ERROR,
};

static void isp_fmcu_config_done(enum isp_id idx, void *isp_handle);

static void isp_ch0_fmcu_cmd_error(uint32_t idx)
{
	unsigned long addr = 0;

	pr_info("fmcu cmd error Register list\n");
	for (addr = 0x0988; addr <= 0x0998 ; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr),
			ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8),
			ISP_REG_RD(idx, addr + 12));
	}
	pr_info("C0 interrupt status\n");
	for (addr = 0x0e00; addr <= 0x0e3c ; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr),
			ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8),
			ISP_REG_RD(idx, addr + 12));
	}
	pr_info("3DNR status\n");
	for (addr = 0x9010; addr <= 0x903c ; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr),
			ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8),
			ISP_REG_RD(idx, addr + 12));
	}
	pr_info("3DNR blend\n");
	for (addr = 0x9110; addr <= 0x9170 ; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr),
			ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8),
			ISP_REG_RD(idx, addr + 12));
	}
}

static int isp_err_pre_proc(uint32_t idx)
{

	isp_ch0_fmcu_cmd_error(idx);

	return 0;
}

#if 0
static void isp_irq_reg(enum isp_id idx,
		enum isp_irq_id irq_id, void *param)
{
	isp_isr_func user_func;
	void *user_data;
	struct camera_frame *frame_info = NULL;

	user_func = p_user_func[idx][irq_id];
	user_data = p_user_data[idx][irq_id];

	frame_info = (struct camera_frame *)param;
	if (!frame_info)
		return;

	if (user_func)
		(*user_func)(frame_info, user_data);
}
#endif

static void isp_path_done(enum isp_id idx, enum isp_scl_id path_id,
	void *isp_handle)
{
	int  ret = 0;
	void *data;
	isp_isr_func user_func;
	enum isp_irq_id img_id = ISP_PATH_PRE_DONE;
	struct camera_frame frame;
	struct isp_path_desc *path;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	if (path_id >= ISP_SCL_MAX) {
		pr_err("fail to get valid img_id %d\n.", path_id);
		return;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	path = &module->isp_path[path_id];
	if (path->valid == 0) {
		pr_info("isp%d:fail to get valid path%d\n", idx, path_id);
		return;
	}

	if (path_id == ISP_SCL_PRE)
		img_id = ISP_PATH_PRE_DONE;
	else if (path_id == ISP_SCL_VID)
		img_id = ISP_PATH_VID_DONE;
	else if (path_id == ISP_SCL_CAP)
		img_id = ISP_PATH_CAP_DONE;
	else {
		pr_err("fail to get valid isp path id %d.\n", path_id);
		return;
	}
	user_func = p_user_func[idx][img_id];
	data = p_user_data[idx][img_id];

	ret = sprd_cam_frm_dequeue(&path->frame_queue, &frame);
	if (ret) {
		pr_info("fail to dequeue frame queue.\n");
		return;
	}

	if (frame.buf_info.dev == NULL)
		pr_info("ISP%d:fail to done dev %p\n",
			idx, frame.buf_info.dev);
	cam_buf_unmap_addr(&frame.buf_info);

	if (!cam_buf_is_equal(&frame.buf_info,
		&path->path_reserved_frame.buf_info)) {
		frame.width = path->dst.w;
		frame.height = path->dst.h;
		frame.irq_type = CAMERA_IRQ_IMG;
		pr_debug("ISP%d: path%d frame %p\n",
			idx, path_id, &frame);
		if (user_func)
			user_func(&frame, data);
	} else {
		pr_debug("isp%d: use reserved [%d]\n", idx, path_id);
		module->path_reserved_frame[path_id].buf_info.iova[0] = 0;
		module->path_reserved_frame[path_id].buf_info.iova[1] = 0;
	}
	dev->isp_busy = 0;
	pr_debug("isp%d path%d done.\n", idx, img_id);
}

static void isp_path_shadow_done(enum isp_id idx, enum isp_scl_id path_id,
	void *isp_handle)
{
	enum isp_path_index path_index = ISP_PATH_IDX_PRE;
	struct isp_path_desc *path = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	path = &module->isp_path[path_id];

	if (path_id == ISP_SCL_PRE)
		path_index = ISP_PATH_IDX_PRE;
	else if (path_id == ISP_SCL_VID)
		path_index = ISP_PATH_IDX_VID;
	else if (path_id == ISP_SCL_CAP)
		path_index = ISP_PATH_IDX_CAP;
	else {
		pr_err("fail to get valid path id %d\n", path_id);
		return;
	}

	if (path->status == ISP_ST_START) {
		pr_debug("ISP%d: shadow s %d\n", idx, path_id);
		if (path->valid == 0) {
			pr_info("ISP%d:fail to get valid path%d\n",
				idx, path_id);
			return;
		}
	}
}

static void isp_shadow_done(enum isp_id idx, void *isp_handle)
{
	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return;
	}
	pr_debug("in.\n");
#if 0
	user_func = p_user_func[idx][ISP_SHADOW_DONE];
	data = p_user_data[idx][ISP_SHADOW_DONE];

	frame.irq_type = CAMERA_IRQ_DONE;
	frame.irq_property = IRQ_SHADOW_DONE;

	if (user_func)
		user_func(&frame, data);
#endif
	isp_path_shadow_done(idx, ISP_SCL_PRE, isp_handle);

}

static void isp_free_dcam_frame(enum isp_id idx, void *isp_handle)
{
	int ret = 0;
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)isp_handle;
	struct isp_module *module = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_frame frame;

	module = &dev->module_info;

	if (sprd_cam_frm_get_firstnode(&module->bin_frm_queue,
		(void **)&pframe) != 0) {
		pr_err("fail to get bin frm frame\n");
		return;
	}

	if (atomic_dec_return(&pframe->usr_cnt) != 0)
		return;

	/*write this buffer back to DCAM */
	if (sprd_cam_frm_dequeue(&module->bin_frm_queue, &frame)) {
		pr_err("fail to dequeue bin frame\n");
		return;
	}

	cam_buf_unmap_addr(&frame.buf_info);
	ret = dcam_set_frame_addr(dev->isp_handle_addr,
			CAMERA_BIN_PATH, &frame);
	if (ret)
		pr_err("fail to set frame back to dcam.\n");
}

static void isp_store_vid_done(enum isp_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;

	pr_debug("store vid done.\n");
	if (module->isp_path[ISP_SCL_VID].valid) {
		isp_path_done(idx, ISP_SCL_VID, dev);
		isp_free_dcam_frame(idx, isp_handle);
	}

	if (module->isp_path[ISP_SCL_CAP].valid && s_isp_group.dual_cap_sts)
		isp_fmcu_config_done(idx, dev);
}

static void isp_store_pre_done(enum isp_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;

	if (module->isp_path[ISP_SCL_PRE].valid) {
		isp_path_done(idx, ISP_SCL_PRE, dev);
		isp_free_dcam_frame(idx, isp_handle);
	}
}

static void isp_store_cap_done(enum isp_id idx, void *isp_handle)
{
	pr_info("isp%d isp_store_cap_done.\n", idx);
}

static void isp_p_all_done(enum isp_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)isp_handle;
	struct isp_module *module = NULL;
	struct camera_frame *pframe = NULL;

	module = &dev->module_info;

	if (sprd_cam_frm_get_firstnode(&module->bin_frm_queue,
		(void **)&pframe) != 0) {
		pr_err("fail to get bin frm frame\n");
		return;
	}
	CAM_TRACE("isp all done. isp_id:%d usr_cnt %d\n", idx,
		atomic_read(&pframe->usr_cnt));

	if (module->isp_path[ISP_SCL_PRE].valid
		&& module->isp_path[ISP_SCL_VID].valid) {
		if (atomic_read(&pframe->usr_cnt) > 1) {
			isp_store_pre_done(idx, isp_handle);
			isp_store_vid_done(idx, isp_handle);
		} else
			isp_store_vid_done(idx, isp_handle);
	} else if (module->isp_path[ISP_SCL_PRE].valid) {
		isp_store_pre_done(idx, isp_handle);
	} else if (module->isp_path[ISP_SCL_VID].valid) {
		isp_store_vid_done(idx, isp_handle);
	}
}

static void isp_c_all_done(enum isp_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;

	CAM_TRACE("isp all done. isp_id:%d\n", idx);
}

static void isp_fmcu_config_done(enum isp_id idx, void *isp_handle)
{
	int ret = 0;
	void *data;
	isp_isr_func user_func;
	enum dcam_id id = ISP_ID_0;
	enum isp_scl_id path_id = ISP_SCL_CAP;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_dev *dev_dual = NULL;
	struct isp_path_desc *path_pre = NULL;
	struct isp_path_desc *path_vid = NULL;
	struct isp_path_desc *path_cap = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct camera_frame frame;
	struct isp_module *module = NULL;
	struct isp_nr3_param *nr3_info = NULL;
	struct sprd_img_capture_param capture_param;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	memset(&frame, 0x00, sizeof(frame));
	memset(&capture_param, 0x00, sizeof(capture_param));

	id = idx;
	dev = (struct isp_pipe_dev *)isp_handle;
	dev->is_wait_fmcu = 0;

	fmcu_slice = &dev->fmcu_slice;
	module = &dev->module_info;
	path_pre = &module->isp_path[ISP_SCL_PRE];
	path_vid = &module->isp_path[ISP_SCL_VID];
	path_cap = &module->isp_path[ISP_SCL_CAP];
	nr3_info = &path_cap->nr3_param;
	dev_dual = (struct isp_pipe_dev *)s_isp_group.isp_dev[id ^ 1];

	pr_info("isp dev handle %p\n", dev);
	pr_debug("path_cap->valid = %d\n", path_cap->valid);
	if (path_cap->valid) {
		if (!nr3_info->need_3dnr || (nr3_info->need_3dnr
			&& nr3_info->cur_cap_frame == ISP_3DNR_NUM)) {
			user_func = p_user_func[idx][ISP_PATH_CAP_DONE];
			data = p_user_data[idx][ISP_PATH_CAP_DONE];
			ret = sprd_cam_frm_dequeue(
				&path_cap->frame_queue, &frame);
			if (ret) {
				dev->isp_busy = 0;
				pr_info("fail to dequeue isp%d cap frame\n",
					idx);
				return;
			}
			cam_buf_unmap_addr(&frame.buf_info);
			if (!cam_buf_is_equal(&frame.buf_info,
				&path_cap->path_reserved_frame.buf_info)) {
				frame.width = path_cap->dst.w;
				frame.height = path_cap->dst.h;
				frame.irq_type = CAMERA_IRQ_IMG;
				if (dev->is_raw_capture == 1) {
					frame.irq_type = CAMERA_IRQ_DONE;
					frame.irq_property = IRQ_RAW_CAP_DONE;
				}

				if (user_func)
					(*user_func)(&frame, data);
			} else {
				pr_info("isp%d: use reserved cap\n", idx);
				module->path_reserved_frame
					[path_id].buf_info.iova[0] = 0;
				module->path_reserved_frame
					[path_id].buf_info.iova[1] = 0;
			}
		}
		cam_buf_unmap_addr(&dev->capture_frame.buf_info);
		if (dev->is_raw_capture == 1)
			ret = dcam_set_frame_addr(dev->isp_handle_addr,
				CAMERA_BIN_PATH, &dev->capture_frame);
		else
			ret = dcam_set_frame_addr(dev->isp_handle_addr,
				CAMERA_FULL_PATH, &dev->capture_frame);
		if (ret)
			pr_err("fail to set back capture buf");
	}

	if (dev->is_raw_capture == 1) {
		path_cap->valid = 0;
		path_cap->status = ISP_ST_STOP;
		dev->is_raw_capture = 0;
		s_isp_group.dual_cap_sts = 0;
		/*dev->cap_flag = DCAM_CAPTURE_STOP;*/
		sprd_isp_stop(dev, 1);
		return;
	}

	if (dev->cap_flag == DCAM_CAPTURE_START_HDR
		|| dev->cap_flag == DCAM_CAPTURE_START_3DNR) {
		pr_debug("dev->cap_flag %d\n", dev->cap_flag);
		dev->isp_busy = 0;
		s_isp_group.dual_cap_sts = 0;
		capture_param.type = DCAM_CAPTURE_START;
		if (sprd_isp_start_fmcu(isp_handle,
			capture_param, 1, ISP_PATH_IDX_CAP)) {
			pr_info("fail to start slice capture\n");
			return;
		}
	} else {
		/*unmap left frame in zsl queue*/
		while (!sprd_cam_frm_dequeue(&module->full_zsl_queue, &frame)) {
			cam_buf_unmap_addr(&frame.buf_info);
			dcam_set_frame_addr(dev->isp_handle_addr,
				CAMERA_FULL_PATH, &frame);
		}
		dcam_full_path_clear(id);
		sprd_cam_frm_queue_clear(&module->full_zsl_queue);
		dcam_full_path_set_next_frm(id);
		dev->isp_offline_state = ISP_ST_START;
		if (s_isp_group.dual_cam) {
			fmcu_slice->capture_state = ISP_ST_STOP;
			if (++s_isp_group.dual_cap_cnt == 1) {
				pr_info("start another cap and buf in dual cam!\n");
				if (s_isp_group.first_need_wait) {
					pr_info("post dual first frame done.\n");
					s_isp_group.dual_cap_sts = 0;
					capture_param.type = DCAM_CAPTURE_START;
					if (sprd_isp_start_fmcu(dev_dual,
						capture_param, 1,
						ISP_PATH_IDX_CAP)) {
						pr_info("fail to start slice capture\n");
						return;
					}
				}
				s_isp_group.first_need_wait = 0;
			} else if (s_isp_group.dual_cap_cnt == 2) {
				sprd_dcam_path_resume(DCAM_ID_0,
					CAMERA_FULL_PATH);
				sprd_dcam_path_resume(DCAM_ID_1,
					CAMERA_FULL_PATH);
				s_isp_group.dual_fullpath_stop = 0;
				s_isp_group.dual_cap_cnt = 0;
				s_isp_group.dual_cap_total++;
				pr_info("dual capture finish!total %d\n",
					s_isp_group.dual_cap_total);
			}
		} else
			sprd_dcam_path_resume(id, CAMERA_FULL_PATH);
		s_isp_group.dual_cap_sts = 0;
	}
	dev->isp_busy = 0;
	pr_info("isp%d end\n", idx);
}

static isp_isr isp_isr_list[ISP_CONTEX_MAX][32] = {
	[ISP_CONTEX_P0][ISP_INT_ISP_ALL_DONE] = isp_p_all_done,
	[ISP_CONTEX_P0][ISP_INT_SHADOW_DONE] = isp_shadow_done,
	[ISP_CONTEX_P0][ISP_INT_STORE_DONE_VID] = NULL,
	[ISP_CONTEX_P0][ISP_INT_STORE_DONE_PRE] = NULL,
	[ISP_CONTEX_P0][ISP_INT_FMCU_CONFIG_DONE] = NULL,

	[ISP_CONTEX_P1][ISP_INT_ISP_ALL_DONE] = isp_p_all_done,
	[ISP_CONTEX_P1][ISP_INT_SHADOW_DONE] = isp_shadow_done,
	[ISP_CONTEX_P1][ISP_INT_STORE_DONE_VID] = NULL,
	[ISP_CONTEX_P1][ISP_INT_STORE_DONE_PRE] = NULL,
	[ISP_CONTEX_P1][ISP_INT_FMCU_CONFIG_DONE] = NULL,

	[ISP_CONTEX_C0][ISP_INT_ISP_ALL_DONE] = isp_c_all_done,
	[ISP_CONTEX_C0][ISP_INT_SHADOW_DONE] = isp_shadow_done,
	[ISP_CONTEX_C0][ISP_INT_STORE_DONE_PRE] = isp_store_cap_done,
	[ISP_CONTEX_C0][ISP_INT_FMCU_CONFIG_DONE] = isp_fmcu_config_done,

	[ISP_CONTEX_C1][ISP_INT_ISP_ALL_DONE] = isp_c_all_done,
	[ISP_CONTEX_C1][ISP_INT_SHADOW_DONE] = isp_shadow_done,
	[ISP_CONTEX_C1][ISP_INT_STORE_DONE_PRE] = isp_store_cap_done,
	[ISP_CONTEX_C1][ISP_INT_FMCU_CONFIG_DONE] = isp_fmcu_config_done,
};

static irqreturn_t isp_isr_root(int irq, void *priv)
{
	uint32_t irq_line[CFG_CONTEXT_NUM] = {0};
	uint32_t irq_numbers[CFG_CONTEXT_NUM] = {0};
	uint32_t j = 0, k = 0, vect = 0;
	unsigned long flag = 0;
	unsigned long base_addr = s_isp_regbase[0];
	enum isp_id id = ISP_ID_0;
	struct isp_pipe_dev *isp_handle = NULL;

	isp_handle = (struct isp_pipe_dev *)priv;
	irq_numbers[0] = ARRAY_SIZE(isp_irq_p0);
	irq_numbers[1] = ARRAY_SIZE(isp_irq_p1);
	irq_numbers[2] = ARRAY_SIZE(isp_irq_c0);
	irq_numbers[3] = ARRAY_SIZE(isp_irq_c1);

	id = ISP_GET_ISP_ID(isp_handle->com_idx);

	base_addr = s_isp_regbase[id];

	if (irq == s_isp_irq[ISP_ID_0].irq0) {
		irq_line[0] = REG_RD(base_addr +
			ISP_P0_INT_BASE + ISP_INT_INT0) & ISP_INT_LINE_MASK_P0;
		irq_line[2] = REG_RD(base_addr +
			ISP_C0_INT_BASE + ISP_INT_INT0) & ISP_INT_LINE_MASK_C0;
	} else if (irq == s_isp_irq[ISP_ID_0].irq1) {
		irq_line[1] = REG_RD(base_addr +
			ISP_P1_INT_BASE + ISP_INT_INT0) & ISP_INT_LINE_MASK_P1;
		irq_line[3] = REG_RD(base_addr +
			ISP_C1_INT_BASE + ISP_INT_INT0) & ISP_INT_LINE_MASK_C1;
	} else {
		pr_info("isp IRQ  %d,  %d\n", irq, s_isp_irq[ISP_ID_0].irq0);
		return IRQ_HANDLED;
	}
	pr_debug("isp IRQ  %d,  %p, %d  %d\n", irq, priv, s_isp_irq[id].irq0,
		s_isp_irq[id].irq1);

	if (unlikely(irq_line[0] == 0 && irq_line[1] == 0
		&& irq_line[2] == 0 && irq_line[3] == 0))
		return IRQ_NONE;
	pr_debug("isp IRQ is  %d, INTP0:0x%x, INTP1:0x%x, INTC0:0x%x, INTC1:0x%x\n",
		irq, irq_line[0], irq_line[1], irq_line[2], irq_line[3]);

	/*clear the interrupt*/
	if (irq == s_isp_irq[ISP_ID_0].irq0) {
		REG_WR(base_addr + ISP_P0_INT_BASE + ISP_INT_CLR0, irq_line[0]);
		REG_WR(base_addr + ISP_C0_INT_BASE + ISP_INT_CLR0, irq_line[2]);
	} else {
		REG_WR(base_addr + ISP_P1_INT_BASE + ISP_INT_CLR0, irq_line[1]);
		REG_WR(base_addr + ISP_C1_INT_BASE + ISP_INT_CLR0, irq_line[3]);
	}

	if (unlikely(ISP_IRQ_ERR_MASK_P0 & irq_line[0])
		|| unlikely(ISP_IRQ_ERR_MASK_P1 & irq_line[1])
		|| unlikely(ISP_IRQ_ERR_MASK_C0 & irq_line[2])
		|| unlikely(ISP_IRQ_ERR_MASK_C1 & irq_line[3])) {
		pr_err("isp%d IRQ is error, INTP0:0x%x, INTP1:0x%x, INTC0:0x%x, INTC1:0x%x\n",
			id, irq_line[0], irq_line[1], irq_line[2], irq_line[3]);
		/*handle the error here*/
		if (isp_err_pre_proc(isp_handle->com_idx))
			return IRQ_HANDLED;
	}

	/*spin_lock_irqsave protect the isr_func*/
	spin_lock_irqsave(&isp_mod_lock, flag);
	for (j = 0; j < CFG_CONTEXT_NUM; j++) {
		for (k = 0; k < irq_numbers[j]; k++) {
			if (j == 0 && irq == s_isp_irq[ISP_ID_0].irq0)
				vect = isp_irq_p0[k];
			else if (j == 1 && irq == s_isp_irq[ISP_ID_0].irq1)
				vect = isp_irq_p1[k];
			else if (j == 2 && irq == s_isp_irq[ISP_ID_0].irq0)
				vect = isp_irq_c0[k];
			else if (j == 3 && irq == s_isp_irq[ISP_ID_0].irq1)
				vect = isp_irq_c1[k];
			if (irq_line[j] & (1 << (uint32_t)vect)) {
				if (isp_isr_list[j][vect])
					isp_isr_list[j][vect](id,
						isp_handle);
			}
			irq_line[j] &= ~(1 << (uint32_t)vect);
			if (!irq_line[j])
				break;
		}
	}

	/*spin_unlock_irqrestore*/
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	return IRQ_HANDLED;
}

int isp_irq_request(struct device *p_dev, struct isp_ch_irq *irq,
	struct isp_pipe_dev *ispdev)
{
	int ret = 0;
	enum isp_id id = 0;

	if (!p_dev || !irq || !ispdev) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	id = ISP_GET_ISP_ID(ispdev->com_idx);
	isp_irq0_lock[id] = __SPIN_LOCK_UNLOCKED(&isp_irq0_lock[id]);
	isp_irq1_lock[id] = __SPIN_LOCK_UNLOCKED(&isp_irq1_lock[id]);

	if (id == ISP_ID_0) {
		ret = request_irq(irq->irq0, isp_isr_root,
			IRQF_SHARED, "ISP0", (void *)ispdev);
		if (ret) {
			pr_err("fail to install IRQ irq0 %d\n", ret);
			goto exit;
		}
	} else if (id == ISP_ID_1) {
		ret = request_irq(irq->irq1, isp_isr_root,
			IRQF_SHARED, "ISP1", (void *)ispdev);
		if (ret) {
			pr_err("fail to install IRQ irq1 %d\n", ret);
			goto exit;
		}
	} else {
		pr_info("fail to get right isp id %d %p\n", id, ispdev);
		return -EFAULT;
	}

	pr_info("isp IRQ %p %d  %d\n", ispdev, s_isp_irq[id].irq0,
		s_isp_irq[id].irq1);
exit:
	return ret;
}

int isp_irq_free(struct isp_ch_irq *irq, struct isp_pipe_dev *ispdev)
{
	int ret = 0;
	enum isp_id id = 0;

	if (!irq || !ispdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	id = ISP_GET_ISP_ID(ispdev->com_idx);
	isp_irq0_lock[id] = __SPIN_LOCK_UNLOCKED(&isp_irq0_lock[id]);
	isp_irq1_lock[id] = __SPIN_LOCK_UNLOCKED(&isp_irq1_lock[id]);

	if (id == ISP_ID_0)
		free_irq(irq->irq0, (void *)ispdev);
	else if (id == ISP_ID_1)
		free_irq(irq->irq1, (void *)ispdev);
	else {
		pr_info("fail to get right isp id %d %p\n", id, ispdev);
		return -EFAULT;
	}

	return ret;
}

int isp_irq_callback(enum isp_id id, enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data)
{
	unsigned long flag = 0;

	if (!user_data) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&isp_mod_lock, flag);
	p_user_func[id][irq_id] = user_func;
	p_user_data[id][irq_id] = user_data;
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	return 0;
}
