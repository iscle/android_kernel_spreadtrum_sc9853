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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pagemap.h>
#include <linux/sprd_iommu.h>
#include <video/sprd_mm.h>
#include <video/sprd_img.h>
#include <linux/clk-provider.h>

#include "isp_buf.h"
#include "isp_drv.h"
#include "isp_int.h"
#include "isp_path.h"
#include "isp_slice.h"
#include "cam_pw_domain.h"
#include "cam_gen_scale_coef.h"
#include "isp_3dnr_cap.h"
#include "isp_3dnr_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

struct platform_device *s_isp_pdev;
struct isp_group s_isp_group;
static atomic_t s_isp_users[ISP_MAX_COUNT];
static atomic_t s_isp_total_users;
static uint32_t s_isp_count;
unsigned long s_isp_regbase[ISP_MAX_COUNT];
unsigned long isp_phys_base[ISP_MAX_COUNT];

static struct clk *isp_clk;
static struct clk *isp_clk_parent;
static struct clk *isp_clk_default;
static struct clk *isp_axi_eb;
static struct clk *isp_eb;


struct isp_ch_irq s_isp_irq[ISP_MAX_COUNT];
static struct mutex isp_module_sema[ISP_MAX_COUNT];
static struct regmap *cam_ahb_gpr;
static struct regmap *aon_apb_gpr;

static spinlock_t isp_glb_reg_axi_lock[ISP_MAX_COUNT];
static spinlock_t isp_glb_reg_mask_lock[ISP_MAX_COUNT];
static spinlock_t isp_glb_reg_clr_lock[ISP_MAX_COUNT];
spinlock_t isp_mod_lock;
#define ISP_AXI_STOP_TIMEOUT           1000

#define ISP_CLK_NUM                    5
#define SHAR                           0x53686172
#define KLJ1                           0x6B4C4A31
#define CLK_CPPLL                      468000000

static const struct isp_clk_tag isp_clk_tab[ISP_CLK_NUM] = {
	{"128", "isp_clk_128m"},
	{"256", "isp_clk_256m"},
	{"307", "isp_clk_307m2"},
	{"384", "isp_clk_384m"},
	{"max", "isp_clk_max"},
};

void isp_reg_trace(enum isp_id idx)
{
#ifdef ISP_DRV_DEBUG
	unsigned long addr = 0;

	pr_info("ISP%d: Register list:\n", idx);
	for (addr = ISP_INT_EN0;
		addr <= ISP_INT_ALL_DONE_SRC_CTRL; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr),
			ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8),
			ISP_REG_RD(idx, addr + 12));
	}
#endif
}
EXPORT_SYMBOL(isp_reg_trace);

static int offline_thread_loop(void *arg)
{
	uint32_t is_irq = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	dev = (struct isp_pipe_dev *)arg;

	while (1) {
		if (wait_for_completion_interruptible(
			&dev->offline_thread_com) == 0) {
			if (dev->is_offline_thread_stop) {
				pr_info("offline thread loop stop\n");
				break;
			}

			if (sprd_isp_offline_proc(arg, is_irq))
				pr_info("fail to start slice capture\n");

		} else {
			pr_info("offline thread exit!");
			break;
		}
	}

	dev->is_offline_thread_stop = 0;

	return 0;
}

static int isp_create_offline_thread(void *param)
{
	char thread_name[20] = { 0 };
	enum isp_id id = ISP_ID_0;
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;

	if (dev == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}
	id = ISP_GET_ISP_ID(dev->com_idx);

	dev->is_wait_fmcu = 0;
	dev->is_offline_thread_stop = 0;
	init_completion(&dev->offline_thread_com);
	sprintf(thread_name, "isp%d_offline_thread", id);
	dev->offline_thread = kthread_run(offline_thread_loop,
					param, thread_name);
	if (IS_ERR(dev->offline_thread)) {
		pr_err("fail to create_offline_thread %ld\n",
				PTR_ERR(dev->offline_thread));
		dev->offline_thread = NULL;
		return -1;
	}

	pr_info("offline_thread id %d\n", id);
	return 0;
}

static int isp_stop_offline_thread(void *param)
{
	int cnt = 0;
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;

	if (dev == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	if (dev->offline_thread) {
		dev->is_offline_thread_stop = 1;
		complete(&dev->offline_thread_com);
		if (dev->is_offline_thread_stop != 0) {
			while (cnt < 500) {
				cnt++;
				if (dev->is_offline_thread_stop == 0)
					break;
				udelay(1000);
			}
		}
		pr_info("done, cnt %d\n", cnt);
		dev->offline_thread = NULL;
	}

	return 0;
}

static void sprd_isp_glb_reg_awr(uint32_t idx, unsigned long addr,
			uint32_t val, uint32_t reg_id)
{
	unsigned long flag;
	enum isp_id id = 0;

	id = ISP_GET_ISP_ID(idx);

	switch (reg_id) {
	case ISP_AXI_REG:
		spin_lock_irqsave(&isp_glb_reg_axi_lock[id], flag);
		ISP_HREG_WR(id, addr, ISP_REG_RD(id, addr) & (val));
		spin_unlock_irqrestore(&isp_glb_reg_axi_lock[id], flag);
		break;
	default:
		ISP_HREG_WR(id, addr, ISP_REG_RD(id, addr) & (val));
		break;
	}
}

static void sprd_isp_glb_reg_owr(uint32_t idx, unsigned long addr,
			uint32_t val, uint32_t reg_id)
{
	unsigned long flag = 0;
	enum isp_id id = 0;

	id = ISP_GET_ISP_ID(idx);

	switch (reg_id) {
	case ISP_AXI_REG:
		spin_lock_irqsave(&isp_glb_reg_axi_lock[id], flag);
		ISP_HREG_WR(id, addr, ISP_REG_RD(id, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_axi_lock[id], flag);
		break;
	case ISP_INIT_MASK_REG:
		spin_lock_irqsave(&isp_glb_reg_mask_lock[id], flag);
		ISP_HREG_WR(id, addr, ISP_REG_RD(id, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_mask_lock[id], flag);
		break;
	case ISP_INIT_CLR_REG:
		spin_lock_irqsave(&isp_glb_reg_clr_lock[id], flag);
		ISP_HREG_WR(id, addr, ISP_REG_RD(id, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_clr_lock[id], flag);
		break;
	default:
		ISP_HREG_WR(id, addr, ISP_REG_RD(id, addr) | (val));
		break;
	}
}

static void sprd_isp_glb_reg_wr(uint32_t idx, unsigned long addr,
			uint32_t val, uint32_t reg_id)
{
	unsigned long flag = 0;
	enum isp_id id = 0;

	id = ISP_GET_ISP_ID(idx);

	switch (reg_id) {
	case ISP_INIT_MASK_REG:
		spin_lock_irqsave(&isp_glb_reg_mask_lock[id], flag);
		ISP_HREG_WR(id, addr, val);
		spin_unlock_irqrestore(&isp_glb_reg_mask_lock[id], flag);
		break;
	case ISP_INIT_CLR_REG:
		spin_lock_irqsave(&isp_glb_reg_clr_lock[id], flag);
		ISP_HREG_WR(id, addr, val);
		spin_unlock_irqrestore(&isp_glb_reg_clr_lock[id], flag);
		break;
	default:
		ISP_HREG_WR(id, addr, val);
		break;
	}
}

static void sprd_irq_mask_enable(uint32_t idx)
{
	sprd_isp_glb_reg_wr(idx, ISP_P0_INT_BASE + ISP_INT_EN0,
		ISP_INT_LINE_MASK_P0, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_wr(idx, ISP_P1_INT_BASE + ISP_INT_EN0,
		ISP_INT_LINE_MASK_P1, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_wr(idx, ISP_C0_INT_BASE + ISP_INT_EN0,
		ISP_INT_LINE_MASK_C0, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_wr(idx, ISP_C1_INT_BASE + ISP_INT_EN0,
		ISP_INT_LINE_MASK_C1, ISP_INIT_MASK_REG);
}

static void sprd_irq_mask_disable(uint32_t idx)
{
	sprd_isp_glb_reg_wr(idx, ISP_P0_INT_BASE + ISP_INT_EN0,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_wr(idx, ISP_P0_INT_BASE + ISP_INT_EN1,
		0x0, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_wr(idx, ISP_P1_INT_BASE + ISP_INT_EN0,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_wr(idx, ISP_P1_INT_BASE + ISP_INT_EN1,
		0x0, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_wr(idx, ISP_C0_INT_BASE + ISP_INT_EN0,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_wr(idx, ISP_C0_INT_BASE + ISP_INT_EN1,
		0x0, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_wr(idx, ISP_C1_INT_BASE + ISP_INT_EN0,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_wr(idx, ISP_C1_INT_BASE + ISP_INT_EN1,
		0x0, ISP_INIT_MASK_REG);
}

static void sprd_irq_clear(uint32_t idx)
{
	sprd_isp_glb_reg_wr(idx, ISP_P0_INT_BASE + ISP_INT_CLR0,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_wr(idx, ISP_P0_INT_BASE + ISP_INT_CLR1,
		0xFFFFFFFF, ISP_INIT_CLR_REG);

	sprd_isp_glb_reg_wr(idx, ISP_P1_INT_BASE + ISP_INT_CLR0,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_wr(idx, ISP_P1_INT_BASE + ISP_INT_CLR1,
		0xFFFFFFFF, ISP_INIT_CLR_REG);

	sprd_isp_glb_reg_wr(idx, ISP_C0_INT_BASE + ISP_INT_CLR0,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_wr(idx, ISP_C0_INT_BASE + ISP_INT_CLR1,
		0xFFFFFFFF, ISP_INIT_CLR_REG);

	sprd_isp_glb_reg_wr(idx, ISP_C1_INT_BASE + ISP_INT_CLR0,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_wr(idx, ISP_C1_INT_BASE + ISP_INT_CLR1,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
}

static int sprd_isp_reset(uint32_t idx)
{
	uint32_t flag = 0, time_out = 0;
	enum isp_id id = ISP_ID_0;
	enum dcam_drv_rtn rtn = ISP_RTN_SUCCESS;

	id = ISP_GET_ISP_ID(idx);

	pr_info("ISP%d: reset:\n", id);

	/* then wait for AHB busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		if (1 == ((ISP_REG_RD(id,  ISP_INT_STATUS) & BIT_3) >> 3))
			break;
	}

	if (time_out >= ISP_AXI_STOP_TIMEOUT) {
		pr_info("ISP%d: reset timeout %d\n", id, time_out);
		return ISP_RTN_TIME_OUT;
	}

	flag = BIT_MM_AHB_ISP_LOG_SOFT_RST;
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	sprd_isp_glb_reg_awr(idx,
		ISP_AXI_ITI2AXIM_CTRL, ~BIT_26, ISP_AXI_REG);

	pr_info("ISP%d:end\n", id);

	return -rtn;
}

static void isp_quickstop(uint32_t idx)
{
	enum isp_id id = 0;

	id = ISP_GET_ISP_ID(idx);

	sprd_isp_glb_reg_owr(idx,
		ISP_AXI_ITI2AXIM_CTRL, BIT_26, ISP_AXI_REG);

	pr_info("ISP%d:ISP_AXI_AXIM_CTRL 0x%x STATUS 0x%x\n",
		id, ISP_REG_RD(id, ISP_AXI_ITI2AXIM_CTRL),
		ISP_REG_RD(id, ISP_INT_STATUS));
	udelay(10);

}

int sprd_isp_stop(void *isp_handle, int is_irq)
{
	uint32_t i = 0;
	unsigned long flag = 0;
	enum isp_id id = ISP_ID_0;
	enum isp_work_mode work_mode = 0;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_cfg_ctx_desc *cfg_ctx = NULL;
	struct isp_k_block *isp_k_param = NULL;
	struct camera_frame frame;

	if (isp_handle == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	cfg_ctx = &module->isp_cfg_contex;
	isp_k_param = &dev->isp_k_param;
	id = ISP_GET_ISP_ID(dev->com_idx);
	work_mode = ISP_GET_MODE_ID(dev->com_idx);
	isp_k_param->isp_status = ISP_STOP;

	if (!is_irq) {
		mutex_lock(&dev->offline_thread_mutex);
		spin_lock_irqsave(&isp_mod_lock, flag);
	}

	isp_quickstop(dev->com_idx);
	sprd_irq_mask_disable(dev->com_idx);
	sprd_irq_clear(dev->com_idx);

	if (dev->is_wait_fmcu == 1)
		dev->is_wait_fmcu = 0;

	for (i = ISP_SCL_PRE; i < ISP_SCL_MAX; i++) {
		module->isp_path[i].status = ISP_ST_STOP;
		module->isp_path[i].valid = 0;
		module->isp_path[i].skip_num = 0;
		module->isp_path[i].frm_cnt = 0;
		module->isp_path[i].buf_cnt = 0;
		isp_frm_clear(dev, 1 << i);
	}

	module->frm_cnt = 0;
	dev->isp_busy = 0;
	dev->fmcu_slice.capture_state = ISP_ST_STOP;
	dev->clr_queue = 0;
	dev->cap_cur_cnt = 0;
	dev->cap_flag = DCAM_CAPTURE_STOP;
	dev->isp_offline_thread_flag = 1;

	while (!sprd_cam_buf_queue_read(&module->bin_buf_queue, &frame))
		cam_buf_unmap_addr(&frame.buf_info);
	while (!sprd_cam_frm_dequeue(&module->bin_frm_queue, &frame))
		cam_buf_unmap_addr(&frame.buf_info);

	sprd_cam_frm_queue_clear(&module->full_zsl_queue);
	sprd_cam_frm_queue_clear(&module->bin_frm_queue);

	cam_buf_unmap_addr(&dev->fmcu_slice.buf_info);
	if (work_mode == ISP_CFG_MODE) {
		isp_reset_cfg_buf(cfg_ctx);
		cam_buf_unmap_addr(&cfg_ctx->buf_info);
		isp_cfg_ctx_buf_init(cfg_ctx, 0);
	}

	if (isp_k_param->lsc_buf_phys_addr != 0x00) {
		isp_k_param->lsc_pfinfo.dev = &s_dcam_pdev->dev;
		cam_buf_unmap_addr(&isp_k_param->lsc_pfinfo);
	}

	if (!is_irq) {
		spin_unlock_irqrestore(&isp_mod_lock, flag);
		mutex_unlock(&dev->offline_thread_mutex);
	}

	sprd_isp_reset(dev->com_idx);
	sprd_irq_mask_disable(dev->com_idx);
	sprd_irq_clear(dev->com_idx);

	pr_info("end\n");

	return -rtn;
}

static int get_slice_init_scaler(struct slice_scaler_path *scaler,
	struct isp_path_desc *path)
{
	if (!scaler || !path) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	scaler->trim0_size_x = path->trim0_info.size_x;
	scaler->trim0_size_y = path->trim0_info.size_y;
	scaler->trim0_start_x = path->trim0_info.start_x;
	scaler->trim0_start_y = path->trim0_info.start_y;

	if (path->deci_info.deci_x_eb)
		scaler->deci_x = 1 << (path->deci_info.deci_x + 1);
	else
		scaler->deci_x = 1;

	if (path->deci_info.deci_y_eb)
		scaler->deci_y = 1 << (path->deci_info.deci_y + 1);
	else
		scaler->deci_y = 1;

	scaler->odata_mode = path->odata_mode;
	scaler->scaler_bypass = path->scaler_bypass;
	scaler->scaler_out_width = path->dst.w;
	scaler->scaler_out_height = path->dst.h;
	scaler->scaler_factor_in = path->scaler_info.scaler_factor_in;
	scaler->scaler_ver_factor_in = path->scaler_info.scaler_ver_factor_in;
	scaler->scaler_factor_out = path->scaler_info.scaler_factor_out;
	scaler->scaler_ver_factor_out = path->scaler_info.scaler_ver_factor_out;
	scaler->scaler_y_ver_tap = path->scaler_info.scaler_y_ver_tap;
	scaler->scaler_uv_ver_tap = path->scaler_info.scaler_uv_ver_tap;

	return 0;
}

static int get_slice_init_param(struct slice_param_in *in_ptr,
	struct isp_pipe_dev *dev, enum isp_scene_id scene_id)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_path_desc *path_cap = NULL;
	struct slice_scaler_path *scaler = NULL;
	struct isp_module *module = NULL;
	struct camera_frame *frame = NULL;
	struct isp_nr3_param *nr3_info  = NULL;
	struct slice_3dnr_info *slice_3dnr_in = NULL;

	if (!dev || !in_ptr) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	module = &dev->module_info;
	path_cap = &module->isp_path[ISP_SCL_CAP];
	nr3_info = &path_cap->nr3_param;

	in_ptr->com_idx = module->com_idx;
	ISP_SET_SCENE_ID(in_ptr->com_idx, ISP_SCENE_CAP);
	frame = &dev->capture_frame;

	pr_debug("fmcu yaddr 0x%x yaddr_vir 0x%x  fd 0x%x\n",
		frame->yaddr, frame->yaddr_vir, frame->buf_info.mfd[0]);

	in_ptr->fetch_addr.chn0 = frame->buf_info.iova[1] + frame->yaddr;
	in_ptr->fetch_addr.chn1 = frame->buf_info.iova[1] + frame->uaddr;
	in_ptr->fetch_addr.chn2 = frame->buf_info.iova[1] + frame->vaddr;
	in_ptr->img_size.width = frame->width;
	in_ptr->img_size.height = frame->height;
	in_ptr->fetch_format = ISP_FETCH_CSI2_RAW10;/*TBD*/
	in_ptr->fmcu_addr_vir = (uint32_t *)dev->fmcu_slice.buf_info.kaddr[0];

	if (nr3_info->need_3dnr) {
		slice_3dnr_in = &in_ptr->nr3_info;
		slice_3dnr_in->need_slice = 1;
		slice_3dnr_in->fetch_3dnr_frame.format =
			nr3_info->fetch_format;
		slice_3dnr_in->store_3dnr_frame.format =
			nr3_info->store_format;
		if (ISP_3DNR_NUM - module->full_zsl_queue.valid_cnt) {
			slice_3dnr_in->cur_frame_num = ISP_3DNR_NUM
				- module->full_zsl_queue.valid_cnt;
		} else {
			pr_err("Fail to get right zsl cnt %d\n",
				module->full_zsl_queue.valid_cnt);
			return -EFAULT;
		}
		nr3_info->cur_cap_frame = slice_3dnr_in->cur_frame_num;
		if (nr3_info->cur_cap_frame != ISP_3DNR_NUM)
			ISP_REG_MWR(module->com_idx,
				ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM,
				BIT_0, 1);
		slice_3dnr_in->mv_x = frame->mv.mv_x;
		slice_3dnr_in->mv_y = frame->mv.mv_y;
		if (slice_3dnr_in->cur_frame_num % 2) {
			slice_3dnr_in->store_3dnr_frame.addr.chn0 =
				nr3_info->buf_info.iova[1];
			slice_3dnr_in->store_3dnr_frame.addr.chn1 =
				nr3_info->buf_info.iova[1] +
				frame->width * frame->height;
			slice_3dnr_in->fetch_3dnr_frame.addr.chn0 =
				nr3_info->buf_info.iova[0];
			slice_3dnr_in->fetch_3dnr_frame.addr.chn1 =
				nr3_info->buf_info.iova[0] +
				frame->width * frame->height;
		} else {
			slice_3dnr_in->store_3dnr_frame.addr.chn0 =
				nr3_info->buf_info.iova[0];
			slice_3dnr_in->store_3dnr_frame.addr.chn1 =
				nr3_info->buf_info.iova[0] +
				frame->width * frame->height;
			slice_3dnr_in->fetch_3dnr_frame.addr.chn0 =
				nr3_info->buf_info.iova[1];
			slice_3dnr_in->fetch_3dnr_frame.addr.chn1 =
				nr3_info->buf_info.iova[1] +
				frame->width * frame->height;
		}
	}

	if (path_cap->valid) {
		in_ptr->cap_slice_need = 1;
		in_ptr->store_frame[SLICE_PATH_CAP].format =
			path_cap->store_info.color_format;
		in_ptr->store_frame[SLICE_PATH_CAP].size.width =
			path_cap->dst.w;
		in_ptr->store_frame[SLICE_PATH_CAP].size.height =
			path_cap->dst.h;
		scaler = &in_ptr->scaler_frame[SLICE_PATH_CAP];
		ret = get_slice_init_scaler(scaler, path_cap);

		if (!nr3_info->need_3dnr || (nr3_info->need_3dnr
			&& nr3_info->cur_cap_frame == ISP_3DNR_NUM)) {
			ret = isp_path_set_next_frm(module, ISP_PATH_IDX_CAP,
				&in_ptr->store_frame[SLICE_PATH_CAP].addr,
				frame);
		}
	}

	in_ptr->is_raw_capture = dev->is_raw_capture;
	pr_debug("End get slice init param\n");

	return ret;
}

#if 0
static void sprd_isp_cap_hblank_cfg(void *handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_path_desc *cap_path = NULL;
	struct isp_coeff *coeff_info = NULL;
	uint32_t zoom = 1;
	uint32_t hblank0 = 2500;
	uint32_t hblank1 = 2500;
	uint32_t factor = 200;
	uint32_t width = 0;
	uint32_t height = 0;

	if (!handle) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	dev = (struct isp_pipe_dev *)handle;

	cap_path = &dev->module_info.isp_path[ISP_SCL_CAP];
	coeff_info = &cap_path->coeff_latest;

	if (!cap_path) {
		pr_err("fail to get valid cap_path\n");
		return;
	}

	width = cap_path->coeff_latest.in_rect.w;
	height = cap_path->coeff_latest.in_rect.h;

	if (coeff_info->in_rect.w == 0 || dev->is_raw_capture == 1)
		return;
	zoom = coeff_info->out_size.w / coeff_info->in_rect.w;

	if (coeff_info->out_size.w % coeff_info->in_rect.w != 0)
		zoom += 1;

	hblank1 = (width + hblank0) * (height + factor) /
		(height / zoom + factor) - (width / zoom);

	ISP_REG_WR(dev->com_idx, ISP_FETCH_LINE_DLY_CTRL, hblank1);

	pr_debug("out:w:%d h:%d, in:w:%d h:%d, hblank1:0x%x, zoom:%d\n",
		coeff_info->out_size.w, coeff_info->out_size.h,
		width, height, hblank1, zoom);
}
#endif

static int sprd_fmcu_pre_proc(struct isp_pipe_dev *dev)
{
	uint32_t fmcu_num = 0;
	enum isp_id idx = ISP_ID_0;
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	enum isp_scene_id scene_id = ISP_SCENE_CAP;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct slice_param_in slice_in = {0};
	struct isp_module *module;

	fmcu_slice = &dev->fmcu_slice;
	module = &dev->module_info;
	idx = ISP_GET_ISP_ID(dev->com_idx);
	scene_id = ISP_GET_SCENE_ID(module->com_idx);

	pr_debug("in\n");

	ret = isp_3dnr_cap_frame_proc(dev, &dev->capture_frame);
	if (ret) {
		ret = ISP_RTN_PARA_ERR;
		pr_info("fail to cfg 3dnr cap\n");
		goto exit;
	}

	ret = get_slice_init_param(&slice_in, dev, scene_id);
	if (ret) {
		ret = ISP_RTN_PARA_ERR;
		pr_info("fail to get slice init param and need new frame\n");
		goto exit;
	}

	/*sprd_isp_cap_hblank_cfg(dev);*/

	ret = isp_fmcu_slice_cfg(fmcu_slice->slice_handle,
		&slice_in, &fmcu_num, scene_id);
	if (ret) {
		pr_err("fail to cfg fmcu slice\n");
		goto exit;
	}
	fmcu_slice->fmcu_num = fmcu_num;

#ifdef ISP_DRV_DEBUG
	{
		unsigned int i = 0;
		unsigned long addr = 0;

		addr = (unsigned long)fmcu_slice->buf_info.kaddr[0];

		pr_info("fmcu slice cmd num %d\n", fmcu_slice->fmcu_num);
		for (i = 0; i <= fmcu_slice->fmcu_num; i += 2) {
			pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
				addr,
				REG_RD(addr),
				REG_RD(addr + 4),
				REG_RD(addr + 8),
				REG_RD(addr + 12));
			addr += 16;
		}
	}
#endif
	ISP_HREG_WR(idx, ISP_FMCU_DDR_ADDR, fmcu_slice->buf_info.iova[0]);
	ISP_HREG_MWR(idx, ISP_FMCU_CTRL, 0xFFFF0000,
		fmcu_slice->fmcu_num << 16);

	pr_info("end\n");
exit:
	return ret;
}

int sprd_isp_start_fmcu(void *handle, struct sprd_img_capture_param param,
			uint32_t is_irq, enum isp_path_index path_index)
{
	unsigned long flag = 0;
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	enum isp_id id = ISP_ID_0;
	enum dcam_id idx = DCAM_ID_0;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	uint32_t cap_flag = param.type;

	if (!handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	id = ISP_GET_ISP_ID(dev->com_idx);
	idx = ISP_GET_ISP_ID(dev->com_idx);

	if (!is_irq)
		spin_lock_irqsave(&isp_mod_lock, flag);

	fmcu_slice = &dev->fmcu_slice;
	module = &dev->module_info;
	fmcu_slice->capture_state = ISP_ST_START;
	if (s_isp_group.dual_cam
		&& s_isp_group.dual_fullpath_stop == 0)
		s_isp_group.capture_param = param;

	if (!is_irq)
		dev->cap_flag = cap_flag;

	if (path_index == ISP_PATH_IDX_CAP) {
		ISP_SET_SCENE_ID(module->com_idx, ISP_SCENE_CAP);
		pr_info("isp%d busy %d, cap_flag %d\n", id,
			dev->isp_busy, cap_flag);
		if (module->full_zsl_queue.valid_cnt == 0
			|| cap_flag == DCAM_CAPTURE_START_WITH_FLASH
			|| cap_flag == DCAM_CAPTURE_START_HDR
			|| cap_flag == DCAM_CAPTURE_START_3DNR) {
			pr_info("wait next zsl frame\n");
			dev->clr_queue = 1;
			dev->cap_cur_cnt = 0;
			dev->dcam_full_path_stop = 0;
			ret = ISP_RTN_SUCCESS;
			goto exit;
		}
	}

	dev->isp_offline_state = ISP_ST_STOP;
	if (s_isp_group.dual_cam) {
		if (s_isp_group.dual_fullpath_stop == 0) {
			s_isp_group.dual_fullpath_stop = 1;
			sprd_dcam_path_pause(DCAM_ID_0, CAMERA_FULL_PATH);
			sprd_dcam_path_pause(DCAM_ID_1, CAMERA_FULL_PATH);
		}
	} else {
		if (!dev->dcam_full_path_stop) {
			dev->dcam_full_path_stop = 1;
			sprd_dcam_path_pause(idx, CAMERA_FULL_PATH);
		}
	}
	dev->offline_proc_cap = 1;
	complete(&dev->offline_thread_com);

exit:
	if (!is_irq)
		spin_unlock_irqrestore(&isp_mod_lock, flag);

	return ret;
}

static void isp_common_cfg(uint32_t com_idx)
{
	enum isp_id id = 0;

	id = ISP_GET_ISP_ID(com_idx);

	ISP_HREG_MWR(id, ISP_COMMON_SCL_PATH_SEL,
		(BIT_6 | BIT_7), 0x3 << 6);
	ISP_HREG_WR(id, ISP_COMMON_GCLK_CTRL_2,
		0xFFFF0000);
	ISP_HREG_WR(id, ISP_COMMON_GCLK_CTRL_3,
		0x7F00);
	ISP_HREG_MWR(id, ISP_AXI_ISOLATION, BIT_0, 0);
	ISP_HREG_MWR(id, ISP_ARBITER_ENDIAN_CH0, BIT_0, 0);
	ISP_HREG_WR(id, ISP_ARBITER_CHK_SUM_CLR, 0xF10);
	ISP_HREG_WR(id, ISP_ARBITER_CHK_SUM0, 0x0);

	ISP_REG_MWR(com_idx, ISP_PSTRZ_PARAM, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_HIST_CFG_READY, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_HIST2_PARAM, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_HIST2_CFG_RDY, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_HUA_PARAM, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(com_idx, ISP_YRANDOM_PARAM1, BIT_0, 1);

	ISP_REG_WR(com_idx, ISP_YDELAY_STEP, 0x144);
	ISP_REG_WR(com_idx, ISP_SCALER_PRE_CAP_BASE
		+ ISP_SCALER_HBLANK, 0x4040);
	ISP_REG_WR(com_idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_RES, 0xFF);
	ISP_REG_WR(com_idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_DEBUG, 1);
	ISP_REG_MWR(com_idx, ISP_STORE_BASE + ISP_STORE_PARAM, BIT_0, 1);

	pr_debug("end\n");
}

static int sprd_isp_sel_cap_frame(void *handle)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct camera_frame frame, frame_tmp;
	uint32_t frame_cnt = 0;
	uint32_t i = 0, j = -1;
	unsigned long diff = 0, min = 0xffffffff;
	uint64_t cap_timestamp = 0;

	pr_debug("in.\n");
	if (!handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)handle;
	module = &dev->module_info;

	memset((void *)&frame, 0x00, sizeof(frame));
	if (s_isp_group.dual_cam) {
		cap_timestamp = (uint64_t)s_isp_group.capture_param.timestamp;
		frame_cnt = sprd_cam_frm_queue_cur_nodes(&module
			->full_zsl_queue);
		for (i = 0; i < frame_cnt; i++) {
			memset((void *)&frame_tmp, 0x00, sizeof(frame_tmp));
			ret = sprd_cam_frm_dequeue(&module->full_zsl_queue,
				&frame_tmp);
			if (ret) {
				ret = -1;
				pr_err("fail to dequeue capture frame\n");
				goto exit;
			}
			if (frame_tmp.timestamp >= cap_timestamp)
				diff = frame_tmp.timestamp - cap_timestamp;
			else
				diff = cap_timestamp - frame_tmp.timestamp;
			pr_info("frame %d %ld %ld\n", i, diff,
					(unsigned long)frame_tmp.timestamp);
			if (diff < min) {
				if (j != -1)
					ret = sprd_cam_frm_enqueue(&module
						->full_zsl_queue, &frame);
				min = diff;
				j = i;
				memcpy(&frame, &frame_tmp,
					sizeof(struct camera_frame));
			} else
				ret = sprd_cam_frm_enqueue(&module
					->full_zsl_queue, &frame_tmp);
		}
		pr_info("done %d %ld %ld\n", j, min,
			(unsigned long)cap_timestamp);
	} else {
		ret = sprd_cam_frm_dequeue(&module->full_zsl_queue, &frame);
		if (ret) {
			ret = -ENOENT;
			pr_info("full_zsl_queue have no valid frame\n");
			goto exit;
		}
	}

	frame.buf_info.dev = &s_isp_pdev->dev;
	ret = cam_buf_map_addr(&frame.buf_info);
	if (ret) {
		ret = -1;
		pr_err("fail to map full_path buf addr\n");
		goto exit;
	}

	memcpy(&dev->capture_frame, &frame,
		sizeof(struct camera_frame));

	pr_debug("done");
exit:
	return ret;
}

int sprd_isp_offline_proc(void *handle, uint32_t is_irq)
{
	uint32_t idx = 0, addr = 0, work_mode = 0;
	unsigned long flag = 0;
	enum isp_id id = ISP_ID_0;
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	enum isp_scene_id scene_id = ISP_SCENE_PRE;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_cfg_ctx_desc *cfg_ctx = NULL;
	struct camera_frame frame;
	enum cfg_buf_id buf_id;
	uint32_t cur_status = 0;
	uint32_t cap_state_clr = 0;

	pr_debug("in.\n");

	if (!handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	memset((void *)&frame, 0x00, sizeof(frame));

	dev = (struct isp_pipe_dev *)handle;
	mutex_lock(&dev->offline_thread_mutex);
	if (dev->isp_offline_thread_flag == 1) {
		pr_info("isp_offline_thread_flag %d.\n",
			dev->isp_offline_thread_flag);
		goto exit;
	}

	fmcu_slice = &dev->fmcu_slice;
	module = &dev->module_info;
	cfg_ctx = &module->isp_cfg_contex;
	idx = dev->com_idx;
	id = ISP_GET_ISP_ID(idx);
	work_mode = ISP_GET_MODE_ID(idx);

	/*preview frame proc*/
	if (module->isp_path[ISP_SCL_PRE].valid ||
	    module->isp_path[ISP_SCL_VID].valid) {
		if (dev->offline_proc_cap == 1)
			goto capture_proc;
		if (sprd_cam_frm_queue_cur_nodes(&module->bin_frm_queue) >= 1) {
			ret = sprd_cam_buf_queue_read(&module->bin_buf_queue,
						      &frame);
			if (!ret)
				dcam_set_frame_addr(dev->isp_handle_addr,
						    CAMERA_BIN_PATH, &frame);
			goto capture_proc;
		}

		if (sprd_cam_buf_queue_read(&module->bin_buf_queue, &frame)) {
			pr_debug("fail to dequeue preview frame\n");
			goto capture_proc;
		}

		scene_id = ISP_SCENE_PRE;
		cfg_ctx->cur_scene_id = scene_id;
		ISP_SET_SCENE_ID(idx, scene_id);
		module->com_idx = idx;

		if (frame.buf_info.dev == NULL)
			pr_info("DCAM%d next dev NULL %p\n",
				idx, frame.buf_info.dev);

		frame.buf_info.dev = &s_isp_pdev->dev;
		ret = cam_buf_map_addr(&frame.buf_info);
		if (ret) {
			pr_err("fail to map dcam bin_path addr\n");
			goto capture_proc;
		}

		if (module->isp_path[ISP_SCL_PRE].valid
			&& module->isp_path[ISP_SCL_VID].valid) {
			if (module->isp_path[ISP_SCL_PRE].buf_cnt == 0)
				atomic_set(&frame.usr_cnt, 2);
			else
				atomic_set(&frame.usr_cnt, 1);
			module->isp_path[ISP_SCL_PRE].buf_cnt++;
			if (module->isp_path[ISP_SCL_PRE].buf_cnt >
				module->isp_path[ISP_SCL_PRE].frm_deci)
				module->isp_path[ISP_SCL_PRE].buf_cnt = 0;
		} else
			atomic_set(&frame.usr_cnt, 1);

		if (sprd_cam_frm_enqueue(&module->bin_frm_queue, &frame)) {
			pr_err("fail to enqueue preview frame\n");
			goto capture_proc;
		}

		addr = frame.buf_info.iova[1] + frame.yaddr;
		ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_ADDR, addr);

		ret = sprd_isp_start_path(handle, ISP_PATH_IDX_PRE
			| ISP_PATH_IDX_VID, &frame);

		if (work_mode == ISP_CFG_MODE) {
			ret = isp_update_cfg_buf(cfg_ctx);
			ret = isp_config_cfg_block(cfg_ctx);
			pr_debug("update CFG hw done\n");
		} else
			ISP_HREG_MWR(id, ISP_CFG_PAMATER, BIT_0, 1);
		if (work_mode == ISP_CFG_MODE) {
			/*isp start with CFG mode, through CFG block*/
			pr_debug("start isp%d in CFG mode, scene_idx 0x%x\n",
					id, scene_id);
			isp_cfg_start_isp(cfg_ctx);
			buf_id = cfg_ctx->cfg_buf[scene_id].cur_buf_id;
			 /*buf is free*/
			cfg_ctx->cfg_buf[scene_id].cmd_buf[buf_id].flag = 0;
		} else {
			/*isp start with AP mode, through FETCH block*/
			pr_debug("start isp%d in AP mode, scene_idx 0x%x\n",
					id, scene_id);
			ISP_REG_WR(idx, ISP_FETCH_START, 1);
		}
	}

capture_proc:
	/*capture frame proc*/
	spin_lock_irqsave(&isp_mod_lock, flag);
	cur_status = s_isp_group.dual_cap_sts;
	dev->offline_proc_cap = 0;
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	if (s_isp_group.dual_cam && s_isp_group.dual_cap_cnt == 0 &&
		fmcu_slice->capture_state == ISP_ST_START && cur_status) {
		pr_info("wait dual first frame done.\n");
		s_isp_group.first_need_wait = 1;
	}
	if (fmcu_slice->capture_state == ISP_ST_START && cur_status == 0) {
		spin_lock_irqsave(&isp_mod_lock, flag);
		s_isp_group.dual_cap_sts = 1;
		dev->is_wait_fmcu = 1;
		spin_unlock_irqrestore(&isp_mod_lock, flag);

		ret = sprd_isp_sel_cap_frame(handle);
		if (ret) {
			if (ret == -ENOENT) {
				cap_state_clr = 1;
				ret = 0;
			} else
				pr_info("fail to sprd_isp_sel_cap_frame\n");
			goto cap_exit;
		}

		scene_id = ISP_SCENE_CAP;
		cfg_ctx->cur_scene_id = scene_id;
		ISP_SET_SCENE_ID(idx, scene_id);
		module->com_idx = idx;

		ret = sprd_isp_start_path(dev, ISP_PATH_IDX_CAP, NULL);
		if (ret) {
			ret = ISP_RTN_PARA_ERR;
			pr_info("fail to sprd_isp_start_path\n");
			goto cap_exit;
		}

		ret = sprd_fmcu_pre_proc(dev);
		if (ret) {
			pr_info("fail to get capture frame\n");
			goto cap_exit;
		}

		if (work_mode == ISP_CFG_MODE) {
			ret = isp_update_cfg_buf(cfg_ctx);
			ret = isp_config_cfg_block(cfg_ctx);
			pr_debug("config CFG hw done\n");
		} else
			ISP_HREG_MWR(id, ISP_CFG_PAMATER, BIT_0, 1);

		if (!module->isp_path[ISP_SCL_VID].valid)
			ISP_HREG_WR(id, ISP_FMCU_START, 1);
		pr_info("isp%d fmcu start\n", id);

		if (work_mode == ISP_CFG_MODE) {
			buf_id = cfg_ctx->cfg_buf[scene_id].cur_buf_id;
			/*buf is free*/
			cfg_ctx->cfg_buf[scene_id].cmd_buf[buf_id].flag = 0;
		}

cap_exit:
		if (ret || cap_state_clr) {
			spin_lock_irqsave(&isp_mod_lock, flag);
			dev->is_wait_fmcu = 0;
			s_isp_group.dual_cap_sts = 0;
			spin_unlock_irqrestore(&isp_mod_lock, flag);
		}
	}

exit:
	mutex_unlock(&dev->offline_thread_mutex);
	pr_debug("done.\n");
	return ret;
}

int sprd_isp_fmcu_slice_stop(void *handle)
{
	unsigned long flag = 0;
	enum isp_id id = ISP_ID_0;
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = NULL;

	if (!handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	fmcu_slice = &dev->fmcu_slice;
	id = ISP_GET_ISP_ID(dev->com_idx);

	spin_lock_irqsave(&isp_mod_lock, flag);
	fmcu_slice->capture_state = ISP_ST_STOP;
	dev->clr_queue = 0;
	dev->cap_cur_cnt = 0;
	dev->cap_flag = DCAM_CAPTURE_STOP;
	if (!s_isp_group.dual_cam)
		s_isp_group.dual_cap_sts = 0;
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	return ret;
}

static uint32_t isp_get_path_deci_factor(uint32_t src_size,
				uint32_t dst_size)
{
	uint32_t factor = 0;

	if (0 == src_size || 0 == dst_size)
		return factor;

	/* factor: 0 - 1/2, 1 - 1/4, 2 - 1/8, 3 - 1/16 */
	for (factor = 0; factor < CAMERA_PATH_DECI_FAC_MAX; factor++) {
		if (src_size < (uint32_t) (dst_size * (1 << (factor + 1))))
			break;
	}

	return factor;
}

static int isp_calc_sc_size(struct isp_path_desc *path)
{

	uint32_t tmp_dstsize = 0, align_size = 0;
	uint32_t d_max = CAMERA_SC_COEFF_DOWN_MAX;
	uint32_t u_max = CAMERA_SC_COEFF_UP_MAX;
	uint32_t f_max = CAMERA_PATH_DECI_FAC_MAX;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_trim_info *in_trim;
	struct camera_size *out_size;

	if (path == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	in_trim = &path->trim0_info;
	out_size = &path->dst;
	if (in_trim->size_x > (out_size->w * d_max * (1 << f_max)) ||
		in_trim->size_y > (out_size->h * d_max * (1 << f_max)) ||
		in_trim->size_x * u_max < out_size->w ||
		in_trim->size_y * u_max < out_size->h) {
		rtn = ISP_RTN_PATH_SC_ERR;
	} else {
		path->scaler_info.scaler_factor_in = in_trim->size_x;
		path->scaler_info.scaler_ver_factor_in = in_trim->size_y;
		if (in_trim->size_x > out_size->w * d_max) {
			tmp_dstsize =
				out_size->w * d_max;
			path->deci_info.deci_x =
				isp_get_path_deci_factor(in_trim->size_x,
							tmp_dstsize);
			path->deci_info.deci_x_eb = 1;
			align_size = (1 << (path->deci_info.deci_x + 1)) *
				ISP_PIXEL_ALIGN_WIDTH;
			in_trim->size_x = (in_trim->size_x)
				& ~(align_size - 1);
			in_trim->start_x = (in_trim->start_x)
				& ~(align_size - 1);
			path->scaler_info.scaler_factor_in =
				in_trim->size_x >> (path->deci_info.deci_x + 1);
		} else {
			path->deci_info.deci_x = 0;
			path->deci_info.deci_x_eb = 0;
		}

		if (in_trim->size_y > out_size->h * d_max) {
			tmp_dstsize = out_size->h * d_max;
			path->deci_info.deci_y =
				isp_get_path_deci_factor(in_trim->size_y,
							tmp_dstsize);
			path->deci_info.deci_y_eb = 1;
			align_size = (1 << (path->deci_info.deci_y + 1)) *
				ISP_PIXEL_ALIGN_HEIGHT;
			in_trim->size_y = (in_trim->size_y)
				& ~(align_size - 1);
			in_trim->start_y = (in_trim->start_y)
				& ~(align_size - 1);
			path->scaler_info.scaler_ver_factor_in =
				in_trim->size_y >> (path->deci_info.deci_y + 1);
		} else {
			path->deci_info.deci_y = 0;
			path->deci_info.deci_y_eb = 0;
		}

		path->scaler_info.scaler_ver_factor_out = path->dst.h;
		path->scaler_info.scaler_factor_out = path->dst.w;
	}

	return -rtn;
}

static int isp_set_sc_coeff(struct isp_module *module,
			enum isp_path_index path_index,
			struct isp_path_desc *path,
			struct isp_coeff *coeff)
{
	uint32_t scale2yuv420 = 0;
	uint32_t *tmp_buf = NULL;
	uint32_t *h_coeff = NULL;
	uint32_t *v_coeff = NULL;
	uint32_t *v_chroma_coeff = NULL;
	unsigned char y_tap = 0;
	unsigned char uv_tap = 0;
	enum isp_scl_id scl_path_id = ISP_SCL_PRE;

	if (!module || !path || !coeff) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	scl_path_id = isp_get_scl_index(path_index);

	if (path->output_format == DCAM_YUV420)
		scale2yuv420 = 1;

	tmp_buf = coeff->coeff_buf;
	if (tmp_buf == NULL)
		return -EFAULT;

	h_coeff = tmp_buf;
	v_coeff = tmp_buf + (ISP_SC_COEFF_COEF_SIZE / 4);
	v_chroma_coeff = v_coeff + (ISP_SC_COEFF_COEF_SIZE / 4);

	wait_for_completion_interruptible(
			&module->scale_coeff_mem_com);

	if (!(cam_gen_scale_coeff((short)path->scaler_info.scaler_factor_in,
				(short)path->scaler_info.scaler_ver_factor_in,
				(short)path->dst.w,
				(short)path->dst.h,
				h_coeff,
				v_coeff,
				v_chroma_coeff,
				scale2yuv420,
				&y_tap,
				&uv_tap,
				tmp_buf + (ISP_SC_COEFF_COEF_SIZE * 3 / 4),
				ISP_SC_COEFF_TMP_SIZE))) {
		pr_err("fail to call cam_gen_scale_coeff\n");
		complete(&module->scale_coeff_mem_com);
		return -DCAM_RTN_PATH_GEN_COEFF_ERR;
	}

	path->scaler_info.scaler_y_ver_tap = y_tap;
	path->scaler_info.scaler_uv_ver_tap = uv_tap;

	pr_debug("Scaler y_tap %d, uv_tap %d\n", y_tap, uv_tap);

	complete(&module->scale_coeff_mem_com);
	return ISP_RTN_SUCCESS;
}

static int isp_path_scaler(struct isp_module *module,
		enum isp_path_index path_index,
		struct isp_path_desc *path)
{
	uint32_t idx = 0;
	unsigned long cfg_reg = 0;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_coeff *coeff = NULL;

	if (!module || !path) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	idx = module->com_idx;

	if (path_index == ISP_PATH_IDX_PRE)
		cfg_reg = ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG;
	else if (path_index == ISP_PATH_IDX_VID)
		cfg_reg = ISP_SCALER_VID_BASE + ISP_SCALER_CFG;
	else if (path_index == ISP_PATH_IDX_CAP)
		cfg_reg = ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG;
	else
		return -EFAULT;

	if (path->output_format == DCAM_RAWRGB ||
		path->output_format == DCAM_JPEG) {
		CAM_TRACE("out format is %d, no need scaler\n",
			path->output_format);
		return ISP_RTN_SUCCESS;
	}

	rtn = isp_calc_sc_size(path);
	if (rtn)
		return rtn;

	if (path->scaler_info.scaler_factor_in ==
		path->scaler_info.scaler_factor_out &&
		path->scaler_info.scaler_ver_factor_in ==
		path->scaler_info.scaler_ver_factor_out &&
		path->output_format == DCAM_YUV422 &&
		path_index != ISP_PATH_IDX_0) {
		path->scaler_bypass = 1;
		ISP_REG_MWR(idx, cfg_reg, BIT_20, 1 << 20);
	} else {
		path->scaler_bypass = 0;
		ISP_REG_MWR(idx, cfg_reg, BIT_20, 0 << 20);
		coeff = &path->coeff_latest;
		rtn = isp_set_sc_coeff(module, path_index, path, coeff);
	}

	return rtn;
}

int isp_raw_cap_proc(void *isp_handle,
	struct isp_raw_proc_info *raw_cap)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_module *module = NULL;
	struct isp_path_desc *cap = NULL;
	struct camera_addr frm_addr = {0};
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)isp_handle;

	if (!dev || !raw_cap) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	module = &dev->module_info;
	cap = &module->isp_path[ISP_SCL_CAP];
	fmcu_slice = &dev->fmcu_slice;

	dev->is_raw_capture = 1;
	fmcu_slice->capture_state = ISP_ST_START;
	cap->coeff_latest.param.in_size.w = raw_cap->in_size.width;
	cap->coeff_latest.param.in_size.h = raw_cap->in_size.height;
	cap->coeff_latest.param.out_size.w = raw_cap->out_size.width;
	cap->coeff_latest.param.out_size.h = raw_cap->out_size.height;
	cap->coeff_latest.param.in_rect.x = 0;
	cap->coeff_latest.param.in_rect.y = 0;
	cap->coeff_latest.param.in_rect.w = raw_cap->in_size.width;
	cap->coeff_latest.param.in_rect.h = raw_cap->in_size.height;
	cap->output_format = DCAM_YUV420;
	cap->valid = 1;

	frm_addr.yaddr = raw_cap->img_offset.chn0;
	frm_addr.uaddr = raw_cap->img_offset.chn1;
	frm_addr.vaddr = raw_cap->img_offset.chn2;
	frm_addr.yaddr_vir = raw_cap->img_vir.chn0;
	frm_addr.uaddr_vir = raw_cap->img_vir.chn1;
	frm_addr.vaddr_vir = raw_cap->img_vir.chn2;
	frm_addr.mfd_y = raw_cap->img_fd;

	ret = set_isp_path_cfg((void *)dev, ISP_PATH_IDX_CAP,
		ISP_PATH_OUTPUT_ADDR, &frm_addr);

	ret = sprd_isp_start((void *)dev);
	pr_info("end\n");
exit:
	return ret;
}

int sprd_isp_update_zoom_param(void *isp_handle,
			enum isp_path_index path_index,
			struct camera_size *in_size,
			struct camera_rect *in_rect,
			struct camera_size *out_size)
{
	enum isp_id id = ISP_ID_0;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_zoom_param zoom_param;

	if (isp_handle == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	id = ISP_GET_ISP_ID(dev->com_idx);

	pr_debug("ISP%d: update path\n", id);

	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];

	zoom_param.in_size.w = in_size->w;
	zoom_param.in_size.h = in_size->h;
	zoom_param.in_rect.x = in_rect->x;
	zoom_param.in_rect.y = in_rect->y;
	zoom_param.in_rect.w = in_rect->w;
	zoom_param.in_rect.h = in_rect->h;
	zoom_param.out_size.w = out_size->w;
	zoom_param.out_size.h = out_size->h;

	if (ISP_PATH_IDX_PRE & path_index) {
		rtn = sprd_cam_buf_queue_write(&pre->coeff_queue, &zoom_param);
		if (rtn) {
			pr_err("fail to write isp coeff node\n");
			return -rtn;
		}
	}

	if (ISP_PATH_IDX_VID & path_index) {
		rtn = sprd_cam_buf_queue_write(&vid->coeff_queue, &zoom_param);
		if (rtn) {
			pr_err("fail to write isp coeff node\n");
			return -rtn;
		}
	}
	if (ISP_PATH_IDX_CAP & path_index) {
		rtn = sprd_cam_buf_queue_write(&cap->coeff_queue, &zoom_param);
		if (rtn) {
			pr_err("fail to write isp coeff node\n");
			return -rtn;
		}
		if (sprd_cam_buf_queue_cur_nodes(&cap->coeff_queue)
			> 1) {
			rtn = sprd_cam_buf_queue_read(&cap->coeff_queue,
						      &zoom_param);
			if (rtn) {
				pr_err("fail to read isp coeff node\n");
				return -rtn;
			}
		}
	}

	pr_debug("update path done\n");
	return -rtn;
}

static int isp_start_path_common(struct isp_module *module,
	struct isp_path_desc *cur_path,
	enum isp_path_index cur_path_idx,
	enum isp_scl_id cur_scl_idx,
	struct camera_frame *dcam_frame)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_zoom_param zoom_param;

	rtn = sprd_cam_buf_queue_read(&cur_path->coeff_queue, &zoom_param);
	if (!rtn) {
		memcpy(&cur_path->coeff_latest.param,
				&zoom_param, sizeof(struct isp_zoom_param));
		isp_path_cfg(cur_path);
	}
	pr_debug("path_idx %d in_size %d %d in_rect %d %d %d %d out_size %d %d\n",
		cur_path_idx,
		zoom_param.in_size.w,
		zoom_param.in_size.h,
		zoom_param.in_rect.x,
		zoom_param.in_rect.y,
		zoom_param.in_rect.w,
		zoom_param.in_rect.h,
		zoom_param.out_size.w,
		zoom_param.out_size.h);
	rtn = isp_path_scaler(module, cur_path_idx, cur_path);
	if (rtn) {
		pr_err("fail to set pre path scaler\n");
		return rtn;
	}

	isp_path_set(module, cur_path, cur_path_idx);

	if (cur_scl_idx != ISP_SCL_CAP) {
		if (cur_path->buf_cnt == 0)
			rtn = isp_path_set_next_frm(module, cur_path_idx,
						    NULL, dcam_frame);

		if (rtn) {
			pr_err("fail to set next frame\n");
			return rtn;
		}
	}
	return rtn;
}

int sprd_isp_start_path(void *isp_handle,
	enum isp_path_index path_index, struct camera_frame *frame)
{
	enum isp_id id = ISP_ID_0;
	enum isp_scl_id cur_scl_idx = ISP_SCL_PRE;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	enum isp_path_index cur_path_idx = ISP_PATH_IDX_0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_path_desc *cur_path = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	id = ISP_GET_ISP_ID(dev->com_idx);
	module = &dev->module_info;
	/*module->com_idx = dev->com_idx;*/
	fmcu_slice = &dev->fmcu_slice;
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];

	isp_set_status(isp_handle);
	pr_debug("cap_valid %d, path_index %d\n",
		module->isp_path[ISP_SCL_CAP].valid, path_index);

	if ((ISP_PATH_IDX_PRE & path_index) &&
		module->isp_path[ISP_SCL_PRE].valid) {
		cur_path = pre;
		cur_path_idx = ISP_PATH_IDX_PRE;
		cur_scl_idx = ISP_SCL_PRE;
		isp_start_path_common(module, cur_path,
			cur_path_idx, cur_scl_idx, frame);

		ISP_HREG_MWR(id,
		ISP_COMMON_SCL_PATH_SEL, (BIT_0 | BIT_1),
		0);
	}

	if ((ISP_PATH_IDX_VID & path_index) &&
		module->isp_path[ISP_SCL_VID].valid) {
		cur_path = vid;
		cur_path_idx = ISP_PATH_IDX_VID;
		cur_scl_idx = ISP_SCL_VID;
		isp_start_path_common(module, cur_path,
			cur_path_idx, cur_scl_idx, frame);

		ISP_HREG_MWR(id,
		ISP_COMMON_SCL_PATH_SEL, (BIT_2 | BIT_3),
		0 << 2);
	}

	if ((ISP_PATH_IDX_CAP & path_index) &&
		module->isp_path[ISP_SCL_CAP].valid) {
		cur_path = cap;
		cur_path_idx = ISP_PATH_IDX_CAP;
		cur_scl_idx = ISP_SCL_CAP;
		dev->dcam_full_path_stop = 0;
		isp_start_path_common(module, cur_path,
			cur_path_idx, cur_scl_idx, frame);

		ISP_HREG_MWR(id,
		ISP_COMMON_SCL_PATH_SEL, (BIT_0 | BIT_1),
		0);
	}

	/*if 3dnr enable && preview mode*/
	if ((pre->nr3_param.need_3dnr && frame != NULL)
		|| (vid->nr3_param.need_3dnr && frame != NULL)) {
		isp_3dnr_conversion_mv(&pre->nr3_param, frame);
		isp_3dnr_get_param(&pre->nr3_param, frame);
		isp_3dnr_config_param(dev->com_idx,
			ISP_OPERATE_PRE, &pre->nr3_param);
	} else
		isp_3dnr_default_param(dev->com_idx);

	if  (!(((ISP_PATH_IDX_PRE & path_index) &&
		module->isp_path[ISP_SCL_PRE].valid) ||
		 ((ISP_PATH_IDX_CAP & path_index) &&
		module->isp_path[ISP_SCL_CAP].valid) ||
		 ((ISP_PATH_IDX_VID & path_index) &&
		module->isp_path[ISP_SCL_VID].valid))) {
		pr_err("fail to get valid path index\n");
		return -(rtn);
	}
	isp_common_cfg(module->com_idx);
	sprd_irq_mask_enable(module->com_idx);

	pr_debug("isp start done.\n");
	return rtn;
}

int sprd_isp_start(void *isp_handle)
{
	uint32_t i = 0, work_mode = 0, idx = 0;
	uint32_t size = 0;
	uint32_t wqos_val = 0;
	uint32_t rqos_val = 0;
	uint32_t r_ostd_val = 0;
	enum chip_id id = SHARKL3;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_cfg_ctx_desc *cfg_ctx = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_nr3_param *nr3_dev = NULL;
	struct cam_buf_info *nr3_buf = NULL;
	void *temp_buf_vaddr = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	fmcu_slice = &dev->fmcu_slice;
	cfg_ctx = &module->isp_cfg_contex;
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	work_mode = ISP_GET_MODE_ID(dev->com_idx);
	idx = ISP_GET_ISP_ID(dev->com_idx);

	isp_k_param = &dev->isp_k_param;
	isp_k_param->isp_status = ISP_START;

	/*fmcu cmdq buf*/
	rtn = cam_buf_alloc_k(&dev->fmcu_slice.buf_info,
		&s_isp_pdev->dev, PAGE_SIZE,
		1, CAM_BUF_KERNEL_TYPE);
	if (rtn)
		pr_err("fail to get fmcu buf\n");

	rtn = cam_buf_map_addr(&fmcu_slice->buf_info);
	if (rtn) {
		pr_err("fail to map fmcu\n");
		return rtn;
	}

	rtn = isp_start_pre_proc(pre, vid, cap);
	if (rtn) {
		pr_err("fail to start isp pre proc\n");
		return -rtn;
	}

	if (work_mode == ISP_CFG_MODE) {
		/*alloc cfg context buffer*/
		rtn = cam_buf_alloc_k(&cfg_ctx->buf_info,
			&s_isp_pdev->dev, ISP_CFG_BUF_SIZE_ALL,
			1, CAM_BUF_KERNEL_TYPE);
		if (rtn) {
			pr_err("fail to get cfg buffer\n");
			return rtn;
		}

		rtn = isp_cfg_ctx_buf_iommu_map(cfg_ctx);
		if (rtn) {
			pr_err("fail to map cmd buffer\n");
			return rtn;
		}

		temp_buf_vaddr = (void *)&cfg_ctx->temp_cfg_buf[0];
		memcpy(cfg_ctx->cfg_buf[0].cmd_buf[0].vir_addr,
			temp_buf_vaddr, ISP_REG_SIZE);
		memcpy(cfg_ctx->cfg_buf[1].cmd_buf[0].vir_addr,
			temp_buf_vaddr + ISP_REG_SIZE, ISP_REG_SIZE);

		rtn = isp_cfg_map_init(cfg_ctx);
		if (rtn) {
			pr_err("fail to init CFG map.\n");
			return rtn;
		}

		rtn = isp_cfg_ctx_buf_init(cfg_ctx, 1);
		if (rtn) {
			pr_err("fail to config CFG.\n");
			return rtn;
		}
	}

	for (i = 0; i < ISP_SCL_MAX ; i++) {
		module->isp_path[i].status = ISP_ST_START;
		module->isp_path[i].frm_cnt = 0;
		module->isp_path[i].buf_cnt = 0;
	}

	dev->isp_offline_state = ISP_ST_START;
	dev->cap_cur_cnt = 0;
	dev->isp_offline_thread_flag = 0;
	/*3dnr memory alloc*/
	if ((pre->valid && pre->nr3_param.need_3dnr)
		|| (vid->valid && vid->nr3_param.need_3dnr)) {
		nr3_dev = (struct isp_nr3_param *)&pre->nr3_param;
		nr3_buf = (struct cam_buf_info *)&nr3_dev->buf_info;
		size = nr3_dev->prev_size.width *
				nr3_dev->prev_size.height * 3/2;
		pr_debug("3DNR prev_vid alloc width=%d, height=%d\n",
			nr3_dev->prev_size.width, nr3_dev->prev_size.height);
		rtn = cam_buf_alloc_k(nr3_buf, &s_isp_pdev->dev,
			size, ISP_NR3_BUFFER_NUM,
			CAM_BUF_KERNEL_TYPE);
		if (rtn) {
			pr_err("fail to 3ndr prev alloc buf\n");
			return rtn;
		}
		rtn = cam_buf_map_addr(nr3_buf);
		if (rtn) {
			pr_err("fail to 3ndr prev map buf\n");
			return rtn;
		}
	}

	if (cap->valid && cap->nr3_param.need_3dnr) {
		nr3_dev = (struct isp_nr3_param *)&cap->nr3_param;
		nr3_buf = (struct cam_buf_info *)&nr3_dev->buf_info;
		size = nr3_dev->sns_max_size.width *
			nr3_dev->sns_max_size.height * 3/2;
		pr_debug("3DNR cap alloc width=%d, height=%d\n",
			nr3_dev->sns_max_size.width,
			nr3_dev->sns_max_size.height);
		rtn = cam_buf_alloc_k(nr3_buf, &s_isp_pdev->dev,
			size, ISP_NR3_BUFFER_NUM,
			CAM_BUF_KERNEL_TYPE);
		if (rtn) {
			pr_err("fail to 3ndr capture alloc buf\n");
			return rtn;
		}
		rtn = cam_buf_map_addr(nr3_buf);
		if (rtn) {
			pr_err("fail to 3ndr capture map buf\n");
			return rtn;
		}
	}

	r_ostd_val = 1 << 8;
	ISP_HREG_MWR(idx, ISP_AXI_ITI2AXIM_CTRL,
			ISP_AXI_ITI2AXIM_ISP_ROSTD_MASK,
			r_ostd_val);

	id = sprd_dcam_get_chip_id();
	if (id == SHARKL3) {
		wqos_val = (0x1 << 13) | (0x0 << 12) |
				(0x4 << 8) | (0x6 << 4) | 0x6;
		rqos_val = (0x0 << 8) | (0x6 << 4) | 0x6;
	} else {
		wqos_val = (0x0 << 13) | (0x0 << 12) |
				(0x4 << 8) | (0xA << 4) | 0xA;
		rqos_val = (0x0 << 8) | (0xA << 4) | 0x8;
	}

	ISP_HREG_MWR(idx, ISP_AXI_ARBITER_WQOS,
					ISP_AXI_ARBITER_WQOS_MASK,
					wqos_val);
	ISP_HREG_MWR(idx, ISP_AXI_ARBITER_RQOS,
					ISP_AXI_ARBITER_RQOS_MASK,
					rqos_val);
	ISP_HREG_MWR(idx, ISP_PMU_EN, BIT_22, (0x1 << 22));

	pr_info("done\n");

	return rtn;
}

static void isp_set_reg_value_default(uint32_t idx)
{
	uint32_t i = 0;

	/*CFA*/
	for (i = 0x3010; i < 0x3058; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*NLM*/
	for (i = 0x2010; i < 0x217C; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*HSV*/
	for (i = 0x3310; i < 0x334C; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*PRECDN*/
	for (i = 0x5010; i < 0x5058; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*YNR*/
	for (i = 0x5110; i < 0x5198; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*CDN*/
	for (i = 0x5610; i < 0x5654; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*EE*/
	for (i = 0x5710; i < 0x5780; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*POSTCDN*/
	for (i = 0x5A10; i < 0x5A80; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*IIRCNR*/
	for (i = 0x5D10; i < 0x5DD4; i += 4)
		ISP_REG_WR(idx, i, 0);

	/*scl_cap*/
	for (i = 0xC010; i < 0xC050; i += 4)
		ISP_REG_WR(idx, i, 0);
	/*scl_vid*/
	for (i = 0xD010; i < 0xC05C; i += 4)
		ISP_REG_WR(idx, i, 0);

}

static int32_t isp_block_buf_alloc(struct isp_k_block *isp_k_param)
{
	int32_t ret = 0;
	uint32_t buf_len = 0;

	buf_len = ISP_FRGB_GAMMA_BUF_SIZE;

	isp_k_param->full_gamma_buf_addr = (unsigned long)vzalloc(buf_len);
	if (isp_k_param->full_gamma_buf_addr == 0) {
		ret = -1;
		pr_err("fail to get valid memory.\n");
	}

	return ret;
}

static int32_t isp_block_buf_free(struct isp_k_block *isp_k_param)
{
	if (isp_k_param->full_gamma_buf_addr != 0x00) {
		vfree((void *)isp_k_param->full_gamma_buf_addr);
		isp_k_param->full_gamma_buf_addr = 0x00;
	}

	return 0;
}

int sprd_isp_buffer_alloc_k(void *isp_handle)
{
	uint32_t work_mode = 0;
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_k_block *isp_k_param = NULL;
	struct isp_cfg_ctx_desc *cfg_ctx = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	isp_k_param = &dev->isp_k_param;
	cfg_ctx = &dev->module_info.isp_cfg_contex;
	work_mode = ISP_GET_MODE_ID(dev->com_idx);


	ret = isp_block_buf_alloc(isp_k_param);
	if (ret != 0)
		isp_block_buf_free(isp_k_param);

	if (work_mode == ISP_CFG_MODE) {
		ret = isp_cfg_ctx_buf_init(cfg_ctx, 0);
		if (ret) {
			pr_err("fail to config CFG.\n");
			return ret;
		}
	}

	return ret;
}

int set_isp_path_cfg(void *isp_handle, enum isp_path_index path_index,
	enum isp_config_param id, void *param)
{
	int dcam_path_id = 0;
	uint32_t frm_deci = 0, format = 0, path_mode = 0, skip_num = 0;
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	enum isp_scl_id path_id;
	struct isp_path_desc *path = NULL;
	struct isp_regular_info *regular_info = NULL;
	struct isp_endian_sel *endian = NULL;
	struct isp_module *module = NULL;
	struct camera_size *size = NULL;
	struct camera_rect *rect = NULL;
	struct camera_addr *p_addr = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_nr3_param *nr3_dev = NULL;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	if (ISP_PATH_IDX_PRE & path_index) {
		path_id = ISP_SCL_PRE;
	} else if (ISP_PATH_IDX_VID & path_index) {
		path_id = ISP_SCL_VID;
	} else if (ISP_PATH_IDX_CAP & path_index) {
		path_id = ISP_SCL_CAP;
	} else {
		pr_err("fail to select path\n");
		return -ISP_RTN_PARA_ERR;
	}

	module = &dev->module_info;
	path = &module->isp_path[path_id];
	nr3_dev = &path->nr3_param;

	switch (id) {
	case ISP_PATH_INPUT_SIZE:
		size = (struct camera_size *)param;

		pr_debug("ISP path%d input size:{%d %d}\n", path_id,
			size->w, size->h);
		if (size->w > ISP_PATH_FRAME_WIDTH_MAX ||
			size->h > ISP_PATH_FRAME_HEIGHT_MAX) {
			rtn = ISP_RTN_PATH_IN_SIZE_ERR;
		} else {
			path->coeff_latest.param.in_size.w = size->w;
			path->coeff_latest.param.in_size.h = size->h;
			nr3_dev->prev_size.width = size->w;
			nr3_dev->prev_size.height = size->h;
		}
		break;

	case ISP_PATH_INPUT_RECT:
		rect = (struct camera_rect *)param;

		pr_debug("ISP path%d trim size:{%d %d} {%d %d}\n", path_id,
			rect->x, rect->y, rect->w, rect->h);
		if (rect->x > ISP_PATH_FRAME_WIDTH_MAX ||
			rect->y > ISP_PATH_FRAME_HEIGHT_MAX ||
			rect->w > ISP_PATH_FRAME_WIDTH_MAX ||
			rect->h > ISP_PATH_FRAME_HEIGHT_MAX) {
			rtn = ISP_RTN_PATH_TRIM_SIZE_ERR;
		} else {
			path->coeff_latest.param.in_rect.x = rect->x;
			path->coeff_latest.param.in_rect.y = rect->y;
			path->coeff_latest.param.in_rect.w = rect->w;
			path->coeff_latest.param.in_rect.h = rect->h;
		}
		break;

	case ISP_PATH_OUTPUT_SIZE:
		size = (struct camera_size *)param;

		pr_debug("ISP path%d output size:{%d %d}\n", path_id,
			size->w, size->h);
		if (size->w > ISP_PATH_FRAME_WIDTH_MAX ||
			size->h > ISP_PATH_FRAME_HEIGHT_MAX) {
			rtn = ISP_RTN_PATH_OUT_SIZE_ERR;
		} else {
			path->coeff_latest.param.out_size.w = size->w;
			path->coeff_latest.param.out_size.h = size->h;
		}
		break;

	case ISP_PATH_OUTPUT_FORMAT:
		format = *(uint32_t *)param;
		if ((format == DCAM_YUV422) || (format == DCAM_YUV420) ||
			(format == DCAM_YVU420) ||
			(format == DCAM_YUV420_3FRAME))
			path->output_format = format;
		else {
			rtn = ISP_RTN_OUT_FMT_ERR;
			path->output_format = DCAM_FTM_MAX;
		}
		break;

	case ISP_PATH_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;
		if (ISP_PATH_IDX_PRE & path_index) {
			dcam_path_id = CAMERA_PRE_PATH;
		} else if (ISP_PATH_IDX_VID & path_index) {
			dcam_path_id = CAMERA_VID_PATH;
		} else if (ISP_PATH_IDX_CAP & path_index) {
			dcam_path_id = CAMERA_CAP_PATH;
		} else {
			pr_err("fail to select path\n");
			return -ISP_RTN_PARA_ERR;
		}

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
			p_addr->uaddr, p_addr->vaddr)
			&& p_addr->mfd_y == 0)
			rtn = ISP_RTN_PATH_ADDR_ERR;
		else {
			struct camera_frame frame;

			memset((void *)&frame, 0x00, sizeof(frame));
			frame.yaddr = p_addr->yaddr;
			frame.uaddr = p_addr->uaddr;
			frame.vaddr = p_addr->vaddr;
			frame.yaddr_vir = p_addr->yaddr_vir;
			frame.uaddr_vir = p_addr->uaddr_vir;
			frame.vaddr_vir = p_addr->vaddr_vir;

			frame.type = dcam_path_id;
			frame.fid = path->frame_base_id;

			frame.buf_info.dev = &s_isp_pdev->dev;
			frame.buf_info.mfd[0] = p_addr->mfd_y;
			frame.buf_info.mfd[1] = p_addr->mfd_u;
			frame.buf_info.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = cam_buf_get_sg_table(&frame.buf_info);
			if (rtn) {
				pr_err("fail to config output addr!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			if (!sprd_cam_buf_queue_write(&path->buf_queue,
				&frame))
				path->output_frame_count++;

			pr_debug("isp%d y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				dev->com_idx,
				p_addr->yaddr, p_addr->uaddr,
				p_addr->vaddr, frame.buf_info.mfd[0],
				frame.buf_info.mfd[1]);
		}
		break;

	case ISP_PATH_OUTPUT_RESERVED_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
			p_addr->uaddr,
			p_addr->vaddr) &&
			p_addr->mfd_y == 0) {
			rtn = ISP_RTN_PATH_ADDR_ERR;
		} else {
			uint32_t output_frame_count = 0;
			struct camera_frame *frame = NULL;

			frame = &module->isp_path[path_id].path_reserved_frame;
			output_frame_count = path->output_frame_count;

			memset((void *)frame, 0x00, sizeof(*frame));
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			frame->buf_info.dev = &s_isp_pdev->dev;
			frame->buf_info.mfd[0] = p_addr->mfd_y;
			frame->buf_info.mfd[1] = p_addr->mfd_u;
			frame->buf_info.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = cam_buf_get_sg_table(&frame->buf_info);
			if (rtn) {
				pr_err("fail to cfg reserved output addr!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}
		}
		break;

	case ISP_PATH_FRM_DECI:
		frm_deci = *(uint32_t *)param;
		if (frm_deci >= DCAM_FRM_DECI_FAC_MAX)
			rtn = ISP_RTN_FRM_DECI_ERR;
		else
			path->frm_deci = frm_deci;
		break;

	case ISP_PATH_MODE:
		path_mode = *(uint32_t *)param;
		path->path_mode = path_mode;
		break;

	case ISP_PATH_ENABLE:
		path->valid = *(uint32_t *)param;
		break;

	case ISP_PATH_SHRINK:
		regular_info = (struct isp_regular_info *)param;
		memcpy(&path->regular_info, regular_info,
		sizeof(struct isp_regular_info));
		break;

	case ISP_PATH_DATA_ENDIAN:
		endian = (struct isp_endian_sel *)param;
		if (endian->y_endian >= DCAM_ENDIAN_MAX) {
			rtn = ISP_RTN_PATH_ENDIAN_ERR;
		} else {
			path->data_endian.y_endian = endian->y_endian;
			if ((ISP_PATH_IDX_PRE |
			ISP_PATH_IDX_CAP |
			ISP_PATH_IDX_VID) & path_index) {
			path->data_endian.uv_endian = endian->uv_endian;
			pr_debug("isp path data endian %d\n",
				path->data_endian.uv_endian);
			}
		}
		break;

	case ISP_PATH_SKIP_NUM:
		skip_num = *(uint32_t *)param;
		if (skip_num >= DCAM_CAP_SKIP_FRM_MAX)
			rtn = ISP_RTN_FRM_DECI_ERR;
		else
			path->skip_num = skip_num;
		break;
	case ISP_NR3_ENABLE:
		nr3_dev->need_3dnr = *(uint32_t *)param;
		break;
	case ISP_SNS_MAX_SIZE:
		size = (struct camera_size *)param;
		nr3_dev->sns_max_size.width = size->w;
		nr3_dev->sns_max_size.height = size->h;
		break;
	case ISP_NR3_ME_CONV_SIZE:
		size = (struct camera_size *)param;
		nr3_dev->me_conv.input_width = size->w;
		nr3_dev->me_conv.input_height = size->h;
		break;
	case ISP_DUAL_CAM_EN:
		s_isp_group.dual_cam = *(uint32_t *)param;
		break;
	default:
		pr_err("fail to get valid isp cfg id %d\n", id);
		break;
	}

	return -rtn;
}

int isp_get_status(void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	uint32_t cur_status = 0;
	unsigned long flag = 0;
	enum isp_id id = 0;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	id = ISP_GET_ISP_ID(dev->com_idx);

	spin_lock_irqsave(&isp_mod_lock, flag);
	cur_status = dev->isp_busy;
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	return cur_status;
}

int isp_set_status(void *isp_handle)
{
	unsigned long flag = 0;
	struct isp_pipe_dev *dev = NULL;
	enum isp_id id = 0;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	id = ISP_GET_ISP_ID(dev->com_idx);

	spin_lock_irqsave(&isp_mod_lock, flag);
	dev->isp_busy = 1;
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	return 0;
}

static int isp_enable_clk(void)
{
	int ret = 0;
	uint32_t flag = 0;

	/*set isp clock to max value*/
	ret = clk_set_parent(isp_clk, isp_clk_parent);
	if (ret) {
		pr_info("fail to set isp_clk_parent.\n");
		clk_set_parent(isp_clk, isp_clk_default);
		clk_disable_unprepare(isp_clk);
		goto exit;
	}

	ret = clk_prepare_enable(isp_clk);
	if (ret) {
		pr_info("fail to enable isp_clk.\n");
		clk_set_parent(isp_clk, isp_clk_default);
		clk_disable_unprepare(isp_clk);
		goto exit;
	}

	/*isp enable*/
	ret = clk_prepare_enable(isp_eb);
	if (ret) {
		pr_info("fail to enable isp_eb.\n");
		goto exit;
	}

	ret = clk_prepare_enable(isp_axi_eb);
	if (ret) {
		pr_info("fail to enable isp_axi_eb.\n");
		goto exit;
	}

	flag = BIT_MM_AHB_ISP_EB;
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_EB, flag, flag);

	pr_info("isp_eb_clk end.\n");
exit:

	return ret;
}

static int isp_disable_clk(void)
{
	uint32_t flag = 0;

	/*cut off the isp colck source*/
	clk_disable_unprepare(isp_eb);
	clk_disable_unprepare(isp_axi_eb);

	/*set isp clock to default value before power off*/
	clk_set_parent(isp_clk, isp_clk_default);
	clk_disable_unprepare(isp_clk);

	flag = BIT_MM_AHB_ISP_EB;
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_EB, flag, ~flag);

	return 0;
}

int sprd_isp_reg_isr(enum isp_id id, enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data)
{
	int ret = 0;

	pr_debug("irq_id %d\n", irq_id);
	if (irq_id >= ISP_IMG_MAX) {
		pr_err("fail to get isp IRQ.\n");
		ret = ISP_RTN_IRQ_NUM_ERR;
	} else {
		pr_debug("irq user_data %p\n", user_data);
		ret = isp_irq_callback(id, irq_id, user_func, user_data);
		if (ret)
			pr_err("fail to Register isp callback\n");
	}

	return ret;
}

static int isp_module_init(struct isp_module *module_info, enum isp_id id)
{
	int i = 0, ret = 0;

	if (!module_info) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	/*cfg context struct init*/
	ret =  isp_cfg_ctx_init(&module_info->isp_cfg_contex,
		module_info->com_idx);
	if (unlikely(ret != 0)) {
		pr_err("Failed to init cfg context.\n");
		ret = -EIO;
		return ret;
	}

	ret = sprd_cam_frm_queue_init(
		&module_info->bin_frm_queue,
		CAM_QUEUE_RW_LOCK,
		sizeof(struct camera_frame),
		DCAM_FRM_QUEUE_LENGTH + 1,
		"binning frm queue");
	ret = sprd_cam_buf_queue_init(
		&module_info->bin_buf_queue,
		CAM_QUEUE_RW_LOCK,
		sizeof(struct camera_frame),
		DCAM_FRM_QUEUE_LENGTH + 1,
		"binning buf queue");
	ret = sprd_cam_frm_queue_init(
		&module_info->full_zsl_queue,
		CAM_QUEUE_RW_LOCK,
		sizeof(struct camera_frame),
		DCAM_FRM_QUEUE_LENGTH + 1,
		"zsl frm queue");

	if (ret) {
		pr_err("Fail to init frm queue.\n");
		return ret;
	}

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++) {
		/*isp store output buffer*/
		ret = sprd_cam_buf_queue_init(
			&module_info->isp_path[i].buf_queue,
			CAM_QUEUE_RW_LOCK,
			sizeof(struct camera_frame),
			DCAM_BUF_QUEUE_LENGTH,
			"isp path buffer queue");
		ret = sprd_cam_frm_queue_init(
			&module_info->isp_path[i].frame_queue,
			CAM_QUEUE_RW_LOCK,
			sizeof(struct camera_frame),
			DCAM_FRM_QUEUE_LENGTH + 1,
			"isp path frm queue");

		if (ret) {
			pr_err("Fail to init buf queue.\n");
			return ret;
		}

		ret = sprd_cam_buf_queue_init(
			&module_info->isp_path[i].coeff_queue,
			CAM_QUEUE_RW_LOCK,
			sizeof(struct isp_zoom_param),
			ISP_SC_COEFF_BUF_COUNT,
			"isp path coeff queue");

		if (ret) {
			pr_err("Fail to init coeff queue.\n");
			return ret;
		}
	}

	sprd_cam_frm_queue_clear(&module_info->bin_frm_queue);
	/*sprd_cam_frm_queue_clear(&module_info->full_frm_queue);*/
	sprd_cam_frm_queue_clear(&module_info->full_zsl_queue);

	init_completion(&module_info->scale_coeff_mem_com);
	complete(&module_info->scale_coeff_mem_com);

	return ret;
}

static int isp_module_deinit(struct isp_module *module_info, enum isp_id id)
{
	int i = 0, ret = 0;

	if (!module_info) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++) {
		ret = sprd_cam_buf_queue_deinit(
			&module_info->isp_path[i].buf_queue);

		if (ret) {
			pr_err("Fail to deinit buf queue.\n");
			return ret;
		}
		ret = sprd_cam_frm_queue_deinit(
			&module_info->isp_path[i].frame_queue);
		if (ret) {
			pr_err("Fail to deinit frm queue.\n");
			return ret;
		}
	}

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++) {
		ret = sprd_cam_buf_queue_deinit(
			&module_info->isp_path[i].coeff_queue);

		if (ret) {
			pr_err("Fail to init coeff queue.\n");
			return ret;
		}
	}

	sprd_cam_frm_queue_clear(&module_info->bin_frm_queue);
	/*sprd_cam_frm_queue_clear(&module_info->full_frm_queue);*/
	sprd_cam_frm_queue_clear(&module_info->full_zsl_queue);

	return ret;
}

int sprd_isp_module_en(void *isp_handle, enum isp_id id)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	ISP_SET_ISP_ID(dev->com_idx, id);
	pr_info("ISP%d: enable isp module in: %d, com_idx 0x%x\n", id,
		atomic_read(&s_isp_users[id]), dev->com_idx);

	mutex_lock(&isp_module_sema[id]);
	if (atomic_inc_return(&s_isp_users[id]) == 1) {
		if (atomic_read(&s_isp_total_users) == 0) {
			/*sprd_isp_pw_on */
			ret = sprd_cam_pw_on();
			if (ret != 0) {
				pr_err("fail to power on sprd cam\n");
				mutex_unlock(&isp_module_sema[id]);
				return ret;
			}

			sprd_cam_domain_eb();
			isp_enable_clk();
			/*sprd_irq_mask_disable(dev->com_idx);*/
			sprd_isp_reset(dev->com_idx);
			sprd_irq_mask_disable(dev->com_idx);
			sprd_irq_clear(dev->com_idx);
			sprd_irq_mask_enable(dev->com_idx);
		}

		ret = isp_irq_request(&s_isp_pdev->dev, &s_isp_irq[id], dev);
		if (ret) {
			pr_err("fail to install isp IRQ %d\n", ret);
			mutex_unlock(&isp_module_sema[id]);
			return ret;
		}

		ret = sprd_isp_buffer_alloc_k(dev);
		if (ret) {
			pr_err("fail to alloc buffer\n");
			mutex_unlock(&isp_module_sema[id]);
			return ret;
		}

		isp_set_reg_value_default(dev->com_idx);

	}
	atomic_inc(&s_isp_total_users);
	mutex_unlock(&isp_module_sema[id]);

	pr_info("ISP%d: enable isp module end: %d\n", id,
		atomic_read(&s_isp_users[id]));

	return ret;
}

int sprd_isp_module_dis(void *isp_handle, enum isp_id id)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;

	pr_info("ISP%d: disable isp moduel, in %d\n", id,
		atomic_read(&s_isp_users[id]));

	if (atomic_read(&s_isp_users[id]) == 0)
		return rtn;

	mutex_lock(&isp_module_sema[id]);
	if (atomic_dec_return(&s_isp_users[id]) == 0) {
		if (atomic_read(&s_isp_total_users) == 1) {
			isp_disable_clk();
			sprd_cam_domain_disable();
			rtn = sprd_cam_pw_off();
			if (rtn != 0) {
				pr_err("fail to power off sprd cam\n");
				mutex_unlock(&isp_module_sema[id]);
				return rtn;
			}
		}

		rtn = isp_irq_free(&s_isp_irq[id], dev);
		if (rtn)
			pr_err("fail to free isp IRQ %d\n", rtn);
	}
	atomic_dec(&s_isp_total_users);
	mutex_unlock(&isp_module_sema[id]);

	pr_info("ISP%d: disable isp moduel end. %d\n", id,
		atomic_read(&s_isp_users[id]));

	return rtn;
}

int sprd_isp_dev_init(void **isp_pipe_dev_handle, enum isp_id id)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_pipe_dev_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = vzalloc(sizeof(*dev));
	if (dev == NULL)
		return -ENOMEM;

	dev->com_idx = 0;
	if  (ISP_CFG_ON != 1)
		ISP_SET_MODE_ID(dev->com_idx, ISP_AP_MODE);
	ISP_SET_ISP_ID(dev->com_idx, id);
	dev->is_raw_capture = 0;
	dev->isp_busy = 0;
	mutex_init(&dev->isp_mutex);
	mutex_init(&dev->offline_thread_mutex);
	init_completion(&dev->irq_com);
	init_completion(&dev->isr_done_lock);

	dev->module_info.com_idx = dev->com_idx;
	ret = isp_module_init(&dev->module_info, id);
	if (unlikely(ret != 0)) {
		pr_err("fail to init isp module info\n");
		ret = -EIO;
		goto exit;
	}

	ret = isp_fmcu_slice_init(&dev->fmcu_slice.slice_handle);
	if (unlikely(ret != 0)) {
		pr_err("fail to init fmcu slice\n");
		ret = -EIO;
		goto fmcu_slice_exit;
	}
	ret = isp_create_offline_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to create offline thread\n");
		isp_stop_offline_thread(dev);
		ret = -EINVAL;
		goto offline_thread_exit;
	}
	pr_info("sprd_isp: dev init end!\n");

	*isp_pipe_dev_handle = (void *)dev;
	dev->isp_handle_addr = isp_pipe_dev_handle;
	s_isp_group.isp_dev[id] = dev;
	s_isp_group.dual_cap_sts = 0;
	s_isp_group.dual_fullpath_stop = 0;
	s_isp_group.first_need_wait = 0;
	s_isp_group.dual_cap_cnt = 0;
	init_completion(&s_isp_group.first_wait_com);
	return ret;
offline_thread_exit:
	isp_fmcu_slice_deinit(dev->fmcu_slice.slice_handle);
fmcu_slice_exit:
	isp_module_deinit(&dev->module_info, id);
exit:
	vfree(dev);
	pr_err("fail to init dev[%d]!\n", id);

	return ret;
}

int sprd_isp_dev_deinit(void *isp_pipe_dev_handle, enum isp_id id)
{
	int ret = 0;
	enum isp_work_mode work_mode = ISP_CFG_MODE;
	struct isp_pipe_dev *dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_pipe_dev_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_pipe_dev_handle;
	isp_k_param = &dev->isp_k_param;
	work_mode = ISP_GET_MODE_ID(dev->com_idx);
	mutex_lock(&dev->isp_mutex);

	ret = isp_module_deinit(&dev->module_info, id);
	if (unlikely(ret != 0))
		pr_err("fail to init queue\n");

	ret = isp_fmcu_slice_deinit(dev->fmcu_slice.slice_handle);
	if (unlikely(ret != 0))
		pr_err("fail to deinit fmcu slice\n");

	ret = isp_stop_offline_thread(dev);
	if (unlikely(ret != 0))
		pr_err("fail to stop offline thread\n");


	init_completion(&dev->irq_com);
	init_completion(&dev->isr_done_lock);

	mutex_unlock(&dev->isp_mutex);
	mutex_destroy(&dev->isp_mutex);
	mutex_destroy(&dev->offline_thread_mutex);

	isp_block_buf_free(isp_k_param);
	vfree(dev);

	pr_info("dev[%d] deinit OK!\n", id);

	return ret;
}

int sprd_isp_drv_init(struct platform_device *pdev)
{
	int i = 0;

	for (i = 0; i < s_isp_count; i++) {
		atomic_set(&s_isp_users[i], 0);
		mutex_init(&isp_module_sema[i]);
		isp_glb_reg_axi_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_axi_lock[i]);
		isp_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_mask_lock[i]);
		isp_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_clr_lock[i]);
	}
	isp_mod_lock = __SPIN_LOCK_UNLOCKED(&isp_mod_lock);
	atomic_set(&s_isp_total_users, 0);

	return 0;
}

void sprd_isp_drv_deinit(void)
{
	int i = 0;

	for (i = 0; i < s_isp_count; i++) {
		atomic_set(&s_isp_users[i], 0);
		s_isp_irq[i].irq0 = 0;
		s_isp_irq[i].irq1 = 0;
		mutex_init(&isp_module_sema[i]);
		isp_glb_reg_axi_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_axi_lock[i]);
		isp_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_mask_lock[i]);
		isp_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_clr_lock[i]);
	}
	isp_mod_lock = __SPIN_LOCK_UNLOCKED(&isp_mod_lock);
	atomic_set(&s_isp_total_users, 0);
}

int sprd_isp_update_clk(uint32_t clk_index, struct device_node *dn)
{
	int ret = 0;
	char *clk_name = NULL;
	char *clk_isp_if = NULL;
	struct device_node *isp_node = NULL;

	if (!dn) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	clk_isp_if = isp_clk_tab[clk_index].clock;
	clk_name = isp_clk_tab[clk_index].clk_name;

	pr_info("isp_clk_name is %s, clk_isp %s\n", clk_name, clk_isp_if);
	if (clk_name != NULL) {
		isp_node = of_parse_phandle(dn, "sprd,isp", 0);
		if (isp_node == NULL) {
			pr_err("fail to parse the property of sprd,isp\n");
			return -EFAULT;
		}

		isp_clk_parent = of_clk_get_by_name(isp_node, clk_name);
		if (IS_ERR_OR_NULL(isp_clk_parent)) {
			pr_err("fail to get isp_clk_parent\n");
			return PTR_ERR(isp_clk_parent);
		}

	} else {
		pr_err("clk_name is NULL & clk_isp_if %s\n", clk_isp_if);
	}


	if (clk_index == ISP_CLK_MAX_INDEX) {
		const char *value = NULL;

		value = __clk_get_name(isp_clk_parent);
		if (!strcmp(value, "clk_cppll")) {
			clk_set_rate(isp_clk_parent, CLK_CPPLL);
			pr_info("isp_clk_parent : %s\n",
				__clk_get_name(isp_clk_parent));
		}
	}

	ret = clk_set_parent(isp_clk, isp_clk_parent);
	if (ret) {
		pr_info("fail to set isp_clk_parent.\n");
		clk_set_parent(isp_clk, isp_clk_default);
		clk_disable_unprepare(isp_clk);
	}

	return ret;
}

int sprd_isp_parse_dt(struct device_node *dn, uint32_t *isp_count)
{
	int i = 0;
	uint32_t count = 0;
	void __iomem *reg_base;
	struct device_node *isp_node = NULL;
	struct resource res = {0};

	pr_info("isp dev device node %s, full name %s\n",
		dn->name, dn->full_name);
	isp_node = of_parse_phandle(dn, "sprd,isp", 0);
	if (isp_node == NULL) {
		pr_err("fail to parse the property of sprd,isp\n");
		return -EFAULT;
	}

	pr_info("after isp dev device node %s, full name %s\n",
		isp_node->name, isp_node->full_name);
	s_isp_pdev = of_find_device_by_node(isp_node);
	pr_info("sprd s_isp_pdev name %s\n", s_isp_pdev->name);
	if (of_device_is_compatible(isp_node, "sprd,isp")) {
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-count", 0, &count)) {
			pr_err("fail to parse the property of sprd,isp-count\n");
			return -EINVAL;
		}
		count++;
		s_isp_count = count;
		*isp_count = count;

		isp_eb = of_clk_get_by_name(isp_node, "isp_eb");
		if (IS_ERR_OR_NULL(isp_eb)) {
			pr_err("fail to get isp_eb\n");
			return PTR_ERR(isp_eb);
		}

		isp_axi_eb = of_clk_get_by_name(isp_node,
			"isp_axi_eb");
		if (IS_ERR_OR_NULL(isp_axi_eb)) {
			pr_err("fail to get isp_axi_eb\n");
			return PTR_ERR(isp_axi_eb);
		}

		isp_clk = of_clk_get_by_name(isp_node, "isp_clk");
		if (IS_ERR_OR_NULL(isp_clk)) {
			pr_err("fail to get isp_clk\n");
			return PTR_ERR(isp_clk);
		}

		isp_clk_parent = of_clk_get_by_name(isp_node,
			"isp_clk_parent");
		if (IS_ERR_OR_NULL(isp_clk_parent)) {
			pr_err("fail to get isp_clk_parent\n");
			return PTR_ERR(isp_clk_parent);
		}

		cam_ahb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,cam-ahb-syscon");
		if (IS_ERR_OR_NULL(cam_ahb_gpr))
			return PTR_ERR(cam_ahb_gpr);

		aon_apb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,aon-apb-syscon");
		if (IS_ERR_OR_NULL(aon_apb_gpr))
			return PTR_ERR(aon_apb_gpr);

		isp_clk_default = clk_get_parent(isp_clk);
		if (IS_ERR_OR_NULL(isp_clk_default)) {
			pr_err("fail to get isp_clk_default\n");
			return PTR_ERR(isp_clk_default);
		}

		if (of_address_to_resource(isp_node, 0, &res))
			pr_err("fail to get isp phys addr\n");

		isp_phys_base[0] = (unsigned long)res.start;
		pr_info("isp phys reg base %lx\n", isp_phys_base[0]);
		reg_base = of_iomap(isp_node, 0);
		if (!reg_base) {
			pr_err("fail to get isp reg_base %d\n", 0);
			return -ENXIO;
		}

		s_isp_regbase[0] = (unsigned long)reg_base;
		s_isp_irq[0].irq0 = irq_of_parse_and_map(isp_node, 0);
		s_isp_irq[0].irq1 = irq_of_parse_and_map(isp_node, 1);
		if (s_isp_irq[0].irq0 <= 0 || s_isp_irq[0].irq1 <= 0) {
			pr_err("fail to get isp irq %d\n", 0);
			return -EFAULT;
		}

		pr_info("ISP dts OK! base %lx, irq0 %d, irq1 %d",
			s_isp_regbase[0], s_isp_irq[0].irq0,
			s_isp_irq[0].irq1);

		for (i = 1; i < count; i++) {
			s_isp_regbase[i] = s_isp_regbase[0];
			isp_phys_base[i] = isp_phys_base[0];
			s_isp_irq[i].irq0 = s_isp_irq[0].irq0;
			s_isp_irq[i].irq1 = s_isp_irq[0].irq1;
		}
	} else {
		pr_err("fail to match isp device node\n");
		return -EINVAL;
	}

	return 0;
}

uint32_t sprd_isp_get_dual_cam_mode(void)
{
	return s_isp_group.dual_cam;
}
