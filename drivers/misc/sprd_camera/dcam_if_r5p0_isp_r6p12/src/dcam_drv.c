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

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/sprd_ion.h>
#include <linux/videodev2.h>
#include <linux/wakelock.h>
#include <video/sprd_mm.h>

#include "dcam_drv.h"
#include "cam_pw_domain.h"
#include "isp_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define DCAM_DRV_DEBUG
#define DCAM_CLK_NUM                   4
#define DCAM_AXI_STOP_TIMEOUT          1000
#define DCAM_STATE_QUICKQUIT           0x01
#define SHAR	                        0x53686172
#define KL3                            0x6B4C3300

struct dcam_if_clk_tag {
	char *clock;
	char *clk_name;
};
static const struct dcam_if_clk_tag dcam_if_clk_tab[DCAM_CLK_NUM] = {
	{"128", "dcam_clk_128m"},
	{"256", "dcam_clk_256m"},
	{"307", "dcam_clk_307m2"},
	{"384", "dcam_clk_384m"},
};

struct platform_device *s_dcam_pdev;
static struct mutex dcam_module_sema[DCAM_MAX_COUNT];
static atomic_t s_dcam_users[DCAM_MAX_COUNT];
static atomic_t s_dcam_total_users;
static struct dcam_module *s_p_dcam_mod[DCAM_MAX_COUNT];
static spinlock_t dcam_mod_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_cfg_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_control_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_mask_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_clr_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_ahbm_sts_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_endian_lock[DCAM_MAX_COUNT];

static struct clk *dcam_clk;
static struct clk *dcam_clk_parent;
static struct clk *dcam_clk_default;
static struct clk *dcam0_bpc_clk;
static struct clk *dcam0_bpc_clk_parent;
static struct clk *dcam0_bpc_clk_default;
static struct clk *dcam_axi_eb;
static struct clk *dcam_eb;
static struct regmap *cam_ahb_gpr;
static struct regmap *aon_apb;

unsigned int chip_id0;
unsigned int chip_id1;
unsigned long s_dcam_regbase[DCAM_MAX_COUNT];
unsigned long s_dcam_aximbase;
static uint32_t s_dcam_count;/*dts parsed useful dcam num*/
int s_dcam_irq[DCAM_MAX_COUNT];/*dts parsed INTC irq no*/
spinlock_t dcam_lock[DCAM_MAX_COUNT];


void dcam_reg_trace(enum dcam_id idx)
{
#ifdef DCAM_DRV_DEBUG
	unsigned long addr = 0;

	pr_info("cb: %pS\n", __builtin_return_address(0));

	pr_info("DCAM%d: Register list", idx);
	for (addr = DCAM_IP_REVISION; addr <= ISP_RAW_AFM_IIR_FILTER5;
		addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(idx, addr),
			DCAM_REG_RD(idx, addr + 4),
			DCAM_REG_RD(idx, addr + 8),
			DCAM_REG_RD(idx, addr + 12));
	}
	pr_info("AXIM: Register list");
	for (addr = DCAM_AXIM_CTRL; addr <= REG_DCAM_IMG_FETCH_RADDR;
		addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_AXIM_RD(addr),
			DCAM_AXIM_RD(addr + 4),
			DCAM_AXIM_RD(addr + 8),
			DCAM_AXIM_RD(addr + 12));
	}
#endif
}

struct dcam_module *get_dcam_module(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return NULL;
	}
	return s_p_dcam_mod[idx];
}

struct dcam_cap_desc *get_dcam_cap(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return NULL;
	}
	return &s_p_dcam_mod[idx]->dcam_cap;
}

struct dcam_path_desc *get_dcam_full_path(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return NULL;
	}
	return &s_p_dcam_mod[idx]->full_path;
}

struct dcam_path_desc *get_dcam_bin_path(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return NULL;
	}
	return &s_p_dcam_mod[idx]->bin_path;
}

struct dcam_fetch_desc *get_dcam_fetch(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return NULL;
	}
	return &s_p_dcam_mod[idx]->dcam_fetch;
}

struct dcam_fast_me_param *get_dcam_me_param(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return NULL;
	}
	return &s_p_dcam_mod[idx]->me_param;
}

void sprd_dcam_glb_reg_awr(enum dcam_id idx, unsigned long addr,
			uint32_t val, uint32_t reg_id)
{
	unsigned long flag = 0;

	switch (reg_id) {
	case DCAM_CFG_REG:
		spin_lock_irqsave(&dcam_glb_reg_cfg_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AHBM_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		REG_WR(addr, REG_RD(addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_AXIM_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		DCAM_AXIM_WR(addr, DCAM_AXIM_RD(addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	default:
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		break;
	}
}

void sprd_dcam_glb_reg_owr(enum dcam_id idx, unsigned long addr,
			uint32_t val, uint32_t reg_id)
{
	unsigned long flag = 0;

	switch (reg_id) {
	case DCAM_CFG_REG:
		spin_lock_irqsave(&dcam_glb_reg_cfg_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AHBM_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		REG_WR(addr, REG_RD(addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_AXIM_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		DCAM_AXIM_WR(addr, DCAM_AXIM_RD(addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	default:
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		break;
	}
}

void sprd_dcam_glb_reg_mwr(enum dcam_id idx, unsigned long addr,
			uint32_t mask, uint32_t val,
			uint32_t reg_id)
{
	unsigned long flag = 0;
	uint32_t tmp = 0;

	switch (reg_id) {
	case DCAM_CFG_REG:
		spin_lock_irqsave(&dcam_glb_reg_cfg_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AHBM_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		tmp = REG_RD(addr);
		tmp &= ~(mask);
		REG_WR(addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_AXIM_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		tmp = DCAM_AXIM_RD(addr);
		tmp &= ~(mask);
		DCAM_AXIM_WR(addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;

	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	default:
		pr_err("DCAM%d: fail to wr no global register 0x%0x:\n",
			idx, reg_id);
		break;
	}
}

void dcam_force_copy(enum dcam_id idx, enum camera_copy_id copy_id)
{
	uint32_t reg_val = 0;

	if (copy_id & CAP_COPY)
		reg_val |= BIT_4;
	if (copy_id & RDS_COPY)
		reg_val |= BIT_6;
	if (copy_id & FULL_COPY)
		reg_val |= BIT_8;
	if (copy_id & BIN_COPY)
		reg_val |= BIT_10;
	if (copy_id & AEM_COPY)
		reg_val |= BIT_12;
	if (copy_id & PDAF_COPY)
		reg_val |= BIT_14;
	if (copy_id & VCH2_COPY)
		reg_val |= BIT_16;
	if (copy_id & VCH3_COPY)
		reg_val |= BIT_18;

	pr_debug("DCAM%d: copy 0x%0x:\n", idx, reg_val);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, reg_val,
			reg_val, DCAM_CONTROL_REG);
}

void dcam_auto_copy(enum dcam_id idx, enum camera_copy_id copy_id)
{
	uint32_t reg_val = 0;

	if (copy_id & CAP_COPY)
		reg_val |= BIT_5;
	if (copy_id & RDS_COPY)
		reg_val |= BIT_7;
	if (copy_id & FULL_COPY)
		reg_val |= BIT_9;
	if (copy_id & BIN_COPY)
		reg_val |= BIT_11;
	if (copy_id & AEM_COPY)
		reg_val |= BIT_13;
	if (copy_id & PDAF_COPY)
		reg_val |= BIT_15;
	if (copy_id & VCH2_COPY)
		reg_val |= BIT_17;
	if (copy_id & VCH3_COPY)
		reg_val |= BIT_19;

	pr_debug("DCAM%d: copy 0x%0x:\n", idx, reg_val);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, reg_val,
			reg_val, DCAM_CONTROL_REG);
}

int sprd_dcam_reset(enum dcam_id idx, int is_irq)
{
	int i = 0;
	uint32_t time_out = 0, flag = 0;
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	pr_info("DCAM%d: reset:\n", idx);
	if (atomic_read(&s_dcam_total_users) == 1) {
		/* firstly, stop AXI writing */
		pr_info("DCAM%d: reset :\n", idx);
		sprd_dcam_glb_reg_owr(idx, DCAM_AXIM_CTRL, BIT_24 | BIT_23,
			DCAM_AXIM_REG);
	}

	/* then wait for AHB busy cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_AXIM_RD(DCAM_AXIM_DBG_STS) & 0x0F))
			break;
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_info("DCAM%d: reset timeout, axim status 0x%x\n", idx,
			DCAM_AXIM_RD(DCAM_AXIM_DBG_STS));
		/*return DCAM_RTN_TIMEOUT;*/
	}

	if (idx == DCAM_ID_0)
		flag = BIT_MM_AHB_DCAM0_SOFT_RST;
	else if (idx == DCAM_ID_1)
		flag = BIT_MM_AHB_DCAM1_SOFT_RST;
	if (chip_id0 == KL3 && chip_id1 == SHAR && idx == DCAM_ID_2)
		flag = DCAM2_SOFT_RST;

	if (is_irq)
		flag = BIT_MM_AHB_DCAM_ALL_SOFT_RST;

	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	for (i = 0x200; i < 0x400; i += 4)
		DCAM_REG_WR(idx, i, 0);

	sprd_dcam_glb_reg_owr(idx, DCAM_INT_CLR,
		DCAM_IRQ_LINE_MASK, DCAM_INIT_CLR_REG);
	sprd_dcam_glb_reg_owr(idx, DCAM_INT_EN,
		DCAM_IRQ_LINE_MASK, DCAM_INIT_MASK_REG);

	/* the end, enable AXI writing */
	sprd_dcam_glb_reg_awr(idx, DCAM_AXIM_CTRL, ~(BIT_24 | BIT_23),
		DCAM_AXIM_REG);

	pr_info("DCAM%d: reset end\n", idx);

	return -rtn;
}

int sprd_dcam_path_pause(enum dcam_id idx, uint32_t channel_id)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *path = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("DCAM%d: path %x\n", idx, channel_id);
	if (channel_id == CAMERA_FULL_PATH) {
		sprd_dcam_glb_reg_awr(idx, DCAM_CFG, ~BIT_1, DCAM_CFG_REG);
		dcam_force_copy(idx, FULL_COPY);
		path = &s_p_dcam_mod[idx]->full_path;
	} else {
		sprd_dcam_glb_reg_awr(idx, DCAM_CFG, ~BIT_2, DCAM_CFG_REG);
		path = &s_p_dcam_mod[idx]->bin_path;
	}
	path->status = DCAM_ST_PAUSE;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	return -rtn;
}

int sprd_dcam_path_resume(enum dcam_id idx, uint32_t channel_id)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *path = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("DCAM%d: path %x\n", idx, channel_id);
	if (channel_id == CAMERA_FULL_PATH) {
		sprd_dcam_glb_reg_owr(idx, DCAM_CFG, BIT_1, DCAM_CFG_REG);
		dcam_force_copy(idx, FULL_COPY);
		path = &s_p_dcam_mod[idx]->full_path;
	} else {
		sprd_dcam_glb_reg_owr(idx, DCAM_CFG, BIT_2, DCAM_CFG_REG);
		path = &s_p_dcam_mod[idx]->bin_path;
	}
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	return -rtn;
}
int sprd_dcam_start(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	uint32_t cap_en = 0;
	unsigned int reg_val = 0;
	/*struct dcam_path_desc *full_path = get_dcam_full_path(idx);*/

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("DCAM%d: dcam start %x\n", idx,
		s_p_dcam_mod[idx]->dcam_cap.cap_mode);
	s_p_dcam_mod[idx]->statis_module_info.aem_status = 0;

#if 1
	if (s_p_dcam_mod[idx]->full_path.src_sel
			== 0) {
		if (idx == 1)
			DCAM_REG_MWR(idx, 0x180,
				0x01 << 0, 0x01 << 0);/*afm bypass*/
	} else
		DCAM_REG_MWR(idx, ISP_BPC_GC_CFG,
			0x07 << 0, 0x06 << 0);/*bpc and gc enable*/
#endif
	dcam_start_full_path(idx);
	dcam_start_bin_path(idx);

	/*set statis buf before stream on*/
	rtn = sprd_cam_set_statis_buf(idx,
		&s_p_dcam_mod[idx]->statis_module_info);
	if (rtn) {
		pr_err("fail to start isp set statis buf\n");
		return -rtn;
	}

	cap_en = DCAM_REG_RD(idx, DCAM_CFG) & BIT_0;
	pr_debug("DCAM%d: cap_eb %d 0x%x\n", idx, cap_en,
		DCAM_REG_RD(idx, DCAM_CFG));

	if (cap_en == 0) {
		DCAM_REG_WR(idx, DCAM_IMAGE_CONTROL,
			0x2b << 8 | 0x01);

		if (s_p_dcam_mod[idx]->full_path.valid
			&& (!s_p_dcam_mod[idx]->bin_path.valid)) {
			DCAM_REG_WR(idx, DCAM_BIN_BASE_WADDR0,
				s_p_dcam_mod[idx]->full_path.reserved_frame
				.buf_info.iova[0]);
			sprd_dcam_glb_reg_awr(idx, DCAM_INT_EN,
				~(1 << DCAM_BIN_PATH_TX_DONE),
				DCAM_INIT_MASK_REG);
		}

		/* Cap force copy */
		dcam_force_copy(idx, ALL_COPY);
		/* Cap Enable */
		sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_0, BIT_0,
				DCAM_CONTROL_REG);
	}

	if (s_p_dcam_mod[idx]->need_nr3) {
		struct camera_size size = {0};

		size.w = s_p_dcam_mod[idx]->me_param.cap_in_size_w;
		size.h = s_p_dcam_mod[idx]->me_param.cap_in_size_h;
		pr_debug("3DNR sprd_dcam_start size w=%d,h=%d\n",
			size.w, size.h);
		rtn = sprd_dcam_set_3dnr_me(idx, &size);
		if (rtn) {
			pr_err("fail to get_fast_me param\n");
			return -rtn;
		}
	}

	if (chip_id0 == KL3)
		reg_val = (0x0 << 20) | (0xA << 12) | (0x8 << 8) |
					(0xD << 4) | 0xA;
	else
		reg_val = (0x0 << 20) | (0xB << 12) | (0x2 << 8) |
					(0xE << 4) | 0xB;
	sprd_dcam_glb_reg_mwr(idx,
			      DCAM_AXIM_CTRL,
			      DCAM_AXIM_AQOS_MASK,
			      reg_val,
			      DCAM_AXIM_REG);

	return -rtn;
}

int sprd_dcam_stop(enum dcam_id idx, int is_irq)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned long flag;
	int32_t ret = 0;
	int time_out = 5000;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	s_p_dcam_mod[idx]->statis_module_info.aem_status = 0;
	s_p_dcam_mod[idx]->state |= DCAM_STATE_QUICKQUIT;
	s_p_dcam_mod[idx]->full_path.status = DCAM_ST_STOP;
	s_p_dcam_mod[idx]->bin_path.status = DCAM_ST_STOP;

	if (!is_irq)
		spin_lock_irqsave(&dcam_lock[idx], flag);

	sprd_dcam_glb_reg_owr(idx, DCAM_PATH_STOP, 0x3F, DCAM_CONTROL_REG);
	udelay(1000);

	/* wait for AHB path busy cleared */
	while (time_out) {
		ret = DCAM_REG_RD(idx, DCAM_PATH_BUSY) & 0xFFF;
		if (!ret)
			break;
		time_out--;
	}
	if (!time_out)
		pr_info("DCAM%d: stop path timeout 0x%x\n", idx, ret);

	if (!is_irq)
		spin_unlock_irqrestore(&dcam_lock[idx], flag);

	rtn = sprd_dcam_reset(idx, is_irq);
	s_p_dcam_mod[idx]->state &= ~DCAM_STATE_QUICKQUIT;

	pr_info("dcam stop\n");
	return -rtn;
}

int sprd_dcam_update_clk(uint32_t clk_index, struct device_node *dn)
{
	int ret = 0;
	char *clk_dcam_if = NULL;
	char *clk_name = NULL;

	if (!dn) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	clk_dcam_if = dcam_if_clk_tab[clk_index].clock;
	clk_name = dcam_if_clk_tab[clk_index].clk_name;

	if (clk_name != NULL) {
		dcam_clk_parent = of_clk_get_by_name(dn, clk_name);
		if (IS_ERR_OR_NULL(dcam_clk_parent)) {
			pr_err("fail to get dcam_clk_parent\n");
			return PTR_ERR(dcam_clk_parent);
		}
	} else {
		pr_err("fail to get valid clk_name &clk_dcam_if %s\n",
			clk_dcam_if);
	}

	ret = clk_set_parent(dcam_clk, dcam_clk_parent);
	if (ret) {
		pr_err("fail to set dcam_clk_parent\n");
		clk_set_parent(dcam_clk, dcam_clk_default);
		clk_disable_unprepare(dcam_eb);
	}

	pr_info("dcam_clk_name is %s\n", clk_name);
	return ret;
}

int sprd_dcam_module_init(enum dcam_id idx)
{
	int ret = 0;

	if (atomic_read(&s_dcam_users[idx]) < 1) {
		pr_err("fail to get user,s_dcam_users[%d] equal to %d",
			idx, atomic_read(&s_dcam_users[idx]));
		return -EIO;
	}
	dcam_full_path_init(idx);
	dcam_bin_path_init(idx);

	ret = sprd_cam_init_statis_queue(
		&s_p_dcam_mod[idx]->statis_module_info);
	s_p_dcam_mod[idx]->frame_id = 0;

	return ret;
}

int sprd_dcam_module_deinit(enum dcam_id idx)
{
	dcam_full_path_deinit(idx);
	dcam_bin_path_deinit(idx);

	sprd_cam_clear_statis_queue(
			&s_p_dcam_mod[idx]->statis_module_info);

	sprd_cam_deinit_statis_queue(
		&s_p_dcam_mod[idx]->statis_module_info);
	s_p_dcam_mod[idx]->need_nr3 = 0;
	s_p_dcam_mod[idx]->frame_id = 0;

	return 0;
}

int sprd_camera_get_path_id(struct camera_get_path_id *path_id,
	uint32_t *channel_id, uint32_t scene_mode)
{
	int ret = DCAM_RTN_SUCCESS;

	if (path_id == NULL || channel_id == NULL)
		return -1;

	pr_info("DCAM: fourcc 0x%x input %d %d output %d %d\n",
		path_id->fourcc,
		path_id->input_size.w, path_id->input_size.h,
		path_id->output_size.w, path_id->output_size.h);
	pr_info("DCAM: input_trim %d %d %d %d need_isp %d\n",
		path_id->input_trim.x, path_id->input_trim.y,
		path_id->input_trim.w, path_id->input_trim.h,
		path_id->need_isp);
	pr_info("DCAM: is_path_work %d %d %d %d %d\n",
		path_id->is_path_work[CAMERA_FULL_PATH],
		path_id->is_path_work[CAMERA_BIN_PATH],
		path_id->is_path_work[CAMERA_PRE_PATH],
		path_id->is_path_work[CAMERA_VID_PATH],
		path_id->is_path_work[CAMERA_CAP_PATH]);
	pr_info("DCAM: scene_mode %d, need_isp_tool %d\n", scene_mode,
		path_id->need_isp_tool);

	if (path_id->need_isp_tool)
		*channel_id = CAMERA_FULL_PATH;
	else if (path_id->fourcc == V4L2_PIX_FMT_GREY &&
		 !path_id->is_path_work[CAMERA_FULL_PATH])
		*channel_id = CAMERA_FULL_PATH;
	else if ((path_id->sn_fmt == IMG_PIX_FMT_NV12 ||
		path_id->sn_fmt == IMG_PIX_FMT_NV21) &&
		!path_id->is_path_work[CAMERA_FULL_PATH] &&
		!path_id->is_path_work[CAMERA_BIN_PATH])
		*channel_id = CAMERA_FULL_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_PREVIEW) {
		if (path_id->output_size.w > ISP_PATH2_LINE_BUF_LENGTH &&
			path_id->output_size.w <= ISP_PATH3_LINE_BUF_LENGTH &&
			!path_id->is_path_work[CAMERA_CAP_PATH])
			*channel_id = CAMERA_CAP_PATH;
		else if (path_id->output_size.w > ISP_PATH1_LINE_BUF_LENGTH &&
			path_id->output_size.w <= ISP_PATH2_LINE_BUF_LENGTH &&
			!path_id->is_path_work[CAMERA_VID_PATH])
			*channel_id = CAMERA_VID_PATH;
		else
			*channel_id = CAMERA_PRE_PATH;
	} else if (scene_mode == DCAM_SCENE_MODE_RECORDING)
		*channel_id = CAMERA_VID_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_CAPTURE)
		*channel_id = CAMERA_CAP_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK) {
		if (path_id->output_size.w <= ISP_PATH2_LINE_BUF_LENGTH
			&& !path_id->is_path_work[CAMERA_VID_PATH])
			*channel_id = CAMERA_VID_PATH;
		else
			*channel_id = CAMERA_CAP_PATH;
	} else {
		*channel_id = CAMERA_FULL_PATH;
		pr_info("fail to select path:error\n");
	}
	pr_info("path id %d\n", *channel_id);

	return ret;
}

int sprd_dcam_get_path_capability(struct cam_path_capability *capacity)
{
	if (capacity == NULL)
		return -1;

	capacity->count = 5;
	capacity->support_3dnr_mode = SPRD_3DNR_HW;

	capacity->path_info[CAMERA_FULL_PATH].line_buf = 0;
	capacity->path_info[CAMERA_FULL_PATH].support_yuv = 0;
	capacity->path_info[CAMERA_FULL_PATH].support_raw = 1;
	capacity->path_info[CAMERA_FULL_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_FULL_PATH].support_scaling = 0;
	capacity->path_info[CAMERA_FULL_PATH].support_trim = 1;
	capacity->path_info[CAMERA_FULL_PATH].is_scaleing_path = 0;

	capacity->path_info[CAMERA_BIN_PATH].line_buf = 0;
	capacity->path_info[CAMERA_BIN_PATH].support_yuv = 0;
	capacity->path_info[CAMERA_BIN_PATH].support_raw = 1;
	capacity->path_info[CAMERA_BIN_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_BIN_PATH].support_scaling = 0;
	capacity->path_info[CAMERA_BIN_PATH].support_trim = 1;
	capacity->path_info[CAMERA_BIN_PATH].is_scaleing_path = 0;

	capacity->path_info[CAMERA_PRE_PATH].line_buf =
		ISP_PATH1_LINE_BUF_LENGTH;
	capacity->path_info[CAMERA_PRE_PATH].support_yuv = 1;
	capacity->path_info[CAMERA_PRE_PATH].support_raw = 0;
	capacity->path_info[CAMERA_PRE_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_PRE_PATH].support_scaling = 1;
	capacity->path_info[CAMERA_PRE_PATH].support_trim = 1;
	capacity->path_info[CAMERA_PRE_PATH].is_scaleing_path = 0;

	capacity->path_info[CAMERA_VID_PATH].line_buf =
		ISP_PATH2_LINE_BUF_LENGTH;
	capacity->path_info[CAMERA_VID_PATH].support_yuv = 1;
	capacity->path_info[CAMERA_VID_PATH].support_raw = 0;
	capacity->path_info[CAMERA_VID_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_VID_PATH].support_scaling = 1;
	capacity->path_info[CAMERA_VID_PATH].support_trim = 1;
	capacity->path_info[CAMERA_VID_PATH].is_scaleing_path = 0;

	capacity->path_info[CAMERA_CAP_PATH].line_buf =
		ISP_PATH3_LINE_BUF_LENGTH;
	capacity->path_info[CAMERA_CAP_PATH].support_yuv = 1;
	capacity->path_info[CAMERA_CAP_PATH].support_raw = 0;
	capacity->path_info[CAMERA_CAP_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_CAP_PATH].support_scaling = 1;
	capacity->path_info[CAMERA_CAP_PATH].support_trim = 1;
	capacity->path_info[CAMERA_CAP_PATH].is_scaleing_path = 0;

	return 0;
}

static int sprd_dcam_enable_clk(enum dcam_id idx)
{
	int ret = 0;
	uint32_t flag = 0;

	if (atomic_read(&s_dcam_total_users) != 0)
		return ret;

	/*set dcam_if clock to max value*/
	ret = clk_set_parent(dcam_clk, dcam_clk_parent);
	if (ret) {
		pr_err("fail to set dcam_clk_parent\n");
		clk_set_parent(dcam_clk, dcam_clk_default);
		clk_disable_unprepare(dcam_eb);
		goto exit;
	}

	ret = clk_prepare_enable(dcam_clk);
	if (ret) {
		pr_err("fail to enable dcam_clk\n");
		clk_set_parent(dcam_clk, dcam_clk_default);
		clk_disable_unprepare(dcam_eb);
		goto exit;
	}

	ret = clk_set_parent(dcam0_bpc_clk, dcam0_bpc_clk_parent);
	if (ret) {
		pr_err("fail to set dcam0_bpc_clk_parent\n");
		clk_set_parent(dcam0_bpc_clk, dcam0_bpc_clk_parent);
		goto exit;
	}

	ret = clk_prepare_enable(dcam0_bpc_clk);
	if (ret) {
		pr_err("fail to enable dcam0_bpc_clk\n");
		clk_set_parent(dcam0_bpc_clk, dcam0_bpc_clk_default);
		goto exit;
	}

	if (chip_id0 != KL3)
		regmap_update_bits(cam_ahb_gpr, REG_MM_AHB_AHB_EB,
			BPC_CLK_EB_LEP, BPC_CLK_EB_LEP);
	/*dcam enable*/
	ret = clk_prepare_enable(dcam_eb);
	if (ret) {
		pr_info("fail to enable dcam0.\n");
		goto exit;
	}

	if (chip_id0 == KL3 && chip_id1 == SHAR) {
		ret = clk_prepare_enable(dcam_axi_eb);
		if (ret) {
			pr_info("fail to enable dcam_axi_eb.\n");
			goto exit;
		}
	}
	flag = BIT_MM_AHB_DCAM_AXIM_SOFT_RST |
			BIT_MM_AHB_DCAM_ALL_SOFT_RST;
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	pr_info("sprd_dcam_enable_clk end.\n");
exit:

	return ret;
}

static int sprd_dcam_disable_clk(enum dcam_id idx)
{
	if (atomic_read(&s_dcam_total_users) != 1)
		return 0;

	/*cut off the dcam_if colck source*/
	if (chip_id0 == KL3 && chip_id1 == SHAR)
		clk_disable_unprepare(dcam_axi_eb);
	clk_disable_unprepare(dcam_eb);

	/*set dcam_if clock to default value before power off*/
	clk_set_parent(dcam_clk, dcam_clk_default);
	clk_disable_unprepare(dcam_clk);
	clk_set_parent(dcam0_bpc_clk, dcam0_bpc_clk_default);
	clk_disable_unprepare(dcam0_bpc_clk);

	pr_info("sprd_dcam_disable_clk end.\n");
	return 0;
}

static int dcam_internal_init(enum dcam_id idx)
{
	int ret = 0;

	s_p_dcam_mod[idx] = vzalloc(sizeof(struct dcam_module));
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	return ret;
}

static void dcam_internal_deinit(enum dcam_id idx)
{
	unsigned long flag = 0;

	spin_lock_irqsave(&dcam_mod_lock[idx], flag);
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_info("DCAM%d: fail to get valid addr, %p",
			idx, s_p_dcam_mod[idx]);
	} else {
		vfree(s_p_dcam_mod[idx]);
		s_p_dcam_mod[idx] = NULL;
	}
	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);

}

int sprd_dcam_module_en(enum dcam_id idx)
{
	int ret = 0;

	pr_info("DCAM%d: enable dcam module, in %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	mutex_lock(&dcam_module_sema[idx]);
	if (atomic_inc_return(&s_dcam_users[idx]) == 1) {

		ret = dcam_internal_init(idx);
		if (ret != 0) {
			pr_err("fail to power on sprd cam\n");
			goto internal_exit;
		}

		ret = sprd_cam_pw_on();
		if (ret != 0) {
			pr_err("fail to power on sprd cam\n");
			goto cam_pw_on_exit;
		}
		sprd_cam_domain_eb();
		sprd_dcam_enable_clk(idx);
		sprd_dcam_reset(idx, 0);

		ret = dcam_irq_request(idx, s_p_dcam_mod[idx]);
		if (ret) {
			pr_err("fail to install IRQ %d\n", ret);
			goto cam_irq_exit;
		}
#if 0
		memset((void *)&s_user_func[idx][0], 0,
			sizeof(void *) * DCAM_IRQ_NUMBER);
		memset((void *)&s_user_data[idx][0], 0,
			sizeof(void *) * DCAM_IRQ_NUMBER);
#endif
		pr_info("DCAM%d: register isr, 0x%x\n", idx,
			DCAM_REG_RD(idx, DCAM_INT_EN));
	}
	atomic_inc(&s_dcam_total_users);
	mutex_unlock(&dcam_module_sema[idx]);

	pr_info("DCAM%d: enable dcam module, end %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	return ret;

cam_irq_exit:
	sprd_dcam_disable_clk(idx);
	sprd_cam_domain_disable();
cam_pw_on_exit:
	dcam_internal_deinit(idx);
internal_exit:
	atomic_dec_return(&s_dcam_users[idx]);
	mutex_unlock(&dcam_module_sema[idx]);
	return ret;
}

int sprd_dcam_module_dis(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	pr_info("DCAM%d: disable dcam module, in %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	if (atomic_read(&s_dcam_users[idx]) == 0)
		return rtn;

	mutex_lock(&dcam_module_sema[idx]);
	if (atomic_dec_return(&s_dcam_users[idx]) == 0) {
		dcam_irq_free(idx, s_p_dcam_mod[idx]);
		dcam_internal_deinit(idx);

		sprd_dcam_disable_clk(idx);
		sprd_cam_domain_disable();
		rtn = sprd_cam_pw_off();
		if (rtn != 0) {
			pr_err("fail to power off sprd cam\n");
			mutex_unlock(&dcam_module_sema[idx]);
			return rtn;
		}
	}
	atomic_dec(&s_dcam_total_users);
	mutex_unlock(&dcam_module_sema[idx]);

	return rtn;
}

int sprd_dcam_drv_init(struct platform_device *p_dev)
{
	int i = 0;

	s_dcam_pdev = p_dev;
	for (i = 0; i < s_dcam_count; i++) {
		atomic_set(&s_dcam_users[i], 0);
		s_p_dcam_mod[i] = NULL;

		mutex_init(&dcam_module_sema[i]);

		dcam_mod_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_mod_lock);
		dcam_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_lock);
		dcam_glb_reg_cfg_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_cfg_lock);
		dcam_glb_reg_control_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_control_lock);
		dcam_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_mask_lock);
		dcam_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_clr_lock);
		dcam_glb_reg_ahbm_sts_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_ahbm_sts_lock);
		dcam_glb_reg_endian_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_endian_lock);
	}
	atomic_set(&s_dcam_total_users, 0);

	return 0;
}

void sprd_dcam_drv_deinit(void)
{
	int i = 0;

	for (i = 0; i < s_dcam_count; i++) {
		atomic_set(&s_dcam_users[i], 0);
		s_p_dcam_mod[i] = NULL;
		s_dcam_irq[i] = 0;

		mutex_init(&dcam_module_sema[i]);

		dcam_mod_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_mod_lock);
		dcam_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_lock);
		dcam_glb_reg_cfg_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_cfg_lock);
		dcam_glb_reg_control_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_control_lock);
		dcam_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_mask_lock);
		dcam_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_clr_lock);
		dcam_glb_reg_ahbm_sts_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_ahbm_sts_lock);
		dcam_glb_reg_endian_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_endian_lock);
	}
	atomic_set(&s_dcam_total_users, 0);
}

int sprd_dcam_get_chip_id(void)
{
	enum chip_id idx = 0;

	if (chip_id0 == KL3 && chip_id1 == SHAR)
		idx = SHARKL3;
	else
		idx = SHARKLEP;

	return idx;
}

int sprd_dcam_parse_dt(struct device_node *dn, uint32_t *dcam_count)
{
	int i = 0, ret = 0;
	uint32_t count = 0;
	void __iomem *reg_base;

	if (!dn || !dcam_count) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	pr_info("Start dcam dts parse\n");
	if (of_property_read_u32_index(dn, "sprd,dcam-count", 0, &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	s_dcam_count = count;
	*dcam_count = count;

	dcam_eb = of_clk_get_by_name(dn, "dcam_eb");
	if (IS_ERR_OR_NULL(dcam_eb)) {
		pr_err("fail to get dcam_eb\n");
		return PTR_ERR(dcam_eb);
	}

	dcam_clk = of_clk_get_by_name(dn, "dcam_clk");
	pr_debug("dcam_clk : %s", __clk_get_name(dcam_clk));
	if (IS_ERR_OR_NULL(dcam_clk)) {
		pr_err("fail to get dcam_clk\n");
		return PTR_ERR(dcam_clk);
	}

	dcam_clk_parent = of_clk_get_by_name(dn, "dcam_clk_parent");
	pr_debug("dcam_clk_parent : %s", __clk_get_name(dcam_clk_parent));
	if (IS_ERR_OR_NULL(dcam_clk_parent)) {
		pr_err("fail to get dcam_clk_parent\n");
		return PTR_ERR(dcam_clk_parent);
	}

	dcam_clk_default = clk_get_parent(dcam_clk);
	if (IS_ERR_OR_NULL(dcam_clk_default)) {
		pr_err("fail to get dcam_clk_default\n");
		return PTR_ERR(dcam_clk_default);
	}

	dcam0_bpc_clk = of_clk_get_by_name(dn, "dcam_bpc_clk");
	pr_info("dcam0_bpc_clk : %s", __clk_get_name(dcam0_bpc_clk));
	if (IS_ERR_OR_NULL(dcam0_bpc_clk)) {
		pr_err("fail to get dcam0_bpc_clk\n");
		return PTR_ERR(dcam0_bpc_clk);
	}

	dcam0_bpc_clk_parent = of_clk_get_by_name(dn, "dcam_bpc_clk_parent");
	pr_info("dcam0_bpc_clk_parent : %s",
		__clk_get_name(dcam0_bpc_clk_parent));
	if (IS_ERR_OR_NULL(dcam0_bpc_clk_parent)) {
		pr_err("fail to get dcam0_bpc_clk_parent\n");
		return PTR_ERR(dcam0_bpc_clk_parent);
	}

	dcam0_bpc_clk_default = clk_get_parent(dcam0_bpc_clk);
	if (IS_ERR_OR_NULL(dcam0_bpc_clk_default)) {
		pr_err("fail to get dcam0_bpc_clk_default\n");
		return PTR_ERR(dcam0_bpc_clk_default);
	}

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(dn,
		"sprd,cam-ahb-syscon");
	if (IS_ERR_OR_NULL(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);

	aon_apb = syscon_regmap_lookup_by_phandle(dn,
		"sprd,aon-apb-syscon");
	if (IS_ERR_OR_NULL(aon_apb))
		return PTR_ERR(aon_apb);

	ret = regmap_read(aon_apb, REG_AON_APB_AON_CHIP_ID0, &chip_id0);
	if (unlikely(ret))
		goto exit;
	ret = regmap_read(aon_apb, REG_AON_APB_AON_CHIP_ID1, &chip_id1);
	if (unlikely(ret))
		goto exit;
	if (chip_id0 == KL3 && chip_id1 == SHAR) {
		dcam_axi_eb = of_clk_get_by_name(dn, "dcam_axi_eb");
		if (IS_ERR_OR_NULL(dcam_axi_eb)) {
			pr_err("fail to get dcam_axi_eb\n");
			return PTR_ERR(dcam_axi_eb);
		}
	}

	for (i = 0; i < count; i++) {
		s_dcam_irq[i] = irq_of_parse_and_map(dn, i);
		if (s_dcam_irq[i] <= 0) {
			pr_err("fail to get dcam irq %d\n", i);
			return -EFAULT;
		}

		reg_base = of_iomap(dn, i);
		if (!reg_base) {
			pr_err("fail to get dcam reg_base %d\n", i);
			return -ENXIO;
		}
		s_dcam_regbase[i] = (unsigned long)reg_base;

		pr_info("Dcam dts OK! base %lx, irq %d\n", s_dcam_regbase[i],
			s_dcam_irq[i]);
	}
	if (chip_id0 == KL3 && chip_id1 == SHAR)
		reg_base = of_iomap(dn, i);
	else
		reg_base = of_iomap(dn, i + 1);
	if (!reg_base) {
		pr_err("fail to get dcam axim_base %d\n", i);
		return -ENXIO;
	}
	s_dcam_aximbase = (unsigned long)reg_base;

	return 0;
exit:
	return ret;
}

static int sprd_dcam_3dnr_me_crop(int len, int max_len, int *x_y, int *out_len)
{
	if (len <= max_len) {
		*out_len = len;
		*x_y = 0;
	} else {
		*out_len = max_len;
		*x_y = (len - max_len) / 2;
		if ((*x_y%2) != 0)
			*x_y = *x_y - 1;
	}
	pr_debug("3DNR 3dnr_me_crop len=%d,out=%d,x_y=%d\n",
		len, *out_len, *x_y);

	return 0;
}

int sprd_dcam_set_3dnr_me(uint32_t idx, void *size)
{
	unsigned int val = 0;
	int roi_start_x = 0, roi_start_y = 0, roi_width = 0, roi_height = 0;
	int nr3_channel_sel = 0, nr3_project_mode = 0;
	struct camera_size *p_size = NULL;
	int roi_width_max = 0;
	int roi_height_max = 0;
	struct dcam_fast_me_param *me_param =
		get_dcam_me_param(idx);

	if (!size) {
		pr_err("fail to nr3_fast_me get valid input ptr\n");
		return -DCAM_RTN_PARA_ERR;
	}

	p_size = (struct camera_size *)size;
	val = DCAM_REG_RD(idx, DCAM_NR3_PARA0);
	nr3_project_mode = val & 0x3;
	nr3_channel_sel = (val >> 2) & 0x3;

	if (idx == 0) {
		if (nr3_project_mode == 0) {
			roi_width_max = DCAM0_3DNR_ME_WIDTH_MAX/2;
			roi_height_max = DCAM0_3DNR_ME_HEIGHT_MAX/2;
		} else {
			roi_width_max = DCAM0_3DNR_ME_WIDTH_MAX;
			roi_height_max = DCAM0_3DNR_ME_HEIGHT_MAX;
		}
	} else {
		if (nr3_project_mode == 0) {
			roi_width_max = DCAM1_3DNR_ME_WIDTH_MAX/2;
			roi_height_max = DCAM1_3DNR_ME_HEIGHT_MAX/2;
		} else {
			roi_width_max = DCAM1_3DNR_ME_WIDTH_MAX;
			roi_height_max = DCAM1_3DNR_ME_HEIGHT_MAX;
		}
	}
	sprd_dcam_3dnr_me_crop(p_size->w,
		roi_width_max,
		&roi_start_x,
		&roi_width);
	sprd_dcam_3dnr_me_crop(p_size->h,
		roi_height_max,
		&roi_start_y,
		&roi_height);

	me_param->nr3_bypass = 0;
	me_param->nr3_channel_sel = nr3_channel_sel;
	me_param->nr3_project_mode = nr3_project_mode;
	me_param->roi_start_x = roi_start_x;
	me_param->roi_start_y = roi_start_y;
	me_param->roi_width = roi_width & 0x1FF8;
	me_param->roi_height = (roi_height - 40) & 0x1FF8;
	pr_debug("3DNR roi_start_x=%d, roi_start_y=%d, w=%d,h=%d\n",
		roi_start_x, roi_start_y,
		me_param->roi_width, me_param->roi_height);

	DCAM_REG_MWR(idx, DCAM_NR3_PARA1, BIT_0,
		me_param->nr3_bypass);
	val = ((me_param->roi_start_x & 0x1FFF) << 16)
		|(me_param->roi_start_y & 0x1FFF);
	DCAM_REG_MWR(idx, DCAM_NR3_ROI_PARA0,
			0x1FFF1FFF, val);
	val = ((me_param->roi_width & 0x1FFF) << 16)
		|(me_param->roi_height & 0x1FFF);
	DCAM_REG_MWR(idx, DCAM_NR3_ROI_PARA1,
		0x1FFF1FFF, val);

	return 0;
}

int sprd_dcam_fast_me_info(enum dcam_id idx,
	uint32_t need_nr3, struct camera_size *size)
{
	if (!size) {
		pr_err("fail to nr3_fast_me get valid input ptr\n");
		return -DCAM_RTN_PARA_ERR;
	}

	get_dcam_module(idx)->need_nr3 = need_nr3;
	get_dcam_module(idx)->me_param.cap_in_size_w = size->w;
	get_dcam_module(idx)->me_param.cap_in_size_h = size->h;

	return 0;
}
