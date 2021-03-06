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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <video/sprd_mm.h>

#include "dcam_drv.h"
#include "isp_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "dcam_int: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

typedef void (*dcam_isr) (enum dcam_id idx, enum dcam_irq_id irq_id,
			void *param);

static dcam_isr_func s_user_func[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];
static void *s_user_data[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];

static struct{
	uint32_t irq;
	char *irq_name;
	int enable_log;
} s_irq_vect[] = {
	{DCAM_SN_SOF, "sensor_sof", 0},
	{DCAM_SN_EOF, "sensor_eof", 0},
	{DCAM_CAP_SOF, "cap_sof", 0},
	{DCAM_CAP_EOF, "cap_eof", 0},
	{DCAM_DCAM_OVF, "dcam_ovf", 1},
	{DCAM_PREVIEW_SOF, "preview_sof", 0},
	{DCAM_ISP_ENABLE_PULSE, "isp_enable_pulse", 0},
	{DCAM_FETCH_SOF_INT, "fetch_sof int", 0},
	{DCAM_AFL_LAST_SOF, "afl_last_sof", 0},
	{DCAM_BPC_MEM_ERR, "bpc_mem_err", 1},
	{DCAM_CAP_LINE_ERR, "cap_line_err", 1},
	{DCAM_CAP_FRM_ERR, "cap_frm_err", 1},
	{DCAM_FULL_PATH_END, "full_path_end", 0},
	{DCAM_BIN_PATH_END, "bin_path_end", 0},
	{DCAM_AEM_PATH_END, "aem_path_end", 0},
	{DCAM_PDAF_PATH_END, "pdaf_path_end", 0},
	{DCAM_VCH2_PATH_END, "vch2_path_end", 0},
	{DCAM_VCH3_PATH_END, "vch3_path_end", 0},
	{DCAM_FULL_PATH_TX_DONE, "full_path_done", 0},
	{DCAM_BIN_PATH_TX_DONE, "bin_path_done", 0},
	{DCAM_AEM_PATH_TX_DONE, "aem_path_done", 0},
	{DCAM_PDAF_PATH_TX_DONE, "pdaf_path_done", 0},
	{DCAM_VCH2_PATH_TX_DONE, "vch2_path_done", 0},
	{DCAM_VCH3_PATH_TX_DONE, "vch3_path_done", 0},
	{DCAM_BPC_MAP_DONE, "bpc_map_done", 0},
	{DCAM_BPC_POS_DONE, "bpc_pos_done", 0},
	{DCAM_AFM_INTREQ0, "afm_intreq0", 0},
	{DCAM_AFM_INTREQ1, "afm_intreq1", 0},
	{DCAM_AFL_TX_DONE, "afl_tx_done", 0},
	{DCAM_NR3_TX_DONE, "nr3_tx_done", 0},
	{DCAM_RESERVED, "reserved", 0},
	{DCAM_MMU_INT, "mmu_int", 0},
};

static void dcam_get_frame_mv(enum dcam_id idx,
	int *mv_x, int *mv_y)
{
	int out0 = 0;
	int out1 = 0;
	signed char temp_mv_x = 0;
	signed char temp_mv_y = 0;
	uint32_t param1 = 0;
	uint32_t fast_me_done = 0;
	uint32_t ping_pang_en = 0;

	param1 = DCAM_REG_RD(idx, DCAM_NR3_PARA1);
	ping_pang_en = param1 & BIT_1;

	if (ping_pang_en == 1) {
		out1 = DCAM_REG_RD(idx, DCAM_NR3_OUT1);
		out0 = DCAM_REG_RD(idx, DCAM_NR3_OUT0);

		fast_me_done = out1 & BIT_0;
		if (fast_me_done == 0) {
			temp_mv_x = (out0 >> 8) & 0xFF;
			temp_mv_y = out0 & 0xFF;
			*mv_x = temp_mv_x;
			*mv_y = temp_mv_y;
		} else {
			temp_mv_x = (out0 >> 24) & 0xFF;
			temp_mv_y = (out0 >> 16) & 0xFF;
			*mv_x = temp_mv_x;
			*mv_y = temp_mv_y;
		}
		pr_debug("3DNR mv x=%d,y=%d,out0=0x%x,out1=0x%x\n",
			*mv_x, *mv_y, out0, out1);
	} else if (ping_pang_en == 0) {
		out0 = DCAM_REG_RD(idx, DCAM_NR3_OUT0);
		temp_mv_x  = (out0 >> 8) & 0xFF;
		temp_mv_y = out0 & 0xFF;
		*mv_x = temp_mv_x;
		*mv_y = temp_mv_y;
		pr_debug("3DNR mv x=%d,y=%d,out0=0x%x\n",
			*mv_x, *mv_y, out0);
	} else {
		pr_err("3DNR fail to frame_mv ping_pang error\n");
	}
}

static void dcam_fast_me_store_frame(struct dcam_module *dcam_dev,
	int irq_id, struct camera_frame *frame)
{
	if (dcam_dev == NULL || frame == NULL) {
		pr_err("fail to fast_me_store_frame\n");
		return;
	}

	if (irq_id == DCAM_BIN_PATH_TX_DONE)
		memcpy(&(dcam_dev->fast_me.bin_frame),
			frame, sizeof(struct camera_frame));
	else
		memcpy(&(dcam_dev->fast_me.full_frame),
			frame, sizeof(struct camera_frame));
}

static void dcam_default_irq(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	dcam_isr_func user_func = s_user_func[idx][irq_id];
	void *data = s_user_data[idx][irq_id];
	struct dcam_module *module = (struct dcam_module *)param;

	if (DCAM_ADDR_INVALID(module)) {
		pr_err("fail to get valid input ptr dcam%d int %s\n",
			idx, s_irq_vect[irq_id].irq_name);
		return;
	}
	if (s_irq_vect[irq_id].enable_log)
		pr_info("DCAM%d: int %s\n", idx, s_irq_vect[irq_id].irq_name);

	if (user_func)
		(*user_func) (NULL, data);
}

void dcam_full_path_sof(enum dcam_id idx)
{
	int copy_flag = 0;
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);

	if (full_path->status == DCAM_ST_START) {
		if (full_path->valid == 0) {
			pr_info("DCAM%d: fail to get valid full_path\n", idx);
			return;
		}

		if (full_path->sof_cnt == 1 + full_path->frame_deci)
			full_path->sof_cnt = 0;

		full_path->sof_cnt++;
		if (full_path->sof_cnt == 1) {
			DCAM_TRACE("DCAM%d: full_path sof %d frame_deci %d\n",
				idx, full_path->sof_cnt, full_path->frame_deci);
		} else {
			DCAM_TRACE("DCAM%d: full_path invalid sof, cnt %d\n",
				idx, full_path->sof_cnt);
			return;
		}

		rtn = dcam_full_path_set_next_frm(idx);

		copy_flag |= FULL_COPY;
		if (full_path->output_format != DCAM_RAWRGB)
			copy_flag |= BIN_COPY;

		if (rtn) {
			full_path->need_wait = 1;
			pr_info("DCAM%d: full_path\n", idx);
			return;
		}
	}
}

static void dacm_full_path_done_notice(enum dcam_id idx)
{
	struct dcam_path_desc *p_full_path = get_dcam_full_path(idx);

	if (DCAM_ADDR_INVALID(p_full_path)) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	DCAM_TRACE("DCAM%d: path done notice %d, %d\n", idx,
		p_full_path->wait_for_done, p_full_path->tx_done_com.done);
	if (p_full_path->wait_for_done) {
		complete(&p_full_path->tx_done_com);
		pr_info("release tx_done_com: %d\n",
			p_full_path->tx_done_com.done);
		p_full_path->wait_for_done = 0;
	}
}

void dcam_full_path_done(enum dcam_id idx,
		enum dcam_irq_id irq_id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);

	if (DCAM_ADDR_INVALID(full_path))
		return;

	if (full_path->valid == 0) {
		pr_info("DCAM%d:fail to get valid full_path\n", idx);
		return;
	}

	if (full_path->frame_deci > 1) {
		if (full_path->done_cnt == 0) {
			full_path->done_cnt = 1;
		} else {
			full_path->done_cnt = 0;
			DCAM_TRACE("DCAM%d:fail to done dummy, drop\n",
					idx);
			return;
		}
	} else {
		full_path->done_cnt = 1;
	}

	pr_debug("dcam%d\n", idx);

	if (full_path->need_stop) {
		sprd_dcam_glb_reg_awr(idx, DCAM_CFG, ~BIT_1,
					DCAM_CFG_REG);
		full_path->need_stop = 0;
	}
	dacm_full_path_done_notice(idx);

	if (full_path->need_wait) {
		full_path->need_wait = 0;
	} else {
		struct camera_frame frame;

		rtn = sprd_cam_frm_dequeue(&full_path->frame_queue, &frame);
		if (rtn)
			return;
		if (frame.buf_info.dev == NULL)
			pr_info("DCAM%d:fail to done dev NULL %p\n",
				idx, frame.buf_info.dev);
		/*cam_buf_unmap_addr(&frame.buf_info);*/
		if (!cam_buf_is_equal(&frame.buf_info,
			&full_path->reserved_frame.buf_info)) {

			frame.width = full_path->output_size.w;
			frame.height = full_path->output_size.h;
			frame.irq_type = CAMERA_IRQ_IMG;

			pr_debug("DCAM%d: full_path frame %p\n",
				idx, &frame);
			DCAM_TRACE("y uv, 0x%x 0x%x, mfd 0x%x,0x%x,iova 0x%x\n",
				frame.yaddr, frame.uaddr,
				frame.buf_info.mfd[0],
				frame.buf_info.mfd[1],
				(uint32_t)frame.buf_info.iova[0] + frame.yaddr);

			dcam_dev = (struct dcam_module *)param;
			if (dcam_dev->need_nr3) {
				uint32_t mv_ready_cnt =
					dcam_dev->fast_me.mv_ready_cnt;
				if (mv_ready_cnt == 0) {
					dcam_dev->fast_me.full_frame_cnt++;
					dcam_fast_me_store_frame(dcam_dev,
						DCAM_FULL_PATH_TX_DONE, &frame);
				} else if (mv_ready_cnt == 1) {
					frame.mv.mv_x = dcam_dev->fast_me.mv_x;
					frame.mv.mv_y = dcam_dev->fast_me.mv_x;
					sprd_dcam_isr_proc(idx, irq_id, &frame);
					dcam_dev->fast_me.full_frame_cnt = 0;
					dcam_dev->fast_me.mv_ready_cnt = 0;
				} else {
					dcam_dev->fast_me.mv_ready_cnt = 0;
					dcam_dev->fast_me.full_frame_cnt = 0;
					pr_err("DCAM%d:fail to full mv_cnt err\n",
						idx);
					return;
				}
			} else {
				pr_debug("DCAM%d: not support nr3\n", idx);
				sprd_dcam_isr_proc(idx, irq_id, &frame);
			}
		} else {
			DCAM_TRACE("DCAM%d: use reserved full_path\n", idx);
		}
	}
}

void dcam_bin_path_sof(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *bin_path = get_dcam_bin_path(idx);

	if (bin_path->status == DCAM_ST_START) {
		if (bin_path->valid == 0) {
			pr_info("DCAM%d: fail to get valid bin_path\n", idx);
			return;
		}

		if (bin_path->sof_cnt == 1 + bin_path->frame_deci)
			bin_path->sof_cnt = 0;

		bin_path->sof_cnt++;
		if (bin_path->sof_cnt == 1) {
			DCAM_TRACE("DCAM%d: bin_path sof %d frame_deci %d\n",
				idx, bin_path->sof_cnt, bin_path->frame_deci);
		} else {
			DCAM_TRACE("DCAM%d: bin_path invalid sof, cnt %d\n",
				idx, bin_path->sof_cnt);
			return;
		}

		rtn = dcam_bin_path_set_next_frm(idx);

		if (rtn) {
			bin_path->need_wait = 1;
			pr_info("DCAM%d: bin_path\n", idx);
			return;
		}
	}
}

static void dcam_bin_path_done_notice(enum dcam_id idx)
{
	struct dcam_path_desc *p_bin_path = get_dcam_bin_path(idx);

	if (DCAM_ADDR_INVALID(p_bin_path)) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	DCAM_TRACE("DCAM%d: path done notice %d, %d\n", idx,
		   p_bin_path->wait_for_done, p_bin_path->tx_done_com.done);
	if (p_bin_path->wait_for_done) {
		complete(&p_bin_path->tx_done_com);
		pr_info("release tx_done_com: %d\n",
			p_bin_path->tx_done_com.done);
		p_bin_path->wait_for_done = 0;
	}
}

void dcam_bin_path_done(enum dcam_id idx, enum dcam_irq_id irq_id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct dcam_path_desc *bin_path = get_dcam_bin_path(idx);

	if (DCAM_ADDR_INVALID(bin_path))
		return;

	if (bin_path->valid == 0) {
		pr_info("DCAM%d:fail to get valid bin_path\n", idx);
		return;
	}

	if (bin_path->frame_deci > 1) {
		if (bin_path->done_cnt == 0) {
			bin_path->done_cnt = 1;
		} else {
			bin_path->done_cnt = 0;
			DCAM_TRACE("DCAM%d:fail to done bin_path dummy, drop\n",
					idx);
			return;
		}
	} else {
		bin_path->done_cnt = 1;
	}

	pr_debug("dcam%d\n", idx);

	dcam_bin_path_done_notice(idx);
	if (bin_path->need_stop) {
		sprd_dcam_glb_reg_awr(idx, DCAM_CFG, ~BIT_2,
					DCAM_CFG_REG);
		bin_path->need_stop = 0;
	}

	if (bin_path->need_wait) {
		bin_path->need_wait = 0;
	} else {
		struct camera_frame frame;

		rtn = sprd_cam_frm_dequeue(&bin_path->frame_queue, &frame);
		if (rtn)
			return;
		if (frame.buf_info.dev == NULL)
			pr_info("DCAM%d:fail to done dev NULL %p\n",
				idx, frame.buf_info.dev);
		/*cam_buf_unmap_addr(&frame.buf_info);*/
		if (!cam_buf_is_equal(&frame.buf_info,
			&bin_path->reserved_frame.buf_info)) {
			frame.width = bin_path->output_size.w;
			frame.height = bin_path->output_size.h;
			frame.irq_type = CAMERA_IRQ_IMG;
			pr_debug("DCAM%d: bin_path frame %p\n",
				idx, &frame);
			DCAM_TRACE("y uv, 0x%x 0x%x, mfd 0x%x,0x%x,iova 0x%x\n",
				frame.yaddr, frame.uaddr,
				frame.buf_info.mfd[0],
				frame.buf_info.mfd[1],
				(uint32_t)frame.buf_info.iova[0] + frame.yaddr);

			dcam_dev = (struct dcam_module *)param;
			if (dcam_dev->need_nr3) {
				uint32_t mv_ready_cnt =
					dcam_dev->fast_me.mv_ready_cnt;

				if (mv_ready_cnt == 0) {
					dcam_dev->fast_me.bin_frame_cnt++;
					dcam_fast_me_store_frame(dcam_dev,
						DCAM_BIN_PATH_TX_DONE, &frame);
				} else if (mv_ready_cnt == 1) {
					frame.mv.mv_x = dcam_dev->fast_me.mv_x;
					frame.mv.mv_y = dcam_dev->fast_me.mv_x;
					sprd_dcam_isr_proc(idx, irq_id, &frame);
					dcam_dev->fast_me.bin_frame_cnt = 0;
					dcam_dev->fast_me.mv_ready_cnt = 0;
				} else {
					dcam_dev->fast_me.mv_ready_cnt = 0;
					dcam_dev->fast_me.bin_frame_cnt = 0;
					pr_err("DCAM%d:fail to 3DNR bin mv_cnt err\n",
						idx);
					return;
				}
			} else {
				pr_debug("DCAM%d: not support nr3\n", idx);
				sprd_dcam_isr_proc(idx, irq_id, &frame);
			}
		} else {
			DCAM_TRACE("DCAM%d: use reserved bin_path\n", idx);
		}
	}
}

int sprd_cam_set_next_statis_buf(int idx, struct cam_statis_module *module,
			enum isp_3a_block_id block_index)
{
	int rtn = 0;
	int use_reserve_buf = 0;
	uint32_t statis_addr = 0;
	struct cam_frm_queue *statis_heap = NULL;
	struct cam_buf_queue *p_buf_queue = NULL;
	struct cam_statis_buf *reserved_buf = NULL;
	struct cam_statis_buf node;
	struct dcam_module *module_dev = NULL;

	module_dev = container_of(module, struct dcam_module,
				  statis_module_info);

	if (module_dev == NULL) {
		pr_err("fail to get module dev\n");
		return rtn;
	}

	memset(&node, 0x00, sizeof(node));
	if (block_index == ISP_AEM_BLOCK) {
		p_buf_queue = &module->aem_statis_queue;
		statis_heap = &module->aem_statis_frm_queue;
		reserved_buf = &module->aem_buf_reserved;
	} else if (block_index == ISP_AFL_BLOCK) {
		p_buf_queue = &module->afl_statis_queue;
		statis_heap = &module->afl_statis_frm_queue;
		reserved_buf = &module->afl_buf_reserved;
	} else if (block_index == ISP_PDAF_BLOCK) {
		p_buf_queue = &module->pdaf_statis_queue;
		statis_heap = &module->pdaf_statis_frm_queue;
		reserved_buf = &module->pdaf_buf_reserved;
	} else if (block_index == ISP_AFM_BLOCK) {
		p_buf_queue = &module->afm_statis_queue;
		statis_heap = &module->afm_statis_frm_queue;
		reserved_buf = &module->afm_buf_reserved;
	} else if (block_index == ISP_NR3_BLOCK) {
		p_buf_queue = &module->nr3_statis_queue;
		statis_heap = &module->nr3_statis_frm_queue;
		reserved_buf = &module->nr3_buf_reserved;
	}

	/*read buf addr from in_buf_queue*/
	if (sprd_cam_buf_queue_read(p_buf_queue, &node) != 0) {
		DCAM_TRACE("NO type %d free statis buf\n", block_index);
		/*use reserved buffer*/
		if (!cam_buf_is_valid(&reserved_buf->buf_info))
			pr_info("NO need to cfg type %d statis buf\n",
				block_index);

		memcpy(&node, reserved_buf, sizeof(struct cam_statis_buf));
		use_reserve_buf = 1;
	}

	if (node.buf_info.dev == NULL)
		pr_info("dev is NULL.\n");

	node.frame_id = module_dev->frame_id;

	/*call iommu func*/
	statis_addr = node.buf_info.iova[0];
	/*enqueue the statis buf into the array*/
	rtn = sprd_cam_frm_enqueue(statis_heap, &node);
	if (rtn) {
		pr_err("fail to enqueue statis buf\n");
		return rtn;
	}
	/*update buf addr to isp ddr addr*/
	if (block_index == ISP_AEM_BLOCK) {
		DCAM_REG_WR(idx, DCAM_AEM_BASE_WADDR, node.phy_addr);
	} else if (block_index == ISP_AFL_BLOCK) {
		/* TBD should modify buf size for afl */
		/* afl old*/

		/* afl new */
		DCAM_REG_WR(idx,
			ISP_ANTI_FLICKER_GLB_WADDR, node.phy_addr);
		DCAM_REG_WR(idx, ISP_ANTI_FLICKER_REGION_WADDR,
				(node.phy_addr + node.buf_size / 2));

		DCAM_REG_MWR(idx, ISP_ANTI_FLICKER_NEW_CFG_READY, BIT_0, 1);
	} else if (block_index == ISP_AFM_BLOCK) {
		DCAM_REG_WR(idx, ISP_RAW_AFM_ADDR, node.phy_addr);
	} else if (block_index == ISP_NR3_BLOCK) {
		DCAM_REG_WR(idx, DCAM_NR3_BASE_WADDR, node.phy_addr);
	} else if (block_index == ISP_PDAF_BLOCK) {
		DCAM_REG_WR(idx, DCAM_PDAF_BASE_WADDR, node.phy_addr);
		DCAM_REG_WR(idx, DCAM_VCH2_BASE_WADDR,
				node.phy_addr + node.buf_size / 2);
	}

	return rtn;
}

void dcam_aem_start(enum dcam_id idx, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;

	rtn = sprd_cam_set_next_statis_buf(idx, module, ISP_AEM_BLOCK);
	if (rtn)
		pr_err("fail to set AEM next statis buf\n");
}

static void dcam_aem_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_frm_queue *statis_heap = NULL;
	struct cam_statis_buf node;
	struct camera_frame frame_info;
	struct cam_statis_module *module = NULL;

	memset(&frame_info, 0x00, sizeof(frame_info));
	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;
	statis_heap = &module->aem_statis_frm_queue;

	/*dequeue the statis buf from a array*/
	rtn = sprd_cam_frm_dequeue(statis_heap, &node);
	if (rtn) {
		pr_err("ISP%d:fail to dequeue AEM buf error\n", idx);
		return;
	}

	if (node.mfd != module->aem_buf_reserved.mfd ||
		node.phy_addr != module->aem_buf_reserved.phy_addr) {
		frame_info.buf_size = node.buf_size;
		memcpy(frame_info.buf_info.mfd, node.buf_info.mfd,
		sizeof(uint32_t) * 3);
		frame_info.phy_addr = node.phy_addr;
		frame_info.vir_addr = node.vir_addr;
		frame_info.addr_offset = node.addr_offset;
		frame_info.irq_type = CAMERA_IRQ_STATIS;
		frame_info.irq_property = IRQ_AEM_STATIS;
		frame_info.frame_id = node.frame_id;

		/*call_back func to write the buf addr to usr_buf_queue*/
		sprd_dcam_isr_proc(idx, DCAM_AEM_PATH_TX_DONE, &frame_info);
	}
	module->aem_status = 0;
}

void dcam_afl_start(enum dcam_id idx, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;

	rtn = sprd_cam_set_next_statis_buf(idx, module, ISP_AFL_BLOCK);
	if (rtn)
		pr_err("fail to set AFL next statis buf\n");
}

static void dcam_pdaf_vch2_start(enum dcam_id idx, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;

	rtn = sprd_cam_set_next_statis_buf(idx, module, ISP_PDAF_BLOCK);
	if (rtn)
		pr_err("fail to set pdaf next statis buf\n");
}

static void dcam_afl_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_frm_queue *statis_heap = NULL;
	struct cam_statis_buf node;
	struct camera_frame frame_info;
	struct cam_statis_module *module = NULL;

	memset(&frame_info, 0x00, sizeof(frame_info));

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;
	statis_heap = &module->afl_statis_frm_queue;

	/*dequeue the statis buf from a array*/
	rtn = sprd_cam_frm_dequeue(statis_heap, &node);
	if (rtn) {
		pr_err("ISP%d: fail to dequeue AFL buf error\n", idx);
		return;
	}

	if (node.mfd != module->afl_buf_reserved.mfd ||
		node.phy_addr != module->afl_buf_reserved.phy_addr) {
		frame_info.buf_size = node.buf_size;
		memcpy(frame_info.buf_info.mfd, node.buf_info.mfd,
		sizeof(uint32_t) * 3);
		frame_info.phy_addr = node.phy_addr;
		frame_info.vir_addr = node.vir_addr;
		frame_info.addr_offset = node.addr_offset;
		frame_info.irq_type = CAMERA_IRQ_STATIS;
		frame_info.irq_property = IRQ_AFL_STATIS;
		frame_info.frame_id = node.frame_id;

		/*call_back func to write the buf addr to usr_buf_queue*/
		sprd_dcam_isr_proc(idx, DCAM_AFL_TX_DONE, &frame_info);
	}
	dcam_afl_start(idx, param);
}

void dcam_afm_start(enum dcam_id idx, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;

	rtn = sprd_cam_set_next_statis_buf(idx, module, ISP_AFM_BLOCK);
	if (rtn)
		pr_err("fail to set AFM next statis buf\n");
}

static void dcam_afm_done(enum dcam_id idx, enum dcam_irq_id irq_id,
			void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_frm_queue *statis_heap = NULL;
	struct cam_statis_buf node;
	struct camera_frame frame_info;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;
	statis_heap = &module->afm_statis_frm_queue;

	memset(&node, 0x00, sizeof(node));
	memset(&frame_info, 0x00, sizeof(frame_info));
	/*dequeue the statis buf from a array*/
	rtn = sprd_cam_frm_dequeue(statis_heap, &node);
	if (rtn) {
		pr_err("ISP%d:fail to dequeue AFM statis buf\n", idx);
		return;
	}

	if (node.mfd != module->aem_buf_reserved.mfd ||
		node.phy_addr != module->afm_buf_reserved.phy_addr) {
		frame_info.buf_size = node.buf_size;
		memcpy(frame_info.buf_info.mfd, node.buf_info.mfd,
		sizeof(uint32_t) * 3);
		frame_info.phy_addr = node.phy_addr;
		frame_info.vir_addr = node.vir_addr;
		frame_info.addr_offset = node.addr_offset;
		frame_info.irq_type = CAMERA_IRQ_STATIS;
		frame_info.irq_property = IRQ_AFM_STATIS;
		frame_info.frame_id = node.frame_id;

		/*call_back func to write the buf addr to usr_buf_queue*/
		sprd_dcam_isr_proc(idx, DCAM_AFM_INTREQ1, &frame_info);
	}
	if (idx == 0)
		dcam_afm_start(idx, param);
}

void dcam_nr3_start(enum dcam_id idx, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;

	rtn = sprd_cam_set_next_statis_buf(idx, module, ISP_NR3_BLOCK);
	if (rtn)
		pr_err("fail to set Binning next statis buf\n");
}

static void dcam_nr3_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	int x = 0;
	int y = 0;
	struct dcam_module *dcam_dev = NULL;

	dcam_dev = (struct dcam_module *)param;

	if (!dcam_dev->need_nr3)
		return;

	dcam_dev->fast_me.mv_ready_cnt++;
	if (dcam_dev->fast_me.mv_ready_cnt > 1) {
		dcam_dev->fast_me.mv_ready_cnt = 1;
		pr_debug("3DNR ME nr3 done ready_cnt exceed\n");
	}

	dcam_get_frame_mv(idx, &x, &y);
	dcam_dev->fast_me.mv_x = x;
	dcam_dev->fast_me.mv_y = y;

	if (dcam_dev->fast_me.bin_frame_cnt > 0) {
		sprd_dcam_isr_proc(idx, DCAM_BIN_PATH_TX_DONE,
			&(dcam_dev->fast_me.bin_frame));
		dcam_dev->fast_me.mv_ready_cnt = 0;
		dcam_dev->fast_me.bin_frame_cnt = 0;
	}

	if (dcam_dev->fast_me.full_frame_cnt > 0) {
		sprd_dcam_isr_proc(idx, DCAM_FULL_PATH_TX_DONE,
			&(dcam_dev->fast_me.full_frame));
		dcam_dev->fast_me.mv_ready_cnt = 0;
		dcam_dev->fast_me.full_frame_cnt = 0;
	}
}

void dcam_pdaf_start(enum dcam_id idx, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_statis_module *module = NULL;

	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;

	rtn = sprd_cam_set_next_statis_buf(idx, module, ISP_PDAF_BLOCK);
	if (rtn)
		pr_err("fail to set pdaf next statis buf\n");
}


static void dcam_type3_left_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_frm_queue *statis_heap = NULL;
	struct cam_statis_buf node;
	struct camera_frame frame_info;
	struct cam_statis_module *module = NULL;

	memset(&frame_info, 0x00, sizeof(frame_info));
	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;
	statis_heap = &module->pdaf_statis_frm_queue;

	/*dequeue the statis buf from a array*/
	rtn = sprd_cam_frm_dequeue(statis_heap, &node);
	if (rtn) {
		pr_err("ISP%d:fail to dequeue PDAF buf error\n", idx);
		return;
	}

	if (node.mfd != module->pdaf_buf_reserved.mfd
		 || node.phy_addr != module->pdaf_buf_reserved.phy_addr) {
		frame_info.buf_size = node.buf_size;
		memcpy(frame_info.buf_info.mfd, node.buf_info.mfd,
		sizeof(uint32_t) * 3);
		frame_info.phy_addr = node.phy_addr;
		frame_info.vir_addr = node.vir_addr;
		frame_info.addr_offset = node.addr_offset;
		frame_info.irq_type = CAMERA_IRQ_STATIS;
		frame_info.irq_property = IRQ_PDAF_STATIS;
		frame_info.frame_id = node.frame_id;

		/*call_back func to write the buf addr to usr_buf_queue*/
		sprd_dcam_isr_proc(idx, DCAM_PDAF_PATH_TX_DONE, &frame_info);
	}

	dcam_pdaf_start(idx, param);
}

static void dcam_pdaf_vch2_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_module *dcam_dev = NULL;
	struct cam_frm_queue *statis_heap = NULL;
	struct cam_statis_buf node;
	struct camera_frame frame_info;
	struct cam_statis_module *module = NULL;

	memset(&frame_info, 0x00, sizeof(frame_info));
	dcam_dev = (struct dcam_module *)param;
	module = &dcam_dev->statis_module_info;
	statis_heap = &module->pdaf_statis_frm_queue;

	/*dequeue the statis buf from a array*/
	rtn = sprd_cam_frm_dequeue(statis_heap, &node);
	if (rtn) {
		pr_err("ISP%d:fail to dequeue PDAF buf error\n", idx);
		return;
	}

	if (node.mfd != module->pdaf_buf_reserved.mfd
		 || node.phy_addr != module->pdaf_buf_reserved.phy_addr) {
		frame_info.buf_size = node.buf_size;
		memcpy(frame_info.buf_info.mfd, node.buf_info.mfd,
		sizeof(uint32_t) * 3);
		frame_info.phy_addr = node.phy_addr;
		frame_info.vir_addr = node.vir_addr;
		frame_info.addr_offset = node.addr_offset;
		frame_info.irq_type = CAMERA_IRQ_STATIS;
		frame_info.irq_property = IRQ_PDAF_STATIS;
		frame_info.frame_id = node.frame_id;

		/*call_back func to write the buf addr to usr_buf_queue*/
		sprd_dcam_isr_proc(idx, DCAM_PDAF_PATH_TX_DONE, &frame_info);
	}

	dcam_pdaf_vch2_start(idx, param);
}

static void dcam_pdaf_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	dcam_type3_left_done(idx, irq_id, param);
}

static void dcam_vch2_done(enum dcam_id idx, enum dcam_irq_id irq_id,
		void *param)
{
	dcam_pdaf_vch2_done(idx, irq_id, param);
}

static void dcam_cap_sof(enum dcam_id idx, enum dcam_irq_id irq_id, void *param)
{
	dcam_isr_func user_func;
	void *data;
	struct dcam_module *module = (struct dcam_module *)param;
	struct camera_frame frame;

	if (DCAM_ADDR_INVALID(module))
		return;

	user_func = s_user_func[idx][irq_id];
	data = s_user_data[idx][irq_id];

	memset(&frame, 0x0, sizeof(frame));
	frame.frame_id = module->frame_id++;

	dcam_full_path_sof(idx);
	dcam_bin_path_sof(idx);

	if (!module->statis_module_info.aem_status) {
		dcam_aem_start(idx, param);
		module->statis_module_info.aem_status = 1;
	}

#if 0
	dcam_afl_start(idx, param);
	dcam_nr3_start(idx, param);
#endif

	dcam_auto_copy(idx, ALL_COPY);
	if (user_func)
		(*user_func) (&frame, data);
}

static const dcam_isr isr_list[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER] = {
	[0][DCAM_SN_SOF]		= dcam_default_irq,
	[0][DCAM_SN_EOF]		= dcam_default_irq,
	[0][DCAM_CAP_SOF]		= dcam_cap_sof,
	[0][DCAM_CAP_EOF]		= dcam_default_irq,
	[0][DCAM_DCAM_OVF]		= dcam_default_irq,
	[0][DCAM_BPC_MEM_ERR]		= dcam_default_irq,
	[0][DCAM_CAP_LINE_ERR]		= dcam_default_irq,
	[0][DCAM_CAP_FRM_ERR]		= dcam_default_irq,
	[0][DCAM_FULL_PATH_TX_DONE]	= dcam_full_path_done,
	[0][DCAM_BIN_PATH_TX_DONE]	= dcam_bin_path_done,
	[0][DCAM_AEM_PATH_TX_DONE]	= dcam_aem_done,
	[0][DCAM_PDAF_PATH_TX_DONE]	= dcam_pdaf_done,
	[0][DCAM_VCH2_PATH_TX_DONE]	= dcam_vch2_done,
	[0][DCAM_VCH3_PATH_TX_DONE]	= dcam_default_irq,
	[0][DCAM_AFM_INTREQ1]		= dcam_afm_done,
	[0][DCAM_AFL_TX_DONE]		= dcam_afl_done,
	[0][DCAM_NR3_TX_DONE]		= dcam_nr3_done,

	[1][DCAM_SN_SOF]		= dcam_default_irq,
	[1][DCAM_SN_EOF]		= dcam_default_irq,
	[1][DCAM_CAP_SOF]		= dcam_cap_sof,
	[1][DCAM_CAP_EOF]		= dcam_default_irq,
	[1][DCAM_DCAM_OVF]		= dcam_default_irq,
	[1][DCAM_BPC_MEM_ERR]		= dcam_default_irq,
	[1][DCAM_CAP_LINE_ERR]		= dcam_default_irq,
	[1][DCAM_CAP_FRM_ERR]		= dcam_default_irq,
	[1][DCAM_FULL_PATH_TX_DONE]	= dcam_full_path_done,
	[1][DCAM_BIN_PATH_TX_DONE]	= dcam_bin_path_done,
	[1][DCAM_AEM_PATH_TX_DONE]	= dcam_aem_done,
	[1][DCAM_AFM_INTREQ1]		= dcam_afm_done,
	[1][DCAM_AFL_TX_DONE]		= dcam_afl_done,
	[1][DCAM_NR3_TX_DONE]		= dcam_nr3_done,

	[2][DCAM_SN_SOF]		= dcam_default_irq,
	[2][DCAM_SN_EOF]		= dcam_default_irq,
	[2][DCAM_DCAM_OVF]		= dcam_default_irq,
	[2][DCAM_CAP_LINE_ERR]		= dcam_default_irq,
	[2][DCAM_CAP_FRM_ERR]		= dcam_default_irq,
	[2][DCAM_FULL_PATH_TX_DONE]	= dcam_full_path_done,
};

static int _dcam_err_pre_proc(enum dcam_id idx, uint32_t irq_status)
{
	if (get_dcam_module(idx)) {
		DCAM_TRACE("DCAM%d: state in err_pre_proc 0x%x\n", idx,
			get_dcam_module(idx)->state);
		if (get_dcam_module(idx)->state & DCAM_STATE_QUICKQUIT)
			return -1;

		get_dcam_module(idx)->err_happened = 1;
	}
	pr_info("DCAM%d: err, 0x%x\n", idx, irq_status);

	isp_reg_trace((int)idx);
	dcam_reg_trace(idx);

	sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_0, 0, DCAM_CFG_REG);
	sprd_dcam_stop(idx, 1);

	return 0;
}

static irqreturn_t dcam_isr_root(int irq, void *priv)
{
	int i = 0;
	uint32_t irq_line = 0, status = 0, vect = 0;
	unsigned long flag = 0;
	enum dcam_id idx = DCAM_ID_0;
	int irq_numbers = ARRAY_SIZE(s_irq_vect);

	if (s_dcam_irq[DCAM_ID_0] == irq)
		idx = DCAM_ID_0;
	else if (s_dcam_irq[DCAM_ID_1] == irq)
		idx = DCAM_ID_1;
	else if (s_dcam_irq[DCAM_ID_2] == irq)
		idx = DCAM_ID_2;
	else
		return IRQ_NONE;

	status = DCAM_REG_RD(idx, DCAM_INT_MASK) & DCAM_IRQ_LINE_MASK;
	if (unlikely(status == 0))
		return IRQ_NONE;
	DCAM_REG_WR(idx, DCAM_INT_CLR, status);

	irq_line = status;
	if (unlikely(DCAM_IRQ_ERR_MASK & status))
		if (_dcam_err_pre_proc(idx, status))
			return IRQ_HANDLED;

	spin_lock_irqsave(&dcam_lock[idx], flag);

	for (i = 0; i < irq_numbers; i++) {
		vect = s_irq_vect[i].irq;
		if (irq_line & (1 << (uint32_t)vect)) {
			if (isr_list[idx][vect])
				isr_list[idx][vect](idx, i, priv);
		}
		irq_line &= ~(uint32_t)(1 << (uint32_t)vect);
		if (!irq_line)
			break;
	}

	spin_unlock_irqrestore(&dcam_lock[idx], flag);

	return IRQ_HANDLED;
}

int dcam_irq_request(enum dcam_id idx, void *param)
{
	int ret = 0;

	ret = request_irq(s_dcam_irq[idx], dcam_isr_root,
			IRQF_SHARED, "DCAM", param);
	if (ret) {
		pr_err("fail to install IRQ %d\n", ret);
		return ret;
	}

	return ret;
}

void dcam_irq_free(enum dcam_id idx, void *param)
{
	free_irq(s_dcam_irq[idx], param);
}

int sprd_dcam_reg_isr(enum dcam_id idx, enum dcam_irq_id id,
		dcam_isr_func user_func, void *user_data)
{
	unsigned long flag = 0;
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	if (id >= DCAM_IRQ_NUMBER) {
		rtn = DCAM_RTN_ISR_ID_ERR;
	} else {
		spin_lock_irqsave(&dcam_lock[idx], flag);
		s_user_func[idx][id] = user_func;
		s_user_data[idx][id] = user_data;
		spin_unlock_irqrestore(&dcam_lock[idx], flag);
	}

	return -rtn;
}

int sprd_dcam_isr_proc(enum dcam_id idx, enum dcam_irq_id id,
				struct camera_frame *frame)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func user_func;
	void *user_data;

	if (id >= DCAM_IRQ_NUMBER) {
		rtn = DCAM_RTN_ISR_ID_ERR;
	} else {
		user_func = s_user_func[idx][id];
		user_data = s_user_data[idx][id];
		if (user_func)
			user_func(frame, user_data);
	}

	return -rtn;
}
