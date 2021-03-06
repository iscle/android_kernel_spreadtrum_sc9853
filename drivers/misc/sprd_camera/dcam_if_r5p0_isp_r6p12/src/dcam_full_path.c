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
#include <video/sprd_mm.h>

#include "dcam_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "full_path: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int dcam_full_path_init(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);

	memset(full_path, 0x00, sizeof(*full_path));
	full_path->id = CAMERA_FULL_PATH;
	init_completion(&full_path->tx_done_com);
	init_completion(&full_path->sof_com);
	rtn = sprd_cam_buf_queue_init(&full_path->buf_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct camera_frame),
		DCAM_BUF_QUEUE_LENGTH, "full path buf_queue");
	rtn |= sprd_cam_frm_queue_init(&full_path->frame_queue,
		CAM_QUEUE_RW_LOCK, sizeof(struct camera_frame),
		DCAM_FRM_QUEUE_LENGTH + 1, "full path frm_queue");
	DCAM_TRACE("path int end!\n");
	return rtn;
}

int set_dcam_full_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);
	struct camera_addr *p_addr;

	if (DCAM_ADDR_INVALID(full_path)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if ((unsigned long)(param) == 0) {
		pr_err("fail to get valid input ptr\n");
		return -DCAM_RTN_PARA_ERR;
	}

	switch (id) {
	case DCAM_PATH_SRC_SEL:
	{
		uint32_t *src = (uint32_t *)param;

		full_path->src_sel = *src;
		full_path->valid_param.src_sel = 1;
		break;
	}

	case DCAM_PATH_INPUT_SIZE:
	{
		struct camera_size *size = (struct camera_size *)param;

		memcpy((void *)&full_path->input_size, (void *)size,
			sizeof(struct camera_size));
		break;
	}

	case DCAM_PATH_INPUT_RECT:
	{
		struct camera_rect *rect = (struct camera_rect *)param;

		memcpy((void *)&full_path->input_rect, (void *)rect,
			sizeof(struct camera_rect));
	}

	case DCAM_PATH_OUTPUT_FORMAT:
	{
		uint32_t *fmt = (uint32_t *)param;

		full_path->output_format = *fmt;
		full_path->valid_param.output_format = 1;
		break;
	}

	case DCAM_PATH_OUTPUT_LOOSE:
	{
		uint32_t *fmt = (uint32_t *)param;

		full_path->is_loose = *fmt;
		break;
	}

	case DCAM_PATH_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;

		if (p_addr->type == CAM_BUF_USER_TYPE
			&& DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
					p_addr->uaddr,
					p_addr->vaddr) &&
					p_addr->mfd_y == 0) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else if (p_addr->type != CAM_BUF_USER_TYPE
			&& (p_addr->client[0] == NULL
			|| p_addr->handle[0] == NULL)) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			struct camera_frame frame;

			memset((void *)&frame, 0x00, sizeof(frame));
			frame.yaddr = p_addr->yaddr;
			frame.uaddr = p_addr->uaddr;
			frame.vaddr = p_addr->vaddr;
			frame.yaddr_vir = p_addr->yaddr_vir;
			frame.uaddr_vir = p_addr->uaddr_vir;
			frame.vaddr_vir = p_addr->vaddr_vir;

			frame.type = CAMERA_FULL_PATH;
			frame.fid = full_path->frame_base_id;

			frame.buf_info.mfd[0] = p_addr->mfd_y;
			frame.buf_info.mfd[1] = p_addr->mfd_u;
			frame.buf_info.mfd[2] = p_addr->mfd_v;
			frame.buf_info.dev = p_addr->p_dev;
			frame.buf_info.type = p_addr->type;
			frame.buf_info.client[0] = p_addr->client[0];
			frame.buf_info.handle[0] = p_addr->handle[0];
			frame.buf_info.state = p_addr->state;
			frame.buf_info.num = p_addr->num;
			if (!(p_addr->state & CAM_BUF_STATE_MAPPING_DCAM)) {
				/*may need update iommu here*/
				rtn = cam_buf_get_sg_table(&frame.buf_info);
				if (rtn) {
					pr_err("fail to cfg output addr!\n");
					rtn = DCAM_RTN_PATH_ADDR_ERR;
					break;
				}
				rtn = cam_buf_map_addr(&frame.buf_info);
			} else
				frame.buf_info.iova[0] = p_addr->iova[0];

			if (!sprd_cam_buf_queue_write(&full_path->buf_queue,
						&frame))
				full_path->output_frame_count++;
			if (full_path->ion_buf_cnt < DCAM_FRM_QUEUE_LENGTH) {
				memcpy(&full_path->ion_buffer
					[full_path->ion_buf_cnt], &frame,
					sizeof(struct camera_frame));
				full_path->ion_buf_cnt++;
			}

			DCAM_TRACE("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				p_addr->yaddr, p_addr->uaddr,
				p_addr->vaddr, frame.buf_info.mfd[0],
				frame.buf_info.mfd[1]);
		}
		break;

	case DCAM_PATH_OUTPUT_RESERVED_ADDR:
		p_addr = (struct camera_addr *)param;

		if (p_addr->type == CAM_BUF_USER_TYPE
			&& DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
					p_addr->uaddr,
					p_addr->vaddr) &&
					p_addr->mfd_y == 0) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else if (p_addr->type != CAM_BUF_USER_TYPE
			&& (p_addr->client[0] == NULL
			|| p_addr->handle[0] == NULL)) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			uint32_t output_frame_count = 0;
			struct camera_frame *frame = NULL;

			frame = &full_path->reserved_frame;
			output_frame_count = full_path->output_frame_count;

			memset((void *)frame, 0x00, sizeof(*frame));
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			frame->buf_info.mfd[0] = p_addr->mfd_y;
			frame->buf_info.mfd[1] = p_addr->mfd_u;
			frame->buf_info.mfd[2] = p_addr->mfd_v;
			frame->buf_info.dev = p_addr->p_dev;
			frame->buf_info.type = p_addr->type;
			frame->buf_info.client[0] = p_addr->client[0];
			frame->buf_info.handle[0] = p_addr->handle[0];
			frame->buf_info.num = p_addr->num;

			/*may need update iommu here*/
			rtn = cam_buf_get_sg_table(&frame->buf_info);
			if (rtn) {
				pr_err("fail to cfg reserved output addr!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}
			rtn = cam_buf_map_addr(&frame->buf_info);

			DCAM_TRACE("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				p_addr->yaddr, p_addr->uaddr,
				p_addr->vaddr, p_addr->mfd_y,
				p_addr->mfd_u);
		}
		break;

	case DCAM_PATH_FRAME_BASE_ID:
	{
		uint32_t base_id = *(uint32_t *)param;

		DCAM_TRACE("DCAM%d: set frame base id 0x%x\n",
			idx, base_id);
		full_path->frame_base_id = base_id;
		break;
	}
	case DCAM_PATH_DATA_ENDIAN:
	{
		struct camera_endian_sel *endian
			= (struct camera_endian_sel *)param;

		if (endian->y_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			full_path->data_endian.y_endian = endian->y_endian;
			full_path->valid_param.data_endian = 1;
		}
		break;
	}
	case DCAM_PATH_FRM_DECI:
	{
		uint32_t deci_factor = *(uint32_t *) param;

		if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
			rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
		} else {
			full_path->frame_deci = deci_factor;
			full_path->valid_param.frame_deci = 1;
		}
		break;
	}

	case DCAM_PATH_ENABLE:
		full_path->valid = *(uint32_t *)param;
		break;

	default:
		pr_err("fail to cfg dcam path\n");
		break;
	}

	return -rtn;
}

int dcam_full_path_set_next_frm(enum dcam_id idx)
{
	int use_reserve_frame = 0;
	uint32_t output_frame_count = 0;
	unsigned long addr[2];
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_frame frame;
	struct camera_frame *reserved_frame = NULL;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);
	struct dcam_module *module = get_dcam_module(idx);
	struct cam_frm_queue *p_heap = NULL;

	if (DCAM_ADDR_INVALID(full_path) || DCAM_ADDR_INVALID(module)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	memset((void *)&frame, 0x00, sizeof(frame));
	reserved_frame = &full_path->reserved_frame;
	p_heap = &full_path->frame_queue;
	output_frame_count = full_path->output_frame_count;

	if (sprd_cam_buf_queue_read(&full_path->buf_queue, &frame) == 0 &&
		cam_buf_is_valid(&frame.buf_info)) {
		full_path->output_frame_count--;
	} else {
		pr_info("DCAM%d: No free frame id %d\n",
			idx, frame.fid);
		if (!cam_buf_is_valid(&reserved_frame->buf_info)) {
			pr_info("DCAM%d: No need to cfg frame buffer", idx);
			return -1;
		}
		memcpy(&frame, reserved_frame, sizeof(struct camera_frame));
		use_reserve_frame = 1;
	}

	if (frame.buf_info.dev == NULL)
		pr_info("DCAM%d next dev NULL %p\n", idx, frame.buf_info.dev);
	/*rtn = cam_buf_map_addr(&frame.buf_info);*/
	if (rtn) {
		pr_err("fail to get path addr\n");
		return rtn;
	}
	addr[0] = frame.buf_info.iova[0] + frame.yaddr;
	addr[1] = frame.buf_info.iova[1] + frame.uaddr;

	DCAM_TRACE("DCAM%d: reserved %d iova[0]=0x%x mfd=0x%x\n",
		idx, use_reserve_frame, (int)addr[0],
		frame.buf_info.mfd[0]);

	DCAM_REG_WR(idx, DCAM_FULL_BASE_WADDR, addr[0]);
	if (full_path->output_format != DCAM_RAWRGB)
		DCAM_REG_WR(idx, DCAM_BIN_BASE_WADDR0, addr[1]);

	frame.frame_id = module->frame_id;

	if (sprd_cam_frm_enqueue(p_heap, &frame) == 0)
		DCAM_TRACE("success to enq frame buf\n");
	else
		rtn = DCAM_RTN_PATH_FRAME_LOCKED;

	return -rtn;
}

int dcam_start_full_path(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);
	uint32_t reg_val = 0;
	unsigned long addr = 0;
	struct camera_rect *rect = NULL;
	int copy_flag = FULL_COPY;
	int path_eb = BIT_1;

	if (DCAM_ADDR_INVALID(full_path)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (full_path->valid) {
		if (full_path->valid_param.data_endian) {
			sprd_dcam_glb_reg_mwr(idx, DCAM_PATH_ENDIAN,
				BIT_1 | BIT_0,
				full_path->data_endian.y_endian << 0,
				DCAM_ENDIAN_REG);
			DCAM_TRACE("data_endian y=0x%x\n",
				full_path->data_endian.y_endian);
		}

		if (full_path->valid_param.src_sel) {
			pr_info("sel source %d\n", full_path->src_sel);
			DCAM_REG_MWR(idx, DCAM_FULL_CFG,
				BIT_2, full_path->src_sel << 2);
		}

		if (full_path->input_size.w != full_path->input_rect.w
			|| full_path->input_size.h != full_path->input_rect.h) {
			pr_info("set crop_eb\n");

			DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_1, BIT_1);

			addr = DCAM_FULL_CROP_START;
			rect = &full_path->input_rect;
			reg_val = (rect->x & 0xFFFF) |
				((rect->y & 0xFFFF) << 16);
			DCAM_REG_WR(idx, addr, reg_val);

			addr = DCAM_FULL_CROP_SIZE;
			reg_val = ((rect->x + rect->w - 1) & 0xFFFF) |
				(((rect->y + rect->h - 1) & 0xFFFF) << 16);
			DCAM_REG_WR(idx, addr, reg_val);
			full_path->output_size.w = rect->w;
			full_path->output_size.h = rect->h;
		} else {
			full_path->output_size.w = full_path->input_size.w;
			full_path->output_size.h = full_path->input_size.h;
		}

		if (full_path->output_format == DCAM_RAWRGB)
			DCAM_REG_MWR(idx, DCAM_FULL_CFG,
				BIT_0, full_path->is_loose);

		rtn = dcam_full_path_set_next_frm(idx);
		if (rtn) {
			pr_err("fail to set next frame\n");
			return -(rtn);
		}

		if (full_path->output_format != DCAM_RAWRGB) {
			copy_flag |= BIN_COPY;
			path_eb |= BIT_2;
		}

		sprd_dcam_glb_reg_owr(idx, DCAM_CFG, path_eb, DCAM_CFG_REG);
		full_path->need_wait = 0;
		full_path->status = DCAM_ST_START;
		full_path->sof_cnt = 0;
		full_path->done_cnt = 0;
	}
	DCAM_TRACE("path start end!\n");
	return rtn;
}

void dcam_quickstop_full_path(enum dcam_id idx)
{
	uint32_t ret = 0;
	int time_out = 5000;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);

	if (DCAM_ADDR_INVALID(full_path)) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	if (full_path->valid == 0) {
		pr_info("DCAM%d: path is not valid\n", idx);
		return;
	}

	sprd_dcam_glb_reg_owr(idx, DCAM_PATH_STOP, BIT_0, DCAM_CONTROL_REG);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_1, ~BIT_1, DCAM_CFG_REG);
	dcam_force_copy(idx, FULL_COPY);
	udelay(1000);

	/* wait for AHB path busy cleared */
	while (time_out) {
		ret = DCAM_REG_RD(idx, DCAM_PATH_BUSY) & BIT_0;
		if (!ret)
			break;
		time_out--;
	}
	if (!time_out)
		pr_info("DCAM%d: stop path time out\n", idx);

	full_path->status = DCAM_ST_STOP;
	full_path->sof_cnt = 0;
	full_path->done_cnt = 0;
	/*full_path->valid = 0;*/
	DCAM_TRACE("path stop end!\n");
}

int dcam_full_path_deinit(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);
	struct camera_frame *frame = NULL;
	struct camera_frame *res_frame = NULL;
	uint32_t i = 0;

	if (DCAM_ADDR_INVALID(full_path)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	for (i = 0; i < DCAM_FRM_QUEUE_LENGTH; i++) {
		frame = &full_path->ion_buffer[i];
		cam_buf_unmap_addr(&frame->buf_info);
	}

	sprd_cam_frm_queue_deinit(&full_path->frame_queue);
	sprd_cam_buf_queue_deinit(&full_path->buf_queue);

	res_frame = &full_path->reserved_frame;
	if (cam_buf_is_valid(&res_frame->buf_info))
		cam_buf_unmap_addr(&res_frame->buf_info);
	memset((void *)res_frame, 0x00, sizeof(*res_frame));
	memset(full_path, 0x00, sizeof(*full_path));
	DCAM_TRACE("path deint end!\n");
	return rtn;
}

int dcam_full_path_clear(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *full_path = get_dcam_full_path(idx);
	struct camera_frame *frame = NULL;
	uint32_t i = 0;

	if (DCAM_ADDR_INVALID(full_path)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	sprd_cam_buf_queue_clear(&full_path->buf_queue);
	sprd_cam_frm_queue_clear(&full_path->frame_queue);

	for (i = 0; i < DCAM_FRM_QUEUE_LENGTH; i++) {
		frame = &full_path->ion_buffer[i];
		rtn = sprd_cam_buf_queue_write(&full_path->buf_queue, frame);
		if (rtn)
			pr_err("fail to write buffer queue\n");
	}

	return rtn;
}
