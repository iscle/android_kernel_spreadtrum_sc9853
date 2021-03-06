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

#ifdef FEATRUE_DCAM_IOCTRL

typedef int(*dcam_io_fun) (struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg);

struct dcam_io_ctrl_fun {
	uint32_t cmd;
	dcam_io_fun io_ctrl;
};

static int sprd_deinit_handle(struct camera_dev *dev);

static int sprd_camera_stream_off(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg);

static int sprd_img_get_cam_dev(struct camera_file *pcamerafile,
	struct camera_dev **ppdev, struct camera_info **ppinfo)
{
	int ret = 0, idx = 0;
	struct camera_group *group = NULL;

	if (!pcamerafile) {
		ret = -EFAULT;
		pr_err("fail to get camerafile\n");
		goto exit;
	}
	group = pcamerafile->grp;
	if (!group) {
		ret = -EFAULT;
		pr_err("fail to get group\n");
		goto exit;
	}

	idx = pcamerafile->idx;
	if (unlikely(idx < 0 || idx >= DCAM_ID_MAX)) {
		pr_err("cam idx=%d, error\n", idx);
		ret = -EFAULT;
		goto exit;
	}

	if (group->dev_inited & (1 << (int)idx)) {
		*ppdev = group->dev[idx];
		if (!(*ppdev)) {
			ret = -EFAULT;
			pr_err("fail to get cam dev[%d]\n", idx);
			goto exit;
		}
		*ppinfo = &(*ppdev)->cam_ctx;
	} else {
		ret = -EFAULT;
		pr_err("fail to get cam dev[%d] and info\n", idx);
		goto exit;
	}

exit:
	return ret;
}

static int channel2path_idx(uint32_t channel_id)
{
	int path_index = 0;

	switch (channel_id) {
	case CAMERA_PRE_PATH:
		path_index = ISP_PATH_IDX_PRE;
		break;
	case CAMERA_VID_PATH:
		path_index = ISP_PATH_IDX_VID;
		break;
	case CAMERA_CAP_PATH:
		path_index = ISP_PATH_IDX_CAP;
		break;
	default:
		path_index = ISP_PATH_IDX_ALL;
		pr_info("fail to get path index, channel %d\n", channel_id);
	}

	return path_index;
}

static int sprd_img_tx_sof(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	if (param == NULL || dev == NULL ||
		atomic_read(&dev->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	memset((void *)&node, 0, sizeof(node));
	node.irq_flag = IMG_TX_DONE;
	node.irq_type = CAMERA_IRQ_DONE;
	node.irq_property = IRQ_DCAM_SOF;
	node.frame_id = frame->frame_id;

	ret = sprd_cam_buf_queue_write(&dev->queue, &node);
	if (ret) {
		pr_err("fail to write to queue\n");
		return ret;
	}

	complete(&dev->irq_com);
	pr_debug("tx sof\n");
	return ret;
}

static int sprd_img_tx_error(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	if (param == NULL || dev == NULL ||
		atomic_read(&dev->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	memset((void *)&node, 0, sizeof(node));
	atomic_set(&dev->run_flag, 1);
	node.irq_flag = IMG_TX_ERR;
	node.irq_type = CAMERA_IRQ_IMG;
	node.irq_property = IRQ_MAX_DONE;
	if (frame != NULL) {
		node.f_type = frame->type;
		node.index = frame->fid;
		node.height = frame->height;
		node.yaddr = frame->yaddr;
		node.uaddr = frame->uaddr;
		node.vaddr = frame->vaddr;
		node.yaddr_vir = frame->yaddr_vir;
		node.uaddr_vir = frame->uaddr_vir;
		node.vaddr_vir = frame->vaddr_vir;
	}
	ret = sprd_cam_buf_queue_write(&dev->queue, &node);
	if (ret) {
		pr_err("fail to write to queue\n");
		return ret;
	}

	complete(&dev->irq_com);
	pr_info("tx error\n");
	return ret;
}

static int sprd_img_tx_done(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_path_spec *path;
	struct camera_node node;

	if (frame == NULL || dev == NULL || param == NULL ||
		atomic_read(&dev->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	atomic_set(&dev->run_flag, 1);

	memset((void *)&node, 0, sizeof(node));

	if (frame->irq_type == CAMERA_IRQ_IMG) {
		path = &dev->cam_ctx.cam_path[frame->type];

		if (path->status == PATH_IDLE) {
			pr_info("CAM: path id %d idle\n", frame->type);
			return ret;
		}

		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = IRQ_MAX_DONE;
		node.f_type = frame->type;
		node.index = frame->fid;
		node.height = frame->height;
		node.yaddr = frame->yaddr;
		node.uaddr = frame->uaddr;
		node.vaddr = frame->vaddr;
		node.yaddr_vir = frame->yaddr_vir;
		node.uaddr_vir = frame->uaddr_vir;
		node.vaddr_vir = frame->vaddr_vir;
		node.frame_id = frame->frame_id;
		if (dev->zoom_ratio)
			node.zoom_ratio = frame->zoom_ratio;
		else
			node.zoom_ratio = ZOOM_RATIO_DEFAULT;
		memcpy(node.mfd, frame->buf_info.mfd, sizeof(uint32_t) * 3);
	} else if (frame->irq_type == CAMERA_IRQ_STATIS) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
		node.phy_addr = frame->phy_addr;
		node.vir_addr = frame->vir_addr;
		node.kaddr[0] = frame->kaddr[0];
		node.kaddr[1] = frame->kaddr[1];
		node.addr_offset = frame->addr_offset;
		node.buf_size = frame->buf_size;
		node.frame_id = frame->frame_id;
		if (dev->zoom_ratio)
			node.zoom_ratio = dev->zoom_ratio;
		else
			node.zoom_ratio = ZOOM_RATIO_DEFAULT;
		memcpy(node.mfd, frame->buf_info.mfd, sizeof(uint32_t) * 3);
	} else if (frame->irq_type == CAMERA_IRQ_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
		node.frame_id = frame->frame_id;
	} else if (frame->irq_type == CAMERA_IRQ_3DNR_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
		node.frame_id = frame->frame_id;
	} else {
		pr_err("fail to get valid irq_type %d\n", frame->irq_type);
		return -EINVAL;
	}
	node.boot_time = ktime_get_boottime();
	cam_get_timestamp(&node.time);

	CAM_TRACE("flag 0x%x type 0x%x\n",
		node.irq_flag, node.irq_type);

	ret = sprd_cam_buf_queue_write(&dev->queue, &node);
	if (ret) {
		pr_err("fail to write to queue\n");
		return ret;
	}

	CAM_TRACE("sprd_img %d %d %p\n", dev->idx, frame->type, dev);
	if (node.irq_property == IRQ_RAW_CAP_DONE)
		pr_info("dcam_core: RAW CAP tx done flag %d type %d fd = 0x%x\n",
			node.irq_flag, node.irq_type, frame->buf_info.mfd[0]);

	complete(&dev->irq_com);
	return ret;
}

static int sprd_img_tx_stop(void *param, uint32_t sensor_id)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	memset((void *)&node, 0, sizeof(node));
	node.irq_flag = IMG_TX_STOP;
	ret = sprd_cam_buf_queue_write(&dev->queue, &node);
	if (ret) {
		pr_err("fail to write to queue\n");
		return ret;
	}
	complete(&dev->irq_com);
	pr_info("tx stop %d\n", dev->irq_com.done);
	return ret;
}

static void sprd_timer_callback(unsigned long data)
{
	int ret = 0;
	struct camera_dev *dev = (struct camera_dev *)data;
	struct camera_node node;

	memset((void *)&node, 0, sizeof(node));
	if (data == 0 || dev == NULL || atomic_read(&dev->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	if (atomic_read(&dev->run_flag) == 0) {
		pr_info("CAM timeout.\n");
		node.irq_flag = IMG_TIMEOUT;
		node.irq_type = CAMERA_IRQ_IMG;
		node.irq_property = IRQ_MAX_DONE;
		node.invalid_flag = 0;
		ret = sprd_cam_buf_queue_write(&dev->queue, &node);
		if (ret)
			pr_err("fail to write queue error\n");

		complete(&dev->irq_com);
	}
}

static void sprd_init_timer(struct timer_list *cam_timer,
			unsigned long data)
{
	setup_timer(cam_timer, sprd_timer_callback, data);
}

static int sprd_start_timer(struct timer_list *cam_timer,
			uint32_t time_val)
{
	int ret = 0;

	CAM_TRACE("starting timer %ld\n", jiffies);
	ret = mod_timer(cam_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret)
		pr_err("fail to start in mod_timer %d\n", ret);

	return ret;
}

static int sprd_stop_timer(struct timer_list *cam_timer)
{
	CAM_TRACE("stop timer\n");
	del_timer_sync(cam_timer);

	return 0;
}

static int sprd_img_full_tx_done(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_path_spec *path =
		&dev->cam_ctx.cam_path[CAMERA_FULL_PATH];

	if (dev->cam_ctx.need_isp_tool
		|| path->assoc_idx == 0) {
		cam_buf_unmap_addr(&frame->buf_info);
		sprd_img_tx_done(frame, param);
		return 0;
	}

	if (path->assoc_idx != 0) {
		struct timeval tv;

		pr_debug("isp fetch start!\n");
		frame->irq_property = DCAM_FULL_PATH_TX_DONE;
		cam_get_timestamp(&tv);
		frame->timestamp = tv.tv_sec * 1000000000LL
		   + tv.tv_usec * 1000;
		isp_set_offline_frame(dev->isp_dev_handle,
			CAMERA_FULL_PATH, frame);
	} else {
		dcam_set_frame_addr(&dev->isp_dev_handle,
			CAMERA_FULL_PATH, frame);
		pr_info("frame back\n");
	}

	return ret;
}

static int sprd_img_bin_tx_done(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct isp_pipe_dev *isp_handle = NULL;
	struct isp_fmcu_slice_desc fmcu_slice;
	struct camera_path_spec *path =
		&dev->cam_ctx.cam_path[CAMERA_BIN_PATH];
	uint32_t isp_status = 0;

	memset((void *)&fmcu_slice, 0x00, sizeof(fmcu_slice));
	isp_handle = (struct isp_pipe_dev *)dev->isp_dev_handle;
	fmcu_slice = isp_handle->fmcu_slice;

	pr_debug("cam%d, isp_busy = %d,capture_state = %d, assoc_id = %d\n",
		dev->idx,
		isp_handle->isp_busy,
		fmcu_slice.capture_state,
		path->assoc_idx);
	if (dev->raw_cap && dev->raw_phase == 0) {
		pr_info("isp fetch start!\n");
		frame->irq_property = DCAM_BIN_PATH_TX_DONE;
		isp_start_raw_proc(dev->isp_dev_handle,
			CAMERA_BIN_PATH, frame);
		dev->raw_phase = 1;
	} else if (path->assoc_idx != 0) {
		frame->irq_property = DCAM_BIN_PATH_TX_DONE;
		isp_status = isp_get_status(isp_handle);
		if ((atomic_read(&dev->stream_on) == 1)) {
			isp_set_offline_frame(dev->isp_dev_handle,
				CAMERA_BIN_PATH, frame);
		} else {
			dcam_set_frame_addr(&dev->isp_dev_handle,
				CAMERA_BIN_PATH, frame);
			pr_info("isp is busy\n");
		}
	} else {
		dcam_set_frame_addr(&dev->isp_dev_handle,
			CAMERA_BIN_PATH, frame);
		pr_info("frame  back\n");
	}
	return ret;
}

static int sprd_img_dcam_reg_isr(struct camera_dev *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_SOF,
		sprd_img_tx_sof, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_FRM_ERR,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_LINE_ERR,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_DCAM_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_EOF,
		dcam_start_flash, param->flash_task);
	sprd_dcam_reg_isr(param->idx, DCAM_FULL_PATH_TX_DONE,
		sprd_img_full_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_BIN_PATH_TX_DONE,
		sprd_img_bin_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_AEM_PATH_TX_DONE,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_AFL_TX_DONE,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_AFM_INTREQ1,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_PATH_TX_DONE,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_VCH2_PATH_TX_DONE,
		sprd_img_tx_done, param);
#if 0
	sprd_dcam_reg_isr(param->idx, DCAM_NR3_TX_DONE,
		sprd_img_tx_done, param);
#endif
	return 0;
}

static int sprd_img_dcam_unreg_isr(struct camera_dev *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_SOF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_FRM_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_LINE_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_DCAM_OVF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_EOF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_FULL_PATH_TX_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_BIN_PATH_TX_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_PATH_TX_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_AEM_PATH_TX_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_AFL_TX_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_AFM_INTREQ1, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_PATH_TX_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_VCH2_PATH_TX_DONE, NULL, param);

	return 0;
}

static int sprd_img_isp_reg_isr(struct camera_dev *param)
{
	enum isp_id id = ISP_ID_0;

	if (!param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (param->idx == DCAM_ID_0)
		id = ISP_ID_0;
	else if (param->idx == DCAM_ID_1)
		id = ISP_ID_1;
	else {
		pr_err("fail to get valid dev idx %d\n", param->idx);
		return -EFAULT;
	}
	sprd_isp_reg_isr(id, ISP_PATH_PRE_DONE,
		sprd_img_tx_done, param);
	sprd_isp_reg_isr(id, ISP_PATH_VID_DONE,
		sprd_img_tx_done, param);
	sprd_isp_reg_isr(id, ISP_PATH_CAP_DONE,
		sprd_img_tx_done, param);
	sprd_isp_reg_isr(id, ISP_DCAM_SOF,
		sprd_img_tx_done, param);
	return 0;
}

static int sprd_img_isp_unreg_isr(struct camera_dev *param)
{
	enum isp_id id = ISP_ID_0;

	if (!param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (param->idx == DCAM_ID_0)
		id = ISP_ID_0;
	else if (param->idx == DCAM_ID_1)
		id = ISP_ID_1;
	else {
		pr_err("fail to get valid dev idx %d\n", param->idx);
		return -EFAULT;
	}
	sprd_isp_reg_isr(id, ISP_PATH_PRE_DONE,
			 NULL, param);
	sprd_isp_reg_isr(id, ISP_PATH_VID_DONE,
			 NULL, param);
	sprd_isp_reg_isr(id, ISP_PATH_CAP_DONE,
			 NULL, param);
	sprd_isp_reg_isr(id, ISP_DCAM_SOF,
			 NULL, param);

	return 0;
}

static int sprd_img_get_res(struct camera_group *group,
		enum dcam_id dcam_id, struct sprd_img_res *res)
{
	int ret = 0;
	uint32_t dcam_res_flag = 0;

	/*normal mode*/
	if (dcam_id == DCAM_ID_0) {
		dcam_res_flag = DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH;
		if (0 == (dcam_res_flag
			& group->dcam_res_used)) {
			pr_info("real sensor:dcam0\n");
			res->flag = dcam_res_flag;
			group->dcam_res_used |=	dcam_res_flag;
		} else {
			pr_err("Err: no valid dcam for real sensor!\n");
			ret = -1;
			goto exit;
		}
	} else if (dcam_id == DCAM_ID_1) {
		dcam_res_flag = DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH;
		if (0 == (dcam_res_flag &
			group->dcam_res_used)) {
			pr_info("front sensor:dcam1\n");
			res->flag = dcam_res_flag;
			group->dcam_res_used |= dcam_res_flag;
		} else {
			pr_err("fail to get valid dcam for front sensor!\n");
			ret = -1;
			goto exit;
		}
	} else if (dcam_id == DCAM_ID_2) {
		dcam_res_flag = DCAM_RES_DCAM2_CAP |
				DCAM_RES_DCAM2_PATH;
		if (0 == (dcam_res_flag &
			group->dcam_res_used)) {
			pr_info("ir sensor:dcam2\n");
			res->flag = dcam_res_flag;
			group->dcam_res_used |=	dcam_res_flag;
		} else {
			pr_err("fail to get valid dcam for ir sensor!\n");
			ret = -1;
			goto exit;
		}
	}

exit:
	if (ret)
		res->flag = 0;
	return ret;
}

static int sprd_img_put_res(struct camera_group *group,
			enum dcam_id dcam_id, struct sprd_img_res *res)
{
	int ret = 0;
	uint32_t dcam_res_flag = 0;

	if (dcam_id == DCAM_ID_0) {
		dcam_res_flag = DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH;
		if (dcam_res_flag ==
			(res->flag &
			group->dcam_res_used)) {
			pr_info("put dcam0 for rear sensor\n");
			group->dcam_res_used &= ~dcam_res_flag;
		} else if (DCAM_RES_DCAM0_CAP ==
			(res->flag & group->dcam_res_used)) {
			pr_info("put dcam0 top for rear sensor\n");
			group->dcam_res_used &= ~DCAM_RES_DCAM0_CAP;
			goto exit;
		} else {
			pr_err("Err: can't put dcam0 for rear sensor!\n");
			ret = -1;
			goto exit;
		}
	} else if (dcam_id == DCAM_ID_1) {
		dcam_res_flag = DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH;
		if (dcam_res_flag ==
			(res->flag & group->dcam_res_used)) {
			pr_info("put dcam1 for front sensor\n");
			group->dcam_res_used &= ~dcam_res_flag;
		} else if (DCAM_RES_DCAM1_CAP ==
			(res->flag &
			group->dcam_res_used)) {
			pr_info("put dcam1 top for front sensor\n");
			group->dcam_res_used &= ~DCAM_RES_DCAM1_CAP;
			goto exit;
		} else {
				pr_err("fail to put dcam1 for front sensor\n");
				ret = -1;
				goto exit;
		}
	} else if (dcam_id == DCAM_ID_2) {
		dcam_res_flag = DCAM_RES_DCAM2_CAP |
				DCAM_RES_DCAM2_PATH;
		if (dcam_res_flag ==
			(res->flag & group->dcam_res_used)) {
			pr_info("put dcam2 for ir sensor\n");
			group->dcam_res_used &= ~dcam_res_flag;
		} else if (DCAM_RES_DCAM2_CAP ==
			(res->flag &
			group->dcam_res_used)) {
			pr_info("put dcam2 top for ir sensor\n");
			group->dcam_res_used &= ~DCAM_RES_DCAM2_CAP;
			goto exit;
		} else {
				pr_err("fail to put dcam1 for front sensor\n");
				ret = -1;
				goto exit;
		}
	}

exit:
	if (ret)
		res->flag = 0;
	return ret;
}

static int sprd_img_io_get_dcam_res(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_res res = {0};
	enum dcam_id idx = DCAM_ID_0;
	struct camera_group *group = NULL;

	idx = camerafile->idx;
	group = camerafile->grp;

	ret = copy_from_user(&res, (void __user *)arg,
				sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy from user!\n");
		goto exit;
	}
	idx = sprd_sensor_find_dcam_id(res.sensor_id);
	if (idx == -1) {
		ret = -EFAULT;
		pr_err("fail to find attach dcam id!\n");
		goto exit;
	}
	ret = sprd_img_get_res(group, idx, &res);
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to get res!\n");
		goto exit;
	}

	if (group->mode_inited & (1 << (int)idx)) {
		pr_info("cam%d has been enabled!\n", idx);
		/*break;*/
	}
	ret = sprd_dcam_module_en(idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to enable dcam module %d\n", idx);
		goto exit;
	}

	ret = sprd_isp_module_en(group->dev[idx]->isp_dev_handle,
		(enum isp_id)idx);

	if (unlikely(ret != 0)) {
		pr_err("fail to enable isp module %d\n", idx);
		ret = -EFAULT;
		goto exit;
	}
	camerafile->idx = idx;
	group->mode_inited |= 1 << idx;
	pr_info("cam dev[%d] init OK: 0x%lx\n", idx,
			(unsigned long)group->dev[idx]);
	pr_info("sprd_img idx %d inited %d res_used %d\n",
			idx, group->dev_inited, group->dcam_res_used);

	ret = copy_to_user((void __user *)arg, &res,
				sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_io_put_dcam_res(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_res res = {0};
	enum dcam_id idx = DCAM_ID_0;
	struct camera_group *group = NULL;

	idx = camerafile->idx;
	group = camerafile->grp;

	ret = copy_from_user(&res, (void __user *)arg,
				sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy_from_user!\n");
		goto exit;
	}

	idx = sprd_sensor_find_dcam_id(res.sensor_id);
	pr_info("dev_inited:%d, idx: %d\n", group->dev_inited, idx);
	if (idx == -1) {
		ret = -EFAULT;
		pr_err("fail to find attach dcam id!\n");
		goto exit;
	}
	if (!(group->mode_inited & (1 << (int)idx))) {
		pr_err("dcam%d has been already disabled!\n", idx);
		/*break;*/
	}

	sprd_camera_stream_off(camerafile, dev, arg);

	ret = sprd_dcam_module_dis(idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to disable dcam%d module\n", idx);
		ret = -EFAULT;
	}

	ret = sprd_isp_module_dis(group->dev[idx]->isp_dev_handle,
		(enum isp_id)idx);

	if (unlikely(ret != 0)) {
		pr_err("SPRD_IMG%d: fail to disable isp module\n",
			idx);
		ret = -EFAULT;
	}
	group->mode_inited &= ~(1<<idx);

	ret = sprd_img_put_res(group, idx, &res);
	if (ret) {
		pr_err("fail to put res %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	ret = copy_to_user((void __user *)arg, &res,
			sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy_to_user!\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_set_sensor_if(struct camera_dev *dev,
				struct sprd_img_sensor_if *sensor_if)
{
	int ret = 0;
	struct camera_info *info = NULL;

	if (!sensor_if) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (sensor_if->res[0] != IF_OPEN)
		goto exit;

	info = &dev->cam_ctx;
	info->if_mode = sensor_if->if_type;
	info->sn_mode = sensor_if->img_fmt;
	info->img_ptn = sensor_if->img_ptn;
	info->frm_deci = sensor_if->frm_deci;

	CAM_TRACE("cam%d: if %d mode %d deci %d\n",
		dev->idx, info->if_mode, info->sn_mode,
		info->frm_deci);

	if (info->if_mode == DCAM_CAP_IF_CCIR) {
		/* CCIR interface */
		info->sync_pol.vsync_pol = sensor_if->if_spec.ccir.v_sync_pol;
		info->sync_pol.hsync_pol = sensor_if->if_spec.ccir.h_sync_pol;
		info->sync_pol.pclk_pol = sensor_if->if_spec.ccir.pclk_pol;
		info->data_bits = 8;
	} else {
		info->sync_pol.need_href = sensor_if->if_spec.mipi.use_href;
		info->is_loose = sensor_if->if_spec.mipi.is_loose;
		info->data_bits = sensor_if->if_spec.mipi.bits_per_pxl;
		info->lane_num = sensor_if->if_spec.mipi.lane_num;
		info->bps_per_lane = sensor_if->if_spec.mipi.pclk;
	}

exit:
	return ret;
}

static int sprd_img_io_set_sensor_if(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_sensor_if sensor;
	enum dcam_id idx = DCAM_ID_0;

	idx = camerafile->idx;

	CAM_TRACE("%d: set sensor if\n", idx);
	ret = copy_from_user(&sensor,
			(void __user *)arg,
			sizeof(struct sprd_img_sensor_if));
	if (unlikely(ret)) {
		pr_err("fail to copy form user%d\n", ret);
		ret = -EFAULT;
		return ret;
	}
	mutex_lock(&dev->cam_mutex);
	ret = sprd_set_sensor_if(dev, &sensor);
	mutex_unlock(&dev->cam_mutex);
	return ret;
}

static int sprd_img_io_set_sensor_size(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size size;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&size, (void __user *)arg,
				sizeof(struct sprd_img_size));
	if (ret) {
		pr_err("fail to get user info %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		return ret;
	}
	info->cap_in_size.w = size.w;
	info->cap_in_size.h = size.h;
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: sensor size %d %d\n", idx,
			info->cap_in_size.w, info->cap_in_size.h);
	return ret;
}

static int sprd_img_io_set_sensor_trim(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_rect rect;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&rect, (void __user *)arg,
				sizeof(struct sprd_img_rect));
	if (ret) {
		pr_err("fail to get user info %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		return ret;
	}
	info->cap_in_rect.x = rect.x;
	info->cap_in_rect.y = rect.y;
	info->cap_in_rect.w = rect.w;
	info->cap_in_rect.h = rect.h;
	info->cap_out_size.w = info->cap_in_size.w;
	info->cap_out_size.h = info->cap_in_size.h;

	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: sensor trim x y w h %d %d %d %d\n", idx,
			info->cap_in_rect.x,
			info->cap_in_rect.y,
			info->cap_in_rect.w,
			info->cap_in_rect.h);
	return ret;
	}

static int sprd_img_io_set_sensor_max_size(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size size;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&size, (void __user *)arg,
				sizeof(struct sprd_img_size));
	if (ret || !size.w || !size.h) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		return ret;
	}
	info->sn_max_size.w = size.w;
	info->sn_max_size.h = size.h;
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: sensor max size %d %d\n", idx,
			info->sn_max_size.w, info->sn_max_size.h);
	return ret;
}

static int sprd_img_io_set_output_size(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	CAM_TRACE("%d: set output size\n", idx);
	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
			sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	info->dst_size.w = parm.dst_size.w;
	info->dst_size.h = parm.dst_size.h;
	info->pxl_fmt = parm.pixel_fmt;
	info->sn_fmt = parm.sn_fmt;
	info->need_isp_tool = parm.need_isp_tool;
	info->need_isp = parm.need_isp;
	info->path_input_rect.x = parm.crop_rect.x;
	info->path_input_rect.y = parm.crop_rect.y;
	info->path_input_rect.w = parm.crop_rect.w;
	info->path_input_rect.h = parm.crop_rect.h;
	info->scene_mode = parm.scene_mode;
	info->raw_cap_flag = parm.need_isp_tool;
	info->slow_motion_flag = parm.slowmotion;

	if (parm.slowmotion)
		info->is_slow_motion = parm.slowmotion;
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_io_set_function_mode(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_function_mode parm;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	if (!dev || !info) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	CAM_TRACE("%d: set function mode\n", idx);
	memset((void *)&parm, 0, sizeof(parm));
	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
		sizeof(struct sprd_img_function_mode));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	info->need_4in1 = parm.need_4in1;
	info->need_3dnr = parm.need_3dnr;
	info->dual_cam = parm.dual_cam;

	pr_info("Set function mode 3dnr %d, 4in1 %d dual_cam %d\n",
		info->need_3dnr, info->need_4in1, info->dual_cam);
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_get_free_channel(struct camera_dev *dev,
	uint32_t *channel_id, uint32_t scene_mode)
{
	int ret = 0;
	struct camera_path_spec *full_path = NULL;
	struct camera_path_spec *bin_path = NULL;
	struct camera_path_spec *path_1 = NULL;
	struct camera_path_spec *path_2 = NULL;
	struct camera_path_spec *path_3 = NULL;
	struct camera_get_path_id path_id;

	if (!dev) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (!channel_id) {
		ret = -EFAULT;
		pr_err("fail to get valid param:channel id\n");
		goto exit;
	}

	full_path = &dev->cam_ctx.cam_path[CAMERA_FULL_PATH];
	bin_path = &dev->cam_ctx.cam_path[CAMERA_BIN_PATH];
	path_1 = &dev->cam_ctx.cam_path[CAMERA_PRE_PATH];
	path_2 = &dev->cam_ctx.cam_path[CAMERA_VID_PATH];
	path_3 = &dev->cam_ctx.cam_path[CAMERA_CAP_PATH];

	memset((void *)&path_id, 0x00, sizeof(path_id));
	path_id.input_size.w = dev->cam_ctx.cap_in_rect.w;
	path_id.input_size.h = dev->cam_ctx.cap_in_rect.h;
	path_id.output_size.w = dev->cam_ctx.dst_size.w;
	path_id.output_size.h = dev->cam_ctx.dst_size.h;
	path_id.fourcc = dev->cam_ctx.pxl_fmt;
	path_id.sn_fmt = dev->cam_ctx.sn_fmt;
	path_id.need_isp_tool = dev->cam_ctx.need_isp_tool;
	path_id.need_isp = dev->cam_ctx.need_isp;
	path_id.input_trim.x = dev->cam_ctx.path_input_rect.x;
	path_id.input_trim.y = dev->cam_ctx.path_input_rect.y;
	path_id.input_trim.w = dev->cam_ctx.path_input_rect.w;
	path_id.input_trim.h = dev->cam_ctx.path_input_rect.h;
	CAM_TRACE("get parm, path work %d %d %d %d %d\n",
		full_path->is_work, bin_path->is_work, path_1->is_work,
		path_2->is_work, path_3->is_work);
	path_id.is_path_work[CAMERA_FULL_PATH] = full_path->is_work;
	path_id.is_path_work[CAMERA_BIN_PATH] = bin_path->is_work;
	path_id.is_path_work[CAMERA_PRE_PATH] = path_1->is_work;
	path_id.is_path_work[CAMERA_VID_PATH] = path_2->is_work;
	path_id.is_path_work[CAMERA_CAP_PATH] = path_3->is_work;
	ret = sprd_camera_get_path_id(&path_id, channel_id, scene_mode);
	CAM_TRACE("get channel %d\n", *channel_id);

exit:
	return ret;
}

static int sprd_img_io_path_frm_deci(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	struct camera_path_spec *path = NULL;
	struct camera_info *info = NULL;

	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	path = &info->cam_path[parm.channel_id];
	path->path_frm_deci = parm.deci;
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_io_set_path_skip_num(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy from user %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	switch (parm.channel_id) {
	case CAMERA_FULL_PATH:
	case CAMERA_BIN_PATH:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		info->cam_path[parm.channel_id].skip_num =
			parm.skip_num;
		break;
	default:
		pr_info("fail to get valid channel ID, %d\n",
			parm.channel_id);
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: channel %d, skip_num %d\n",
			idx, parm.channel_id, parm.skip_num);
exit:
	return ret;
}

static int sprd_img_set_crop(struct camera_dev *dev, struct sprd_img_parm *p)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_rect *input_rect = NULL;
	struct camera_size *input_size = NULL;

	if (!dev) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	idx = dev->idx;
	if (p->crop_rect.x + p->crop_rect.w > dev->cam_ctx.cap_out_size.w ||
	p->crop_rect.y + p->crop_rect.h > dev->cam_ctx.cap_out_size.h) {
		ret = -EINVAL;
		goto exit;
	}

	CAM_TRACE("cam%d: set crop, window %d %d %d %d\n", idx,
		p->crop_rect.x, p->crop_rect.y,
		p->crop_rect.w, p->crop_rect.h);

	switch (p->channel_id) {
	case CAMERA_FULL_PATH:
	case CAMERA_BIN_PATH:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		input_size = &dev->cam_ctx.cam_path[p->channel_id].in_size;
		input_rect = &dev->cam_ctx.cam_path[p->channel_id].in_rect;
		break;
	default:
		pr_info("cam%d:fail to get right channel id %d\n",
			idx, p->channel_id);
		ret = -EINVAL;
		goto exit;
	}

	input_size->w = dev->cam_ctx.cap_out_size.w;
	input_size->h = dev->cam_ctx.cap_out_size.h;
	input_rect->x = p->crop_rect.x;
	input_rect->y = p->crop_rect.y;
	input_rect->w = p->crop_rect.w;
	input_rect->h = p->crop_rect.h;
	pr_debug("path %d, rect %d %d %d %d, size %d %d\n",
		p->channel_id, input_rect->x, input_rect->y,
		input_rect->w, input_rect->h, input_size->w,
		input_size->h);

exit:
	return ret;
}

static int sprd_img_io_set_crop(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	ret = sprd_img_set_crop(dev, &parm);
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_io_get_fmt(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct camera_format *fmt = NULL;
	struct sprd_img_get_fmt fmt_desc;

	CAM_TRACE("get fmt\n");
	ret = copy_from_user(&fmt_desc,
				(void __user *)arg,
				sizeof(struct sprd_img_get_fmt));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy from user\n");
		goto exit;
	}
	if (unlikely(fmt_desc.index >= ARRAY_SIZE(dcam_img_fmt)))
		return -EINVAL;

	fmt = &dcam_img_fmt[fmt_desc.index];
	fmt_desc.fmt = fmt->fourcc;

	ret = copy_to_user((void __user *)arg,
			&fmt_desc,
			sizeof(struct sprd_img_get_fmt));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_cam_convert_size(struct camera_size *in_size,
				     struct camera_rect *in_rect,
				     struct camera_size *out_size)
{
	int ret = 0;

	if (!in_size || !in_rect) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (in_size->w > ISP_PATH1_LINE_BUF_LENGTH) {
		in_size->w >>= 1;
		in_size->h >>= 1;
		in_rect->w >>= 1;
		in_rect->w = CAM_ALIGNTO(in_rect->w);
		if (in_rect->w * CAMERA_SC_COEFF_UP_MAX < out_size->w) {
			pr_info("more than max in_size{%d %d}, in_rect{%d %d %d %d}\n",
					in_size->w, in_size->h, in_rect->x,
					in_rect->y, in_rect->w, in_rect->h);
			in_rect->w = out_size->w / 4;
			in_rect->w = CAM_ALIGNTO(in_rect->w);
		}
		in_rect->h >>= 1;
		in_rect->h = CAM_ALIGNTO(in_rect->h);
		if (in_rect->h * CAMERA_SC_COEFF_UP_MAX < out_size->h) {
			pr_info("more than max in_size{%d %d}, in_rect{%d %d %d %d}\n",
					in_size->w, in_size->h, in_rect->x,
					in_rect->y, in_rect->w, in_rect->h);
			in_rect->h = out_size->h / 4;
			in_rect->h = CAM_ALIGNTO(in_rect->h);
		}
		in_rect->x = (in_size->w - in_rect->w) >> 1;
		in_rect->x = CAM_ALIGNTO(in_rect->x);
		in_rect->y = (in_size->h - in_rect->h) >> 1;
		in_rect->y = CAM_ALIGNTO(in_rect->y);
	}

	pr_debug("in_size{%d %d}, in_rect{%d %d %d %d}\n",
		in_size->w, in_size->h, in_rect->x,
		in_rect->y, in_rect->w, in_rect->h);

exit:

	return ret;
}

static int sprd_img_update_video(struct camera_dev *dev,
		uint32_t channel_id)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_path_spec *path = NULL;
	enum isp_path_index path_index;
	uint32_t zoom_ratio = ZOOM_RATIO_DEFAULT;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_debug("update video, channel %d\n", channel_id);

	mutex_lock(&dev->cam_mutex);

	path = &dev->cam_ctx.cam_path[channel_id];
	path_index = channel2path_idx(channel_id);

	if (channel_id == CAMERA_PRE_PATH || channel_id == CAMERA_VID_PATH)
		sprd_cam_convert_size(&path->in_size, &path->in_rect,
				      &path->out_size);
	pr_debug("in_size{%d %d}, in_rect{%d %d %d %d}\n",
		path->in_size.w, path->in_size.h, path->in_rect.x,
		path->in_rect.y, path->in_rect.w, path->in_rect.h);
	pr_debug("out_size{%d %d}\n",
		path->out_size.w, path->out_size.h);
	ret = sprd_isp_update_zoom_param(dev->isp_dev_handle,
					path_index,
					&path->in_size,
					&path->in_rect,
					&path->out_size);
	if (path->in_rect.w) {
		zoom_ratio = path->in_size.w * 1000 / path->in_rect.w;
		dev->zoom_ratio = zoom_ratio;
	}

	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("update video 0x%x\n", ret);

	if (ret)
		pr_err("fail to update video 0x%x\n", ret);

	return ret;
}

static struct camera_format *sprd_img_get_format(uint32_t fourcc)
{
	uint32_t i = 0;
	struct camera_format *fmt;

	for (i = 0; i < ARRAY_SIZE(dcam_img_fmt); i++) {
		fmt = &dcam_img_fmt[i];
		if (fmt->fourcc == fourcc)
			break;
	}

	if (unlikely(i == ARRAY_SIZE(dcam_img_fmt))) {
		pr_err("fail to get valid image format\n");
		return NULL;
	}

	return &dcam_img_fmt[i];
}

static int sprd_img_check_full_path_cap(uint32_t fourcc,
				struct sprd_img_format *f,
				struct camera_info *info)
{
	struct camera_path_spec *full_path = &info->cam_path[CAMERA_FULL_PATH];

	CAM_TRACE("check format for full_path\n");

	full_path->frm_type = f->channel_id;
	full_path->is_work = 0;

	switch (fourcc) {
	case IMG_PIX_FMT_GREY:
		full_path->out_fmt = DCAM_RAWRGB;
		full_path->pixel_depth = info->is_loose ? 16 : 10;
		full_path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
		break;
	case IMG_PIX_FMT_NV21:
		full_path->fourcc = fourcc;
		full_path->out_fmt = DCAM_YVU420;
		full_path->pixel_depth = 8;
		full_path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
		break;
	case IMG_PIX_FMT_NV12:
		full_path->fourcc = fourcc;
		full_path->out_fmt = DCAM_YUV420;
		full_path->pixel_depth = 8;
		full_path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
		break;
	default:
		pr_info("fail to get valid image format for full_path 0x%x\n",
			fourcc);
		return -EINVAL;
	}
	full_path->fourcc = fourcc;

	CAM_TRACE("check format for full_path: out_fmt=%d, is_loose=%d\n",
		full_path->out_fmt, info->is_loose);
	full_path->out_size.w = f->width;
	full_path->out_size.h = f->height;
	full_path->src_sel = 0;
	full_path->is_work = 1;

	return 0;
}

static int sprd_img_check_bin_path_cap(uint32_t fourcc,
				struct sprd_img_format *f,
				struct camera_info *info)
{
	struct camera_path_spec *bin_path = &info->cam_path[CAMERA_BIN_PATH];

	CAM_TRACE("check format for full_path\n");

	bin_path->frm_type = f->channel_id;
	bin_path->is_work = 0;

	switch (fourcc) {
	case IMG_PIX_FMT_GREY:
		bin_path->out_fmt = DCAM_RAWRGB;
		bin_path->pixel_depth = info->is_loose ? 16 : 10;
		bin_path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		break;
	default:
		pr_info("fail to get valid image format for bin_path 0x%x\n",
			fourcc);
		return -EINVAL;
	}
	bin_path->fourcc = fourcc;

	CAM_TRACE("check format for bin_path: out_fmt=%d, is_loose=%d\n",
		bin_path->out_fmt, info->is_loose);
	bin_path->out_size.w = f->width;
	bin_path->out_size.h = f->height;

	bin_path->is_work = 1;

	return 0;
}

static int sprd_img_check_path_pdaf_cap(uint32_t fourcc,
				struct sprd_img_format *f,
				struct camera_info *info)
{
	struct camera_path_spec *path_pdaf = &info->cam_path[CAMERA_PDAF_PATH];

	CAM_TRACE("check format for path pdaf\n");

	path_pdaf->frm_type = f->channel_id;
	path_pdaf->is_work = 0;
	/* need add pdaf path necessary info here*/
	path_pdaf->is_work = 1;

	return 0;
}

static int sprd_img_check_scaling(struct sprd_img_format *f,
				struct camera_info *info,
				struct camera_path_spec *path,
				uint32_t line_buf_size)
{
	uint32_t maxw = 0, maxh = 0, tempw = 0, temph = 0;

	tempw = path->in_rect.w;
	temph = path->in_rect.h;

	/* no need to scale */
	if (tempw == f->width && temph == f->height) {
		pr_info("Do not need scale\n");
		return 0;
	}

	/* scaling needed */
	switch (info->sn_mode) {
	case DCAM_CAP_MODE_RAWRGB:
		maxw = f->width * CAMERA_SC_COEFF_DOWN_MAX;
		maxw = maxw * (1 << CAMERA_PATH_DECI_FAC_MAX);
		maxh = f->height * CAMERA_SC_COEFF_DOWN_MAX;
		maxh = maxh * (1 << CAMERA_PATH_DECI_FAC_MAX);
		if (unlikely(tempw > maxw || temph > maxh)) {
			/* out of scaling capbility */
			pr_err("fail to scale:out of scaling capbility\n");
			return -EINVAL;
		}

		if (unlikely(f->width > line_buf_size)) {
			/* out of scaling capbility. TBD */
			pr_err("fail to scale:out of scaling capbility\n");
			/*return -EINVAL;*/
		}

		maxw = tempw * CAMERA_SC_COEFF_UP_MAX;
		maxh = temph * CAMERA_SC_COEFF_UP_MAX;
		if (unlikely(f->width > maxw || f->height > maxh)) {
			/* out of scaling capbility */
			pr_err("fail to scale:out of scaling capbility\n");
			return -EINVAL;
		}
		break;

	default:
		break;
	}

	return 0;
}

static void sprd_img_endian_sel(uint32_t fourcc,
				struct camera_path_spec *path)
{

	if (fourcc == IMG_PIX_FMT_YUV422P ||
		fourcc == IMG_PIX_FMT_RGB565 ||
		fourcc == IMG_PIX_FMT_RGB565X) {
		if (fourcc == IMG_PIX_FMT_YUV422P) {
			path->out_fmt = DCAM_YUV422;
		} else {
			path->out_fmt = DCAM_RGB565;
			if (fourcc == IMG_PIX_FMT_RGB565)
				path->end_sel.y_endian = DCAM_ENDIAN_HALFBIG;
			else
				path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		}
	} else {
		if (fourcc == IMG_PIX_FMT_YUV420 ||
			fourcc == IMG_PIX_FMT_YVU420) {
			path->out_fmt = DCAM_YUV420_3FRAME;
		} else {
			if (fourcc == IMG_PIX_FMT_NV12) {
				path->out_fmt = DCAM_YVU420;
				path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
			} else {
				path->out_fmt = DCAM_YUV420;
				path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
			}
		}
	}
}

static int sprd_img_check_path_cap(uint32_t fourcc,
				struct sprd_img_format *f,
				struct camera_info *info,
				enum camera_path_id path_id)
{
	int ret = 0;
	uint32_t tempw = 0, temph = 0, line_buf_size = 0;
	struct camera_path_spec *path;

	CAM_TRACE("check format for path%d\n", path_id);

	switch (path_id) {
	case CAMERA_PRE_PATH:
		line_buf_size = ISP_PATH1_LINE_BUF_LENGTH;
		break;
	case CAMERA_VID_PATH:
		line_buf_size = ISP_PATH2_LINE_BUF_LENGTH;
		break;
	case CAMERA_CAP_PATH:
		line_buf_size = ISP_PATH3_LINE_BUF_LENGTH;
		break;
	default:
		return -EINVAL;
	}

	path = &info->cam_path[path_id];
	path->is_work = 0;
	path->frm_type = f->channel_id;
	path->src_sel = f->need_isp;
	path->rot_mode = f->flip_on;
	path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
	path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
	path->pixel_depth = 0;
	path->img_deci.x_factor = 0;
	path->img_deci.y_factor = 0;
	tempw = path->in_rect.w;
	temph = path->in_rect.h;
	info->img_deci.x_factor = 0;
	f->need_binning = 0;
	/* app should fill in this field(fmt.pix.priv) to set the base index
	 * of frame buffer, and lately this field will return the flag
	 * whether ISP is needed for this work path
	 */
	switch (fourcc) {
	case IMG_PIX_FMT_GREY:
	case IMG_PIX_FMT_JPEG:
	case IMG_PIX_FMT_YUYV:
	case IMG_PIX_FMT_YVYU:
	case IMG_PIX_FMT_UYVY:
	case IMG_PIX_FMT_VYUY:
		if (unlikely(f->width != tempw || f->height != temph)) {
			/* need need scaling or triming */
			pr_err("fail to scale, src %d %d, dst %d %d\n",
				tempw, temph, f->width, f->height);
			return -EINVAL;
		}

		if (fourcc == IMG_PIX_FMT_GREY) {
			if (unlikely(info->sn_mode != DCAM_CAP_MODE_RAWRGB)) {
				/* the output of sensor is not RawRGB
				 * which is needed by app
				 */
				pr_err("fail to get RawRGB sensor\n");
				return -EINVAL;
			}

			path->out_fmt = DCAM_RAWRGB;
			path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		} else if (fourcc == IMG_PIX_FMT_JPEG) {
			if (unlikely(info->sn_mode != DCAM_CAP_MODE_JPEG)) {
				/* the output of sensor is not JPEG
				 * which is needed by app
				 */
				pr_err("fail to get JPEG sensor\n");
				return -EINVAL;
			}
			path->out_fmt = DCAM_JPEG;
		}
		break;
	case IMG_PIX_FMT_YUV422P:
	case IMG_PIX_FMT_YUV420:
	case IMG_PIX_FMT_YVU420:
	case IMG_PIX_FMT_NV12:
	case IMG_PIX_FMT_NV21:
	case IMG_PIX_FMT_RGB565:
	case IMG_PIX_FMT_RGB565X:
		if (info->sn_mode == DCAM_CAP_MODE_RAWRGB) {
			if (path->src_sel) {
				/* check scaling */
				ret = sprd_img_check_scaling(f, info, path,
							line_buf_size);
				if (ret)
					return ret;
			} else {
				/* no isp, only RawRGB data can be sampled */
				pr_err("fail to get valid format 0x%x\n",
					fourcc);
				return -EINVAL;
			}
		} else {
			pr_err("fail to get valid sensor mode 0x%x\n",
				info->sn_mode);
			return -EINVAL;
		}

		sprd_img_endian_sel(fourcc, path);
		break;
	default:
		pr_info("fail to get valid image format for path %d 0x%x\n",
			path_id, fourcc);
		return -EINVAL;
	}

	path->fourcc = fourcc;
	path->out_size.w = f->width;
	path->out_size.h = f->height;
	path->is_work = 1;

	return 0;
}

static int sprd_img_io_check_fmt(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id = 0;
	struct camera_format *fmt;
	struct sprd_img_format img_format;

	CAM_TRACE("check fmt\n");
	ret = copy_from_user(&img_format,
				(void __user *)arg,
				sizeof(struct sprd_img_format));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to get img_format\n");
		goto exit;
	}

	if (!dev) {
		ret = -EFAULT;
		pr_err("fail to get valid camerafile:NULL\n");
		goto exit;
	}

	dev->use_path = img_format.buffer_cfg_isp;

	fmt = sprd_img_get_format(img_format.fourcc);
	if (unlikely(!fmt)) {
		ret = -EFAULT;
		pr_err("fail to get valid fourcc format (0x%08x)\n",
			img_format.fourcc);
		goto exit;
	}

	if (img_format.channel_id == CAMERA_FULL_PATH) {
		mutex_lock(&dev->cam_mutex);
		ret = sprd_img_check_full_path_cap(fmt->fourcc,
					&img_format,
					&dev->cam_ctx);
		mutex_unlock(&dev->cam_mutex);
		channel_id = CAMERA_FULL_PATH;
	} else if (img_format.channel_id == CAMERA_BIN_PATH) {
		mutex_lock(&dev->cam_mutex);
		ret = sprd_img_check_bin_path_cap(fmt->fourcc,
					&img_format,
					&dev->cam_ctx);
		mutex_unlock(&dev->cam_mutex);
		channel_id = CAMERA_BIN_PATH;
	} else if (img_format.channel_id == CAMERA_PDAF_PATH) {
		mutex_lock(&dev->cam_mutex);
		ret = sprd_img_check_path_pdaf_cap(fmt->fourcc,
					&img_format,
					&dev->cam_ctx);
		mutex_unlock(&dev->cam_mutex);
		channel_id = CAMERA_PDAF_PATH;
	} else {
		mutex_lock(&dev->cam_mutex);
		ret = sprd_img_check_path_cap(fmt->fourcc, &img_format,
			&dev->cam_ctx, img_format.channel_id);
		mutex_unlock(&dev->cam_mutex);
		channel_id = img_format.channel_id;
	}

	if (channel_id < CAMERA_PDAF_PATH) {
		img_format.endian.y_endian =
			dev->cam_ctx.cam_path[channel_id].end_sel.y_endian;
		img_format.endian.uv_endian =
			dev->cam_ctx.cam_path[channel_id].end_sel.uv_endian;
	}

	if (ret == 0) {
		if (atomic_read(&dev->stream_on) != 0) {
			if (channel_id == CAMERA_PRE_PATH
				|| channel_id == CAMERA_VID_PATH
				|| channel_id == CAMERA_CAP_PATH) {
				ret = sprd_img_update_video(dev, channel_id);
			}
		} else {
			struct camera_path_spec *path =
				&dev->cam_ctx.cam_path[channel_id];

			sprd_cam_buf_queue_init(&path->buf_queue,
				CAM_QUEUE_NO_LOCK, sizeof(struct camera_addr),
				DCAM_BUF_QUEUE_LENGTH, "img buf_queue");
		}
	}

	ret = copy_to_user((void __user *)arg,
			&img_format,
			sizeof(struct sprd_img_format));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_io_set_shrink(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	struct camera_path_spec *path = NULL;
	struct camera_info *info = NULL;

	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	path = &info->cam_path[parm.channel_id];
	path->regular_desc = parm.regular_desc;
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("channel %d, regular mode %d\n",
		parm.channel_id, path->regular_desc.regular_mode);
exit:
	return ret;
}

static int sprd_img_io_pdaf_control(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	struct camera_path_spec *path = NULL;
	struct camera_info *info = NULL;

	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	path = &info->cam_path[parm.channel_id];
	path->pdaf_ctrl.mode = parm.pdaf_ctrl.mode;
	path->pdaf_ctrl.phase_data_type =
		parm.pdaf_ctrl.phase_data_type;
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("channel %d, pdaf mode %d type %d\n",
		parm.channel_id, path->pdaf_ctrl.mode,
		path->pdaf_ctrl.phase_data_type);
exit:
	return ret;
}

static int sprd_img_io_set_frm_id_base(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	switch (parm.channel_id) {
	case CAMERA_FULL_PATH:
	case CAMERA_BIN_PATH:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		info->cam_path[parm.channel_id].frm_id_base =
			parm.frame_base_id;
		break;
	default:
		pr_info("fail to get valid channel ID, %d\n",
			parm.channel_id);
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: channel %d, base id 0x%x\n",
			idx, parm.channel_id, parm.frame_base_id);
exit:
	return ret;
}

static int sprd_img_set_frame_addr(struct camera_file *camerafile,
				struct camera_dev *dev,
				struct sprd_img_parm *p)
{
	int ret = 0;
	uint32_t i = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_path_spec *path = NULL;
	enum isp_path_index path_index = ISP_PATH_IDX_0;

	if (!p) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	idx = camerafile->idx;

	switch (p->channel_id) {
	case CAMERA_FULL_PATH:
	case CAMERA_BIN_PATH:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		path = &dev->cam_ctx.cam_path[p->channel_id];
		break;
	default:
		pr_info("fail to get valid channel %d\n", p->channel_id);
		return -EINVAL;
	}

	CAM_TRACE("set path%d frame addr,status %d cnt %d reserved_buf %d\n",
		p->channel_id, path->status, path->frm_cnt_act,
		p->is_reserved_buf);

	if (unlikely(p->fd_array[0] == 0)) {
		pr_info("chn %d fail to get fd\n", p->channel_id);
		ret = -EINVAL;
		goto exit;
	}

	if (p->is_reserved_buf == 1) {
		path->frm_reserved_addr.yaddr = p->frame_addr_array[0].y;
		path->frm_reserved_addr.uaddr = p->frame_addr_array[0].u;
		path->frm_reserved_addr.vaddr = p->frame_addr_array[0].v;
		path->frm_reserved_addr.yaddr_vir =
			p->frame_addr_vir_array[0].y;
		path->frm_reserved_addr.uaddr_vir =
			p->frame_addr_vir_array[0].u;
		path->frm_reserved_addr.vaddr_vir =
			p->frame_addr_vir_array[0].v;
		path->frm_reserved_addr.mfd_y = p->fd_array[0];
		path->frm_reserved_addr.mfd_u = p->fd_array[0];
		path->frm_reserved_addr.mfd_v = p->fd_array[0];
		path->frm_reserved_addr.p_dev = &camerafile->grp->pdev->dev;
		path->frm_reserved_addr.type = CAM_BUF_USER_TYPE;

	} else {
		struct camera_addr frame_addr = {0};

		if (atomic_read(&dev->stream_on) == 1 &&
			path->status == PATH_RUN) {

			for (i = 0; i < p->buffer_count; i++) {
				if (p->fd_array[0] == 0) {
					pr_info("fail to get valid fd\n");
					ret = -EINVAL;
					goto exit;
				}
				frame_addr.yaddr = p->frame_addr_array[i].y;
				frame_addr.uaddr = p->frame_addr_array[i].u;
				frame_addr.vaddr = p->frame_addr_array[i].v;
				frame_addr.yaddr_vir =
						p->frame_addr_vir_array[i].y;
				frame_addr.uaddr_vir =
						p->frame_addr_vir_array[i].u;
				frame_addr.vaddr_vir =
						p->frame_addr_vir_array[i].v;
				frame_addr.mfd_y = p->fd_array[i];
				frame_addr.mfd_u = p->fd_array[i];
				frame_addr.mfd_v = p->fd_array[i];
				frame_addr.p_dev = &camerafile->grp->pdev->dev;

				if (p->channel_id == CAMERA_FULL_PATH) {
					ret = set_dcam_full_path_cfg(idx,
						DCAM_PATH_OUTPUT_ADDR,
						&frame_addr);
					if (unlikely(ret)) {
						pr_err("fail to full addr\n");
						goto exit;
					}
				} else if (p->channel_id == CAMERA_BIN_PATH) {
					ret = set_dcam_bin_path_cfg(idx,
						DCAM_PATH_OUTPUT_ADDR,
						&frame_addr);
					if (unlikely(ret)) {
						pr_err("fail to bin addr\n");
						goto exit;
					}
				} else {
					path_index =
						channel2path_idx(p->channel_id);
					ret = set_isp_path_cfg(
						dev->isp_dev_handle,
						path_index,
						ISP_PATH_OUTPUT_ADDR,
						&frame_addr);
					if (unlikely(ret)) {
						pr_err("fail to cfg isp\n");
						goto exit;
					}
				}
			}
		} else {

			for (i = 0; i < p->buffer_count; i++) {
				if (unlikely(p->fd_array[0] == 0)) {
					pr_info("fail to get valid fd\n");
					ret = -EINVAL;
					goto exit;
				}
				frame_addr.yaddr =
					p->frame_addr_array[i].y;
				frame_addr.uaddr =
					p->frame_addr_array[i].u;
				frame_addr.vaddr =
					p->frame_addr_array[i].v;
				frame_addr.yaddr_vir =
					p->frame_addr_vir_array[i].y;
				frame_addr.uaddr_vir =
					p->frame_addr_vir_array[i].u;
				frame_addr.vaddr_vir =
					p->frame_addr_vir_array[i].v;
				frame_addr.mfd_y = p->fd_array[i];
				frame_addr.mfd_u = p->fd_array[i];
				frame_addr.mfd_v = p->fd_array[i];
				frame_addr.p_dev =
					&camerafile->grp->pdev->dev;

				ret = sprd_cam_buf_queue_write(
					&path->buf_queue,
					&frame_addr);
				pr_debug("y=0x%x u=0x%x mfd=0x%x 0x%x\n",
					frame_addr.yaddr, frame_addr.uaddr,
					frame_addr.mfd_y, frame_addr.mfd_u);
			}
		}
	}

	CAM_TRACE("%d: success to set frame addr\n", idx);

exit:
	return ret;
}

static int sprd_img_io_set_frame_addr(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	enum dcam_id idx = DCAM_ID_0;

	idx = camerafile->idx;

	CAM_TRACE("%d: set frame addr\n", idx);
	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	ret = sprd_img_set_frame_addr(camerafile, dev, &parm);
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

int dcam_set_frame_addr(void **isp_dev_handle, enum camera_path_id path_idx,
		struct camera_frame *frame_addr)
{
	int ret = 0;
	struct camera_path_spec *path_full = NULL;
	struct camera_path_spec *path_bin = NULL;
	struct camera_addr cam_addr = {0};
	struct camera_dev *dev = container_of(isp_dev_handle,
				struct camera_dev, isp_dev_handle);
	struct camera_group *group = image_dev.this_device->platform_data;

	if (frame_addr == NULL) {
		CAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}

	path_full = &dev->cam_ctx.cam_path[CAMERA_FULL_PATH];
	path_bin = &dev->cam_ctx.cam_path[CAMERA_BIN_PATH];

	if (atomic_read(&dev->stream_on) == 0) {
		CAM_TRACE("it has been stream off\n");
		return -EPERM;
	}

	cam_addr.yaddr = frame_addr->yaddr;
	cam_addr.p_dev = &group->pdev->dev;
	cam_addr.type = frame_addr->buf_info.type;
	cam_addr.client[0] = frame_addr->buf_info.client[0];
	cam_addr.handle[0] = frame_addr->buf_info.handle[0];
	cam_addr.iova[0] = frame_addr->buf_info.iova[0];
	cam_addr.state = frame_addr->buf_info.state;
	cam_addr.num = frame_addr->buf_info.num;

	if (path_idx == CAMERA_FULL_PATH
		&& frame_addr->width == path_full->out_size.w
		&& frame_addr->height == path_full->out_size.h) {

		ret = set_dcam_full_path_cfg(dev->idx,
			DCAM_PATH_OUTPUT_ADDR,
			&cam_addr);
		if (unlikely(ret)) {
			pr_err("fail to cfg full_path output addr\n");
			goto exit;
		}
	} else if (path_idx == CAMERA_BIN_PATH
		&& frame_addr->width == path_bin->out_size.w
		&& frame_addr->height == path_bin->out_size.h) {

		ret = set_dcam_bin_path_cfg(dev->idx,
			DCAM_PATH_OUTPUT_ADDR,
			&cam_addr);
		if (unlikely(ret)) {
			pr_err("fail to cfg bin_path output addr\n");
			goto exit;
		}
	}  else {
		pr_err("fail to take back frame, %d %dx%d\n", path_idx,
			frame_addr->width, frame_addr->height);
	}
	CAM_TRACE("%d: success to set frame addr\n", dev->idx);

exit:
	return ret;
}

static int sprd_camera_update_clk(struct camera_dev *dev,
	struct device_node *dn)
{
	int ret = 0;
	uint32_t dcam_clk_index = DCAM_CLK_307M2_INDEX;
	uint32_t isp_clk_index = ISP_CLK_307M2_INDEX;
	uint32_t width = 0, height = 0;
	uint32_t width_max = 0, height_max = 0;
	struct camera_info *info = NULL;
	struct camera_path_spec *path_cap = NULL;

	if (!dev || !dn) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	info = &dev->cam_ctx;
	if (info == NULL) {
		ret = -EFAULT;
		pr_err("fail to get valid cam_ctx:NULL\n");
		goto exit;
	}
	path_cap = &info->cam_path[CAMERA_CAP_PATH];
	width_max = info->sn_max_size.w;
	height_max = info->sn_max_size.h;
	width = info->cap_in_size.w;
	height = info->cap_in_size.h;

	if (info->bps_per_lane == 0xffff) {
		pr_info("dcam and isp not support power optimization\n");
		goto exit;
	}

	if (width <= CAP_IN_SIZE_2M_WIDTH)
		dcam_clk_index = DCAM_CLK_256M_INDEX;
	else
		dcam_clk_index = DCAM_CLK_307M2_INDEX;

	if ((width_max > CAP_IN_SIZE_5M_WIDTH &&
		path_cap->is_work) ||
		info->raw_cap_flag ||
		info->slow_motion_flag)
		dcam_clk_index = DCAM_CLK_307M2_INDEX;

	ret = sprd_dcam_update_clk(dcam_clk_index, dn);
	if (unlikely(ret != 0)) {
		pr_err("fail to update dcam_if clk\n");
		goto exit;
	}

	if (width > CAP_IN_SIZE_5M_WIDTH)
		isp_clk_index = ISP_CLK_MAX_INDEX;
	else if (width <= CAP_IN_SIZE_2M_WIDTH)
		isp_clk_index = ISP_CLK_256M_INDEX;
	else
		isp_clk_index = ISP_CLK_307M2_INDEX;

	if ((width_max > CAP_IN_SIZE_5M_WIDTH &&
		path_cap->is_work) ||
		info->raw_cap_flag ||
		info->slow_motion_flag)
		isp_clk_index = ISP_CLK_MAX_INDEX;

	ret = sprd_isp_update_clk(isp_clk_index, dn);
	if (unlikely(ret != 0)) {
		pr_err("fail to update isp clk\n");
		goto exit;
	}

exit:
	return ret;
}

static int sprd_cfg_dcam_isp_pipeline(struct camera_file *camerafile)
{
	int ret = 0;
	struct camera_info *info;
	struct camera_dev *dev;

	struct camera_path_spec *path_pre = NULL;
	struct camera_path_spec *path_vid = NULL;
	struct camera_path_spec *path_cap = NULL;
	struct camera_path_spec *path_full = NULL;
	struct camera_path_spec *path_bin = NULL;
	struct camera_path_spec *path = NULL;

	sprd_img_get_cam_dev(camerafile, &dev, &info);
	if (!info) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	path_pre = &info->cam_path[CAMERA_PRE_PATH];
	path_vid = &info->cam_path[CAMERA_VID_PATH];
	path_cap = &info->cam_path[CAMERA_CAP_PATH];
	path_full = &info->cam_path[CAMERA_FULL_PATH];
	path_bin = &info->cam_path[CAMERA_BIN_PATH];

	path_pre->path_mode = ISP_PRE_OFFLINE;
	path_vid->path_mode = ISP_VID_OFFLINE;
	path_cap->path_mode = ISP_CAP_OFFLINE;
	if (path_cap->is_work) {
		if (!path_full->is_work) {
			memcpy(path_full, path_cap,
				sizeof(struct camera_path_spec));
			path_full->is_work = 1;
			path_full->in_rect.x = 0;
			path_full->in_rect.y = 0;
			path_full->in_rect.w = path_full->in_size.w;
			path_full->in_rect.h = path_full->in_size.h;
			path_full->out_size = path_full->in_size;
			if (info->need_4in1)
				path_full->src_sel = 0;
			else
				path_full->src_sel = 1;
			path_full->assoc_idx = 1 << CAMERA_CAP_PATH;
			path_full->out_fmt = DCAM_RAWRGB;
			path_cap->assoc_idx = 1 << CAMERA_FULL_PATH;
		}
	}
	if (path_pre->is_work) {
		if (!path_bin->is_work) {
			memcpy(path_bin, path_pre,
				sizeof(struct camera_path_spec));
			sprd_cam_convert_size(&path_pre->in_size,
						  &path_pre->in_rect,
						  &path_pre->out_size);
			path_bin->in_rect.x = 0;
			path_bin->in_rect.y = 0;
			path_bin->in_rect.w = path_bin->in_size.w;
			path_bin->in_rect.h = path_bin->in_size.h;
			path_bin->out_size = path_pre->in_size;
			path_bin->is_work = 1;
			path_bin->assoc_idx = 1 << CAMERA_PRE_PATH;
			path_bin->out_fmt = DCAM_RAWRGB;
			path_pre->assoc_idx = 1 << CAMERA_BIN_PATH;
			if (path_vid->is_work)
				sprd_cam_convert_size(&path_vid->in_size,
							  &path_vid->in_rect,
							  &path_vid->out_size);
		}
	}
	if (path_vid->is_work) {
		if (path_full->is_work
			&& path_vid->in_size.w < 640
			&& path_vid->in_size.h < 480) {
			path_full->assoc_idx |= 1 << CAMERA_VID_PATH;
			path_vid->assoc_idx = 1 << CAMERA_FULL_PATH;
		} else if (!path_bin->is_work) {
			memcpy(path_bin, path_vid,
				sizeof(struct camera_path_spec));
			sprd_cam_convert_size(&path_vid->in_size,
						  &path_vid->in_rect,
						  &path_vid->out_size);
			path_bin->is_work = 1;
			path_bin->in_rect.x = 0;
			path_bin->in_rect.y = 0;
			path_bin->in_rect.w = path_bin->in_size.w;
			path_bin->in_rect.h = path_bin->in_size.h;
			path_bin->out_size = path_vid->in_size;
			path_bin->assoc_idx |= 1 << CAMERA_VID_PATH;
			path_bin->out_fmt = DCAM_RAWRGB;
			path_vid->assoc_idx = 1 << CAMERA_BIN_PATH;
		} else {
			path_bin->assoc_idx |= 1 << CAMERA_VID_PATH;
			path_vid->assoc_idx = 1 << CAMERA_BIN_PATH;
		}
	}

	if (info->need_4in1 && path_bin->is_work) {
		path_bin->in_size.w = path_bin->in_size.w >> 1;
		path_bin->in_size.h = path_bin->in_size.h >> 1;
		path_bin->in_rect.w = path_bin->in_rect.w >> 1;
		path_bin->in_rect.h = path_bin->in_rect.h >> 1;
		path_bin->in_rect.x = (path_bin->in_size.w -
			path_bin->in_rect.w) >> 1;
		path_bin->in_rect.x = CAM_ALIGNTO(path_bin->in_rect.x);
		path_bin->in_rect.y = (path_bin->in_size.h -
			path_bin->in_rect.h) >> 1;
		path_bin->in_rect.y = CAM_ALIGNTO(path_bin->in_rect.y);
	}

	if (path_full->is_work && path_full->assoc_idx != 0) {
		struct cam_buf_info buf_info;
		struct camera_addr cam_addr = {0};
		size_t size;
		int i, buf_num;

		path = path_full;
		sprd_cam_buf_queue_init(&path->buf_queue, CAM_QUEUE_NO_LOCK,
			sizeof(struct camera_addr), DCAM_FRM_QUEUE_LENGTH + 1,
			"full path (cam_core for isp)");

		size = dcam_calc_raw10_line_bytes(0, path->out_size.w)
			* path->out_size.h;
		/*if it is zsl capture*/
		if (info->scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK)
			buf_num = 5;
		else
			buf_num = 5;

		for (i = 0; i < buf_num; i++) {
			/*sprintf(name, "sprd-cam-offline-%d", i++);*/
			cam_buf_alloc_k(&buf_info, &s_dcam_pdev->dev,
				size, 1, CAM_BUF_SWAP_TYPE);
			cam_addr.p_dev = &camerafile->grp->pdev->dev;
			cam_addr.type = buf_info.type;
			cam_addr.num = buf_info.num;
			cam_addr.client[0] = buf_info.client[0];
			cam_addr.handle[0] = buf_info.handle[0];
			sprd_cam_buf_queue_write(&path->buf_queue,
				&cam_addr);
		}
		cam_buf_alloc_k(&buf_info, &s_dcam_pdev->dev,
			size, 1, CAM_BUF_SWAP_TYPE);
		cam_addr.p_dev = &camerafile->grp->pdev->dev;
		cam_addr.type = buf_info.type;
		cam_addr.num = buf_info.num;
		cam_addr.client[0] = buf_info.client[0];
		cam_addr.handle[0] = buf_info.handle[0];
		path->frm_reserved_addr = cam_addr;
	}
	if (path_bin->is_work && path_bin->assoc_idx != 0) {
		struct cam_buf_info buf_info;
		struct camera_addr cam_addr = {0};
		size_t size;
		int i;

		path = path_bin;
		sprd_cam_buf_queue_init(&path->buf_queue, CAM_QUEUE_NO_LOCK,
			sizeof(struct camera_addr), DCAM_FRM_QUEUE_LENGTH + 1,
			"bin path (cam_core for isp)");

		size = dcam_calc_raw10_line_bytes(0, path->out_size.w)
			* path->out_size.h;
		for (i = 0; i < DCAM_FRM_QUEUE_LENGTH; i++) {
			cam_buf_alloc_k(&buf_info, &s_dcam_pdev->dev,
				size, 1, CAM_BUF_SWAP_TYPE);
			cam_addr.p_dev = &camerafile->grp->pdev->dev;
			cam_addr.type = buf_info.type;
			cam_addr.num = buf_info.num;
			cam_addr.client[0] = buf_info.client[0];
			cam_addr.handle[0] = buf_info.handle[0];
			sprd_cam_buf_queue_write(&path->buf_queue,
				&cam_addr);
		}
		cam_buf_alloc_k(&buf_info, &s_dcam_pdev->dev,
			size, 1, CAM_BUF_SWAP_TYPE);
		cam_addr.p_dev = &camerafile->grp->pdev->dev;
		cam_addr.type = buf_info.type;
		cam_addr.num = buf_info.num;
		cam_addr.client[0] = buf_info.client[0];
		cam_addr.handle[0] = buf_info.handle[0];
		path->frm_reserved_addr = cam_addr;

	}

	pr_debug("path is_work: %d %d %d %d %d, assoc_idx: 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		path_full->is_work, path_bin->is_work,
		path_pre->is_work, path_cap->is_work, path_vid->is_work,
		path_full->assoc_idx, path_bin->assoc_idx,
		path_pre->assoc_idx, path_cap->assoc_idx, path_vid->assoc_idx);
	pr_debug("prv path_mode: %d, cap path_mode %d, vid path mode %d\n",
		path_pre->path_mode, path_cap->path_mode, path_vid->path_mode);
exit:
	return ret;
}

static int sprd_dcam_cfg_cap(struct camera_info *info, enum dcam_id idx)
{
	int ret = DCAM_RTN_SUCCESS;
	/*uint32_t param = 0;*/

	if (info == NULL) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INTERFACE, &info->if_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap interface\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SENSOR_MODE, &info->sn_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sensor mode\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SAMPLE_MODE, &info->capture_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sample mode\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SYNC_POL, &info->sync_pol);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sync pol\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_BITS, &info->data_bits);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap data bits\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_PATTERN,
				&info->img_ptn);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap data bits\n");
		goto exit;
	}

	if (info->sn_mode == DCAM_CAP_MODE_RAWRGB &&
		info->if_mode == DCAM_CAP_IF_CSI2) {

		ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_PACKET,
					&info->is_loose);
		if (unlikely(ret)) {
			pr_err("fail to cfg dcam cap data packet\n");
			goto exit;
		}
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_FRM_DECI, &info->frm_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap frame deci\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INPUT_RECT, &info->cap_in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap input rect\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_FRM_COUNT_CLR, NULL);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap frame count clear\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_PRE_SKIP_CNT, &info->skip_number);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap pre skip count\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_IMAGE_XY_DECI, &info->img_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap image xy deci\n");
		goto exit;
	}

	if (info->need_4in1) {
		ret = set_dcam_cap_cfg(idx, DCAM_CAP_4IN1_BYPASS, NULL);
		if (unlikely(ret)) {
			pr_err("fail to cfg dcam cap 4in1 bypass\n");
			goto exit;
		}
	}

exit:
	return ret;
}

static int sprd_dcam_cfg_full_path(struct camera_path_spec *full_path,
	enum dcam_id idx)
{
	int ret = 0;
	uint32_t param = 0;
	struct camera_addr cur_node = {0};

	if (full_path == NULL) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_SRC_SEL,
		&full_path->src_sel);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path src\n");
		goto exit;
	}

	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_INPUT_SIZE,
		&full_path->in_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path input rect\n");
		goto exit;
	}

	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_INPUT_RECT,
		&full_path->in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path input rect\n");
		goto exit;
	}

	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_OUTPUT_FORMAT,
		&full_path->out_fmt);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path format\n");
		goto exit;
	}

	param = full_path->pixel_depth > 10 ? 1 : 0;
	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_OUTPUT_LOOSE,
		&param);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path format\n");
		goto exit;
	}

	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_DATA_ENDIAN,
		&full_path->end_sel);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path data endian\n");
		goto exit;
	}

	while (sprd_cam_buf_queue_read(&full_path->buf_queue,
		&cur_node) == 0) {
		ret = set_dcam_full_path_cfg(idx, DCAM_PATH_OUTPUT_ADDR,
			&cur_node);
		if (unlikely(ret)) {
			pr_err("fail to cfg full_path output addr\n");
			goto exit;
		}
	}

	ret = set_dcam_full_path_cfg(idx,
		DCAM_PATH_OUTPUT_RESERVED_ADDR,
		&full_path->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path output reserved addr\n");
		goto exit;
	}

	param = 1;
	ret = set_dcam_full_path_cfg(idx, DCAM_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable full_path\n");
		goto exit;
	}

exit:
	return ret;
}

static int sprd_dcam_cfg_bin_path(struct camera_path_spec *bin_path,
	enum dcam_id idx)
{
	int ret = 0;
	uint32_t param = 0;
	struct camera_addr cur_node = {0};

	if (bin_path == NULL) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_INPUT_SIZE,
		&bin_path->in_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg bin_path input rect\n");
		goto exit;
	}

	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_INPUT_RECT,
		&bin_path->in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg bin_path input rect\n");
		goto exit;
	}

	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_OUTPUT_FORMAT,
		&bin_path->out_fmt);
	if (unlikely(ret)) {
		pr_err("fail to cfg bin_path format\n");
		goto exit;
	}

	param = bin_path->pixel_depth > 10 ? 1 : 0;
	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_OUTPUT_LOOSE,
		&param);
	if (unlikely(ret)) {
		pr_err("fail to cfg bin_path format\n");
		goto exit;
	}

	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_DATA_ENDIAN,
		&bin_path->end_sel);
	if (unlikely(ret)) {
		pr_err("fail to cfg bin_path data endian\n");
		goto exit;
	}

	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_OUTPUT_SIZE,
		&bin_path->out_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg bin_path size\n");
		goto exit;
	}

	while (sprd_cam_buf_queue_read(&bin_path->buf_queue, &cur_node) == 0) {

		ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_OUTPUT_ADDR,
			&cur_node);
		if (unlikely(ret)) {
			pr_err("fail to cfg bin_path output addr\n");
			goto exit;
		}
	}

	if (!(bin_path->assoc_idx & (1 << CAMERA_CAP_PATH))) {
		ret = set_dcam_bin_path_cfg(idx,
			DCAM_PATH_OUTPUT_RESERVED_ADDR,
			&bin_path->frm_reserved_addr);
		if (unlikely(ret)) {
			pr_err("fail to cfg bin_path output reserved addr\n");
			goto exit;
		}
	}
	param = 1;
	ret = set_dcam_bin_path_cfg(idx, DCAM_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable bin_path\n");
		goto exit;
	}

exit:
	return ret;
}

static int sprd_dcam_cfg(struct camera_dev *dev)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;
	struct camera_path_spec *full_path = NULL;
	struct camera_path_spec *bin_path = NULL;
	struct camera_path_spec *path_pdaf = NULL;
	struct dcam_module *dcam_dev = NULL;
	struct camera_size size = {0};

	info = &dev->cam_ctx;
	idx = dev->idx;
	full_path = &info->cam_path[CAMERA_FULL_PATH];
	bin_path = &info->cam_path[CAMERA_BIN_PATH];
	path_pdaf = &info->cam_path[CAMERA_PDAF_PATH];
	dcam_dev = get_dcam_module(dev->idx);

	ret = sprd_dcam_module_init(idx);
	if (unlikely(ret)) {
		pr_err("fail to init dcam module\n");
		goto exit;
	}

	ret = sprd_img_dcam_reg_isr(dev);
	if (unlikely(ret)) {
		pr_err("fail to register dcam isr\n");
		goto exit;
	}

	/* config cap sub-module */
	ret = sprd_dcam_cfg_cap(info, idx);
	if (unlikely(ret)) {
		pr_err("fail to config cap");
		goto exit;
	}

	/* config dcam_if full_path */
	if (full_path->is_work) {
		ret = sprd_dcam_cfg_full_path(full_path, idx);
		if (unlikely(ret)) {
			pr_err("fail to config full_path cap");
			goto exit;
		}
		full_path->status = PATH_RUN;
	}

	/* config dcam_if bin_path */
	if (bin_path->is_work) {
		ret = sprd_dcam_cfg_bin_path(bin_path, idx);
		if (unlikely(ret)) {
			pr_err("fail to config bin_path cap");
			goto exit;
		}
		bin_path->status = PATH_RUN;
	}
	/*for dcam 3dnr_me*/
	if (info->need_3dnr) {
		size.w = info->cap_in_size.w;
		size.h = info->cap_in_size.h;
		pr_debug("3DNR fast me size w%d h %d\n", size.w, size.h);
		ret = sprd_dcam_fast_me_info(idx, info->need_3dnr, &size);
		if (unlikely(ret)) {
			pr_err("fail to cfg_nr3_fast_me info\n");
			goto exit;
		}
	}

	ret = sprd_cam_cfg_statis_buf(&s_dcam_pdev->dev,
		&dcam_dev->statis_module_info, &dev->init_inptr);
	if (ret != 0) {
		pr_err("fail to cfg statis!\n");
		goto exit;
	}

	pr_debug("end dcam cfg.\n");
exit:
	return ret;
}

static int sprd_set_isp_path_buf_cfg(void *handle,
	enum isp_config_param cfg_id, enum isp_path_index path_index,
	struct cam_buf_queue *queue)
{
	int ret = 0;
	struct camera_addr cur_node;

	if (!queue || !handle) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		goto exit;

	}

	while (sprd_cam_buf_queue_read(queue, &cur_node) == 0)
		ret = set_isp_path_cfg(handle, path_index, cfg_id, &cur_node);

exit:
	return ret;
}

static int sprd_isp_path_cfg_block(struct camera_path_spec *path,
	void **handle, enum isp_path_index path_index)
{
	int ret = 0;
	uint32_t param = 0;
	struct isp_endian_sel endian;
	struct isp_regular_info regular_info;
	struct camera_size me_conv_size;
	struct camera_size sns_size;
	struct camera_dev *dev = NULL;

	memset(&endian, 0x00, sizeof(endian));

	if (!path || !handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = container_of(handle, struct camera_dev, isp_dev_handle);

	if (!dev) {
		pr_err("fail to get valid dev\n");
		return -EINVAL;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_INPUT_SIZE,
		&path->in_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d input size\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_INPUT_RECT,
		&path->in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d input rect\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_OUTPUT_SIZE,
		&path->out_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output size\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_OUTPUT_FORMAT,
		&path->out_fmt);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output format\n", path_index);
		goto exit;
	}

	ret = sprd_set_isp_path_buf_cfg(*handle, ISP_PATH_OUTPUT_ADDR,
		path_index, &path->buf_queue);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output addr\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index,
		ISP_PATH_OUTPUT_RESERVED_ADDR,
		&path->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output reserved addr\n",
			path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_FRM_DECI,
		&path->path_frm_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d frame deci\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_MODE,
		&path->path_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d mode\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_SKIP_NUM,
		&path->skip_num);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d skip num\n", path_index);
		goto exit;
	}

	memset(&regular_info, 0, sizeof(regular_info));
	regular_info.regular_mode = path->regular_desc.regular_mode;
	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_SHRINK,
		&regular_info);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d shrink\n", path_index);
		goto exit;
	}

	param = 1;
	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable path %d\n", path_index);
		goto exit;
	}

	endian.y_endian = path->end_sel.y_endian;
	endian.uv_endian = path->end_sel.uv_endian;
	ret = set_isp_path_cfg(*handle, path_index, ISP_PATH_DATA_ENDIAN,
		&endian);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d data endian\n", path_index);
		goto exit;
	}

	param = dev->cam_ctx.need_3dnr;
	ret = set_isp_path_cfg(*handle, path_index, ISP_NR3_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d need_3dnr\n", path_index);
		goto exit;
	}

	me_conv_size.w = dev->cam_ctx.cap_in_size.w;
	me_conv_size.h = dev->cam_ctx.cap_in_size.h;
	ret = set_isp_path_cfg(*handle, path_index, ISP_NR3_ME_CONV_SIZE,
			&me_conv_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d me_conv size\n", path_index);
		goto exit;
	}

	sns_size.w = dev->cam_ctx.sn_max_size.w;
	sns_size.h = dev->cam_ctx.sn_max_size.h;
	ret = set_isp_path_cfg(*handle, path_index, ISP_SNS_MAX_SIZE,
		&sns_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d cap_size\n", path_index);
		goto exit;
	}

exit:
	return ret;
}

static int sprd_isp_path_cfg(struct camera_dev *dev)
{
	int ret = 0, path_not_work = 0;
	struct camera_info *info = NULL;
	struct cam_path_info path_info;
	struct camera_path_spec *path_pre = NULL;
	struct camera_path_spec *path_vid = NULL;
	struct camera_path_spec *path_cap = NULL;

	info = &dev->cam_ctx;
	path_pre = &info->cam_path[CAMERA_PRE_PATH];
	path_vid = &info->cam_path[CAMERA_VID_PATH];
	path_cap = &info->cam_path[CAMERA_CAP_PATH];
	memset((void *)&path_info, 0, sizeof(path_info));
	path_info.is_slow_motion = info->is_slow_motion;

	ret = sprd_img_isp_reg_isr(dev);
	if (unlikely(ret)) {
		pr_err("fail to register isp isr\n");
		goto exit;
	}

	do {
		/* config isp pre path */
		if (dev->use_path && path_pre->is_work) {
			ret = sprd_isp_path_cfg_block(path_pre,
				&dev->isp_dev_handle, ISP_PATH_IDX_PRE);
			if (unlikely(ret)) {
				pr_err("fail to config path_pre\n");
				break;
			}
			path_pre->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_PRE, ISP_PATH_ENABLE,
				&path_not_work);
			if (unlikely(ret)) {
				pr_err("fail to config isp path pre\n");
				break;
			}
		}

		/* config isp vid path*/
		if (dev->use_path && path_vid->is_work) {
			ret = sprd_isp_path_cfg_block(path_vid,
				&dev->isp_dev_handle, ISP_PATH_IDX_VID);
			if (unlikely(ret)) {
				pr_err("fail to config path_vid\n");
				break;
			}
			path_vid->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_VID, ISP_PATH_ENABLE,
				&path_not_work);
			if (unlikely(ret)) {
				pr_err("fail to config isp path vid\n");
				break;
			}
		}

		/* config isp cap path*/
		if (dev->use_path && path_cap->is_work) {
			ret = sprd_isp_path_cfg_block(path_cap,
				&dev->isp_dev_handle, ISP_PATH_IDX_CAP);
			if (unlikely(ret)) {
				pr_err("fail to config path_cap");
				break;
			}
			path_cap->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_CAP, ISP_PATH_ENABLE,
				&path_not_work);
			if (unlikely(ret)) {
				pr_err("fail to config isp path cap\n");
				break;
			}
		}

		ret = set_isp_path_cfg(dev->isp_dev_handle,
			ISP_PATH_IDX_PRE, ISP_DUAL_CAM_EN,
			&info->dual_cam);
		if (unlikely(ret)) {
			pr_err("fail to config isp path pre\n");
			break;
		}
	} while (0);

	pr_debug("end isp path cfg\n");
exit:
	return ret;
}

static int sprd_camera_stream_on(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct flash_led_task *flash_task = NULL;

	CAM_TRACE("stream on\n");
	pr_info("stream on in\n");

	group = camerafile->grp;
	idx = camerafile->idx;
	flash_task = dev->flash_task;

	mutex_lock(&dev->cam_mutex);

	ret = sprd_camera_update_clk(dev,
		group->pdev->dev.of_node);
	if (unlikely(ret != 0)) {
		mutex_unlock(&dev->cam_mutex);
		pr_err("fail to update clk\n");
		goto exit;
	}

	ret = sprd_cam_buf_queue_clear(&dev->queue);
	if (unlikely(ret != 0)) {
		mutex_unlock(&dev->cam_mutex);
		pr_err("fail to clear queue\n");
		goto exit;
	}
	ret = sprd_cfg_dcam_isp_pipeline(camerafile);
	if (unlikely(ret)) {
		pr_err("fail to config isp path mode\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	ret = sprd_dcam_cfg(dev);
	if (unlikely(ret)) {
		mutex_unlock(&dev->cam_mutex);
		pr_err("fail to config dcam param\n");
		goto exit;
	}

	ret = sprd_isp_path_cfg(dev);
	if (unlikely(ret)) {
		mutex_unlock(&dev->cam_mutex);
		pr_err("fail to config isp path\n");
		goto exit;
	}

	flash_task->frame_skipped = 0;
	flash_task->skip_number = dev->cam_ctx.skip_number;
	if ((flash_task->set_flash.led0_ctrl &&
		flash_task->set_flash.led0_status == FLASH_HIGH_LIGHT) ||
		(flash_task->set_flash.led1_ctrl &&
		flash_task->set_flash.led1_status == FLASH_HIGH_LIGHT)) {
		if (dev->cam_ctx.skip_number == 0)
			dcam_start_flash(NULL, flash_task);
	}

	ret = sprd_isp_start(dev->isp_dev_handle);
	if (unlikely(ret)) {
		mutex_unlock(&dev->cam_mutex);
		pr_err("fail to start isp path\n");
		goto exit;
	}

	ret = sprd_dcam_start(idx);
	if (unlikely(ret)) {
		pr_err("fail to start dcam\n");
		ret = sprd_img_isp_unreg_isr(dev);
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	} else {
		atomic_set(&dev->run_flag, 0);
		sprd_start_timer(&dev->cam_timer, DCAM_TIMEOUT);
	}

	atomic_set(&dev->stream_on, 1);
	mutex_unlock(&dev->cam_mutex);

	pr_info("stream on end\n");
exit:
	return ret;
}

static int sprd_camera_stream_off(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_group *group = NULL;
	struct camera_path_spec *full_path = NULL;
	struct camera_path_spec *bin_path = NULL;
	struct camera_path_spec *path_pdaf = NULL;
	enum chip_id id = SHARKL3;

	CAM_TRACE("stream off\n");
	pr_info("stream off in\n");

	group = camerafile->grp;
	idx = camerafile->idx;

	mutex_lock(&dev->cam_mutex);

	full_path = &dev->cam_ctx.cam_path[CAMERA_FULL_PATH];
	bin_path = &dev->cam_ctx.cam_path[CAMERA_BIN_PATH];
	path_pdaf = &dev->cam_ctx.cam_path[CAMERA_PDAF_PATH];

	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		pr_info("stream not on, idx: %d\n", idx);
		ret = sprd_deinit_handle(dev);
		if (unlikely(ret))
			pr_err("fail to local deinit\n");

		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	do {
		ret = sprd_stop_timer(&dev->cam_timer);
		if (unlikely(ret)) {
			pr_err("fail to stop timer\n");
			break;
		}
		ret = sprd_dcam_stop(idx, 0);
		if (unlikely(ret)) {
			pr_err("fail to stop dcam\n");
			break;
		}
		ret = sprd_img_dcam_unreg_isr(dev);
		if (unlikely(ret)) {
			pr_err("fail to unregister isr\n");
			break;
		}

		if (dev->cam_ctx.need_3dnr) {
			ret = isp_3dnr_release(dev->isp_dev_handle);
			if (unlikely(ret)) {
				pr_err("fail to release 3ndr\n");
				break;
			}
		}

		ret = sprd_isp_stop(dev->isp_dev_handle, 0);
		if (unlikely(ret)) {
			pr_err("fail to stop isp\n");
			break;
		}

		ret = sprd_img_isp_unreg_isr(dev);
		if (unlikely(ret)) {
			pr_err("fail to unregister isp isr\n");
			break;
		}

		ret = sprd_dcam_module_deinit(idx);
		if (unlikely(ret)) {
			pr_err("fail to deinit dcam module\n");
			break;
		}

		id = sprd_dcam_get_chip_id();
		atomic_set(&dev->stream_on, 0);

		if (atomic_read(&group->dev[DCAM_ID_0]->stream_on) == 0
			&& atomic_read(&group->dev[DCAM_ID_1]->stream_on)
			== 0) {
			if (id == SHARKL3) {
				if (atomic_read
					(&group->dev[DCAM_ID_2]->stream_on)
					== 0)
					cam_buf_put_sg_table();
			} else {
				cam_buf_put_sg_table();
			}
			sprd_iommu_restore(&s_dcam_pdev->dev);
			sprd_iommu_restore(&s_isp_pdev->dev);
		}

		ret = sprd_deinit_handle(dev);
		if (unlikely(ret)) {
			pr_err("fail to local deinit\n");
			break;
		}
	} while (0);

	mutex_unlock(&dev->cam_mutex);
	pr_info("camera stream off end.\n");
exit:
	return ret;
}

static int sprd_img_io_cfg_flash(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_flash_cfg_param cfg_parm;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&cfg_parm,
				(void __user *)arg,
				sizeof(struct sprd_flash_cfg_param));
	if (ret) {
		pr_err("fail to copy from user %d\n", ret);
		mutex_unlock(&dev->cam_mutex);
		ret = -EFAULT;
		return ret;
	}
	ret = sprd_flash_cfg(&cfg_parm);
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("config flash, ret %d\n", ret);
	return ret;
}

static int sprd_img_io_get_time(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_time utime;
	struct timeval time;

	CAM_TRACE("get time\n");
	cam_get_timestamp(&time);
	utime.sec = time.tv_sec;
	utime.usec = time.tv_usec;
	ret = copy_to_user((void __user *)arg, &utime,
				sizeof(struct sprd_img_time));
	if (ret) {
		pr_err("fail to get time info %d\n", ret);
		ret = -EFAULT;
	}
	return ret;
}

static int sprd_dcam_start_fetch(enum dcam_id idx, enum dcam_id fetch_idx,
	struct camera_group *group)
{
	int ret = 0;
	uint32_t param = 0;
	size_t image_size;
	struct camera_dev *dev = group->dev[idx];
	struct camera_rect rect;
	struct camera_addr cam_addr = {0};
	struct camera_path_spec *path;
	struct cam_buf_info buf_info;
	struct isp_img_size size;
	uint32_t pattern = 0;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		return ret;
	}

	/*enable dcam 1*/
	if (idx != fetch_idx ||
		!(group->mode_inited & (1 << fetch_idx))) {

		ret = sprd_dcam_module_en(fetch_idx);
		if (unlikely(ret != 0)) {
			pr_err("fail to enable dcam module %d\n", fetch_idx);
			goto exit;
		}
		pr_info("dcam%d has been enabled!\n", fetch_idx);
	}
	ret = sprd_dcam_reg_isr(fetch_idx, DCAM_BIN_PATH_TX_DONE,
		sprd_img_bin_tx_done, group->dev[fetch_idx]);
	if (unlikely(ret)) {
		pr_err("fail to register dcam isr\n");
		goto exit;
	}
	group->fetch_inited |= 1 << fetch_idx;

	/*config fetch*/
	path = &group->dev[fetch_idx]->cam_ctx.cam_path[CAMERA_BIN_PATH];
	path->is_work = 1;
	path->status = PATH_RUN;
	path->assoc_idx = 1 << CAMERA_CAP_PATH;

	sprd_cam_buf_queue_init(&path->buf_queue, CAM_QUEUE_NO_LOCK,
		sizeof(struct camera_addr), DCAM_FRM_QUEUE_LENGTH,
		"bin path (cam_core for isp)");

	size = dev->fetch_info.size;
	image_size = dcam_calc_raw10_line_bytes(0, size.width) * size.height;
	cam_buf_alloc_k(&buf_info, &s_dcam_pdev->dev,
		image_size, 1, CAM_BUF_SWAP_TYPE);
	cam_addr.type = buf_info.type;
	cam_addr.p_dev = &group->pdev->dev;
	cam_addr.client[0] = buf_info.client[0];
	cam_addr.handle[0] = buf_info.handle[0];
	cam_addr.num = buf_info.num;
	sprd_cam_buf_queue_write(&path->buf_queue,
		&cam_addr);

	path->in_size.w = size.width;
	path->in_size.h = size.height;
	path->in_rect.x = 0;
	path->in_rect.y = 0;
	path->in_rect.w = size.width;
	path->in_rect.h = size.height;
	path->out_size = path->in_size;
	path->out_fmt = DCAM_RAWRGB;
	path->pixel_depth = 10;
	pattern = dev->cam_ctx.img_ptn;

	dcam_bin_path_init(fetch_idx);

	ret = sprd_dcam_cfg_bin_path(path, fetch_idx);
	if (unlikely(ret)) {
		pr_err("fail to config full_path cap");
		goto exit;
	}

#if 1
	if (1/*full_path->src_sel == DCAM_PATH_FROM_CAP*/) {
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG,
			0x0f << 16, pattern << 16);

		DCAM_REG_MWR(idx, ISP_AEM_PARAM, 0x1, 1);
		DCAM_REG_MWR(idx, DCAM_NR3_PARA1, 0xF, 1);

		DCAM_REG_MWR(idx, 0x158,
			0x07 << 0, 0x01 << 0);/*bpc and gc bypass*/
		DCAM_REG_MWR(idx, 0x15c,
			0x0F << 0, 0x0F << 0);/*bpc bypass*/
		DCAM_REG_MWR(idx, 0x168,
			0x01 << 0, 0x01 << 0);/*gc bypass*/
	} else {
		DCAM_REG_MWR(idx, 0x158,
			0x07 << 0, 0x06 << 0);/*bpc and gc enable*/
		DCAM_REG_MWR(idx, 0x168,
			0x01 << 0, 0x00 << 0);/*gc enable*/
	}
	dcam_force_copy(idx, FULL_COPY | AEM_COPY);
#endif

	dcam_start_bin_path(fetch_idx);

	/*config fetch*/
	param = 0;
	ret = set_dcam_fetch_cfg(fetch_idx, DCAM_FETCH_DATA_PACKET, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable full_path\n");
		goto exit;
	}

	rect.x = 0;
	rect.y = 0;
	rect.w = size.width;
	rect.h = size.height;
	ret = set_dcam_fetch_cfg(fetch_idx, DCAM_FETCH_INPUT_RECT,
		&rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path input rect\n");
		goto exit;
	}

	param = 0;
	ret = set_dcam_fetch_cfg(fetch_idx, DCAM_FETCH_DATA_ENDIAN,
		&param);
	if (unlikely(ret)) {
		pr_err("fail to cfg full_path data endian\n");
		goto exit;
	}

	memset(&cam_addr, 0x00, sizeof(cam_addr));
	cam_addr.type = CAM_BUF_USER_TYPE;
	cam_addr.p_dev = &group->pdev->dev;
	cam_addr.mfd_y = dev->fetch_info.fetch_addr.img_fd;
	cam_addr.yaddr = dev->fetch_info.fetch_addr.offset.x;
	cam_addr.num = 1;
	ret = set_dcam_fetch_cfg(fetch_idx, DCAM_FETCH_INPUT_ADDR,
		&cam_addr);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam fetch source addr\n");
		goto exit;
	}

	/*start dcam1 and isp fetch flow*/
	param = 1;
	ret = set_dcam_fetch_cfg(fetch_idx, DCAM_FETCH_START,
		&param);

	pr_info("size mfd offset: 0x%x 0x%x, %dx%d, fetch id %d\n",
		dev->fetch_info.fetch_addr.img_fd,
		dev->fetch_info.fetch_addr.offset.x,
		size.width, size.height, fetch_idx);
exit:
	return ret;
}

static int sprd_dcam_stop_fetch(enum dcam_id idx, enum dcam_id fetch_idx,
	struct camera_group *group)
{
	int ret = 0;
	uint32_t param = 0;
	struct camera_dev *dev = group->dev[idx];
	struct camera_path_spec *path;
	struct dcam_module *dcam_dev = NULL;

	pr_info("In");
	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		return ret;
	}

	ret = set_dcam_fetch_cfg(fetch_idx, DCAM_FETCH_START,
		&param);

	path = &group->dev[fetch_idx]->cam_ctx.cam_path[CAMERA_BIN_PATH];
	sprd_cam_buf_queue_deinit(&path->buf_queue);

	dcam_dev = get_dcam_module(fetch_idx);
	cam_buf_unmap_addr(&dcam_dev->dcam_fetch.frame.buf_info);

	/*dcam_quickstop_bin_path(fetch_idx);*/
	dcam_bin_path_deinit(fetch_idx);
	path->is_work = 0;
	path->status = PATH_IDLE;
	path->assoc_idx = 0;

	if (idx != fetch_idx ||
		!(group->mode_inited & group->fetch_inited)) {
		ret = sprd_dcam_module_dis(fetch_idx);
		if (unlikely(ret != 0)) {
			pr_err("fail to disable dcam%d module\n", idx);
			ret = -EFAULT;
		}
	}
	ret = sprd_dcam_reg_isr(fetch_idx, DCAM_BIN_PATH_TX_DONE,
		NULL, group->dev[fetch_idx]);
	if (unlikely(ret)) {
		pr_err("fail to register dcam isr\n");
		goto exit;
	}
	group->fetch_inited &= ~(1 << fetch_idx);
	pr_info("fetch stop end!\n");

exit:
	return ret;
}

static int sprd_dcam_out_size(struct camera_dev *dev,
	struct camera_info *info,
	enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_dcam_path_size parm;

	CAM_TRACE("%d: dcam out size\n", idx);
	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
		sizeof(struct sprd_dcam_path_size));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	if ((!parm.dcam_in_w) || (!parm.dcam_in_h)) {
		pr_err("fail to get dcam in size %d %d\n",
			parm.dcam_in_w, parm.dcam_in_h);
		mutex_unlock(&dev->cam_mutex);
		return -EFAULT;
	}

	parm.dcam_out_w = parm.dcam_in_w;
	parm.dcam_out_h = parm.dcam_in_h;

	ret = copy_to_user((void __user *)arg, &parm,
		sizeof(struct sprd_dcam_path_size));
	if (ret) {
		pr_err("fail to copy to user, ret = %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}
static int sprd_img_io_set_mode(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	uint32_t mode = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&mode, (void __user *) arg,
			sizeof(uint32_t));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	info->capture_mode = mode;
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: capture mode %d\n", idx,
		dev->cam_ctx.capture_mode);
exit:
	return ret;
}

static int sprd_img_io_set_cap_skip_num(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	uint32_t skip_num = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&skip_num, (void __user *)arg,
			sizeof(uint32_t));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	info->skip_number = skip_num;
	mutex_unlock(&dev->cam_mutex);
	CAM_TRACE("%d: cap skip number %d\n", idx,
		dev->cam_ctx.skip_number);
exit:
	return ret;
}

static int sprd_img_io_get_ch_id(struct camera_file *camerafile,
		struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id = 0;
	struct camera_info *info = NULL;

	info = &dev->cam_ctx;

	CAM_TRACE("get free channel\n");
	sprd_img_get_free_channel(dev, &channel_id, info->scene_mode);
	ret = copy_to_user((void __user *)arg, &channel_id,
			sizeof(uint32_t));
	if (ret) {
		pr_err("fail to copy to user, ret = %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_io_set_flash(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	uint32_t led0_ctrl = 0, led1_ctrl = 0;
	uint32_t led0_status = 0, led1_status = 0;
	struct flash_led_task *flash_task = NULL;

	flash_task = dev->flash_task;
	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&flash_task->set_flash,
			(void __user *)arg,
			sizeof(struct sprd_img_set_flash));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	led0_ctrl = flash_task->set_flash.led0_ctrl;
	led1_ctrl = flash_task->set_flash.led1_ctrl;
	led0_status = flash_task->set_flash.led0_status;
	led1_status = flash_task->set_flash.led1_status;

	mutex_unlock(&dev->cam_mutex);
	if ((led0_ctrl &&
		(led0_status == FLASH_CLOSE_AFTER_OPEN ||
		led0_status == FLASH_CLOSE ||
		led0_status == FLASH_CLOSE_AFTER_AUTOFOCUS)) ||
		(led1_ctrl &&
		(led1_status == FLASH_CLOSE_AFTER_OPEN ||
		led1_status == FLASH_CLOSE ||
		led1_status == FLASH_CLOSE_AFTER_AUTOFOCUS))) {
		complete(&flash_task->flash_thread_com);
	}

	CAM_TRACE("led0_ctrl %d led0_status %d\n",
		flash_task->set_flash.led0_ctrl,
		flash_task->set_flash.led0_status);
	CAM_TRACE("led1_ctrl %d led1_status %d\n",
		flash_task->set_flash.led1_ctrl,
		flash_task->set_flash.led1_status);
exit:
	return ret;
}

static int sprd_img_io_start_capture(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct timeval tv;
	enum dcam_id idx = DCAM_ID_0;
	struct sprd_img_capture_param capture_param;

	idx = camerafile->idx;

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&capture_param, (void __user *)arg,
			sizeof(struct sprd_img_capture_param));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		pr_info("stream not on\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	cam_get_timestamp(&tv);
	capture_param.timestamp = tv.tv_sec * 1000000000LL
		+ tv.tv_usec * 1000;
	pr_info("cam%d capture start: cap_flag = %d\n", idx,
		dev->cap_flag);
	if (dev->cap_flag == CAMERA_CAPTURE_STOP &&
			dev->cam_ctx.need_isp_tool != 1) {
		ret = sprd_isp_start_fmcu(dev->isp_dev_handle,
			capture_param, 0, ISP_PATH_IDX_CAP);
		if (ret) {
			mutex_unlock(&dev->cam_mutex);
			pr_err("fail to start offline\n");
			goto exit;
		}
		dev->cap_flag = CAMERA_CAPTURE_START;
	}
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_io_stop_capture(struct camera_file *camerafile,
				struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;

	idx = camerafile->idx;

	mutex_lock(&dev->cam_mutex);
	pr_info("cam%d capture stop\n", idx);
	if (dev->cap_flag == CAMERA_CAPTURE_START) {
		dev->cap_flag = CAMERA_CAPTURE_STOP;
		ret = sprd_isp_fmcu_slice_stop(dev->isp_dev_handle);
		if (ret) {
			mutex_unlock(&dev->cam_mutex);
			pr_err("fail to stop offline\n");
			goto exit;
		}
	}
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_io_get_iommu_status(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	uint32_t iommu_enable = 0;
	struct camera_group *group = NULL;

	group = camerafile->grp;

	ret = copy_from_user(&iommu_enable, (void __user *)arg,
			sizeof(unsigned char));
	if (ret) {
		pr_err("fail to copy from user\n");
		ret = -EFAULT;
		goto exit;
	}

	if (sprd_iommu_attach_device(&group->pdev->dev) == 0)
		iommu_enable = 1;
	else
		iommu_enable = 0;

	ret = copy_to_user((void __user *)arg, &iommu_enable,
			sizeof(unsigned char));
	if (ret) {
		pr_err("fail to copy to user, ret = %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
exit:
	return ret;
}

static int sprd_isp_io_set_statis_buf(struct camera_file *camerafile,
	struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct isp_statis_buf_input parm_inptr;
	struct dcam_module *dcam_dev = NULL;
	struct camera_group *group = NULL;

	group = camerafile->grp;

	dcam_dev = get_dcam_module(dev->idx);

	mutex_lock(&dev->cam_mutex);
	ret = copy_from_user(&parm_inptr,
				(void __user *)arg,
				sizeof(struct isp_statis_buf_input));
	if (ret != 0) {
		pr_err("fail to copy_from_user\n");
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	pr_debug("statis buf_flag %d\n", parm_inptr.buf_flag);
	if (parm_inptr.buf_flag == STATIS_BUF_FLAG_INIT)
		dev->init_inptr = parm_inptr;
	else
		ret = sprd_cam_set_statis_addr(&group->pdev->dev,
			&dcam_dev->statis_module_info, &parm_inptr);
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_isp_io_capability(struct camera_file *camerafile,
		struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;

	mutex_lock(&dev->cam_mutex);
	ret = isp_capability((void *)arg);
	mutex_unlock(&dev->cam_mutex);
	return ret;
}
static int sprd_isp_io_cfg_start(struct camera_file *camerafile,
		struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct isp_pipe_dev *isp_dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	mutex_lock(&dev->cam_mutex);
	if (!dev->isp_dev_handle) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	isp_dev = (struct isp_pipe_dev *)dev->isp_dev_handle;
	isp_k_param = &isp_dev->isp_k_param;
	if (!isp_k_param) {
		ret = -EFAULT;
		pr_err("fail to get isp_private.\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	isp_k_param->lsc_2d_weight_en = 0;
	isp_k_param->lsc_updated = 0;
	isp_k_param->param_update_flag = 0;
	isp_k_param->lsc_load_buf_id_prv = 1;
	isp_k_param->lsc_load_buf_id_cap = 1;
	isp_k_param->isp_status = 0;
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_isp_io_cfg_param(struct camera_file *camerafile,
		struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_group *group = NULL;
	struct isp_io_param param;
	struct isp_pipe_dev *isp_dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	idx = camerafile->idx;
	group = camerafile->grp;

	mutex_lock(&dev->cam_mutex);

	ret = copy_from_user((void *)&param,
		(void __user *)arg, sizeof(param));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n",
			ret);
		ret = -EFAULT;
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	if (param.sub_block == ISP_BLOCK_FETCH) {
		if (param.property ==
				ISP_PRO_FETCH_RAW_BLOCK) {
			struct isp_dev_fetch_info *fetch_info =
			&dev->fetch_info;

			ret = copy_from_user((void *)fetch_info,
			param.property_param,
			sizeof(struct isp_dev_fetch_info));
			if (ret != 0)
				pr_err("fail to copy user\n");
		} else if (dev->raw_cap == 1
			&& param.property ==
				ISP_PRO_FETCH_START) {
			ret = sprd_dcam_start_fetch(idx,
				idx, group);
			mutex_unlock(&dev->cam_mutex);
			goto exit;
		}
	}

	if (!dev->isp_dev_handle) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	isp_dev = (struct isp_pipe_dev *)dev->isp_dev_handle;
	isp_k_param = &isp_dev->isp_k_param;
	if (!isp_k_param) {
		ret = -EFAULT;
		pr_err("fail to get isp_private.\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	ret = isp_cfg_param((void *)arg, isp_k_param, isp_dev);
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_isp_io_update_param_start(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct isp_pipe_dev *isp_dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	mutex_lock(&dev->cam_mutex);

	if (!dev->isp_dev_handle) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	isp_dev = (struct isp_pipe_dev *)dev->isp_dev_handle;
	isp_k_param = &isp_dev->isp_k_param;
	if (!isp_k_param) {
		ret = -EFAULT;
		pr_err("fail to get isp_private.\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	isp_k_param->param_update_flag++;
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_isp_io_update_param_end(struct camera_file *camerafile,
			struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct isp_pipe_dev *isp_dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	mutex_lock(&dev->cam_mutex);

	if (!dev->isp_dev_handle) {
		pr_err("fail to get valid input ptr\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	isp_dev = (struct isp_pipe_dev *)dev->isp_dev_handle;
	isp_k_param = &isp_dev->isp_k_param;
	if (!isp_k_param) {
		ret = -EFAULT;
		pr_err("fail to get isp_private.\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	isp_k_param->param_update_flag--;
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_isp_io_raw_cap(struct camera_file *camerafile,
	struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct isp_pipe_dev *isp_dev = NULL;
	struct isp_raw_proc_info raw_cap;

	idx = camerafile->idx;

	mutex_lock(&dev->cam_mutex);
	pr_info("raw_cap, idx: %d, %d\n",
		idx, atomic_read(&dev->stream_on));
	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		ret = sprd_cam_buf_queue_clear(&dev->queue);
		if (unlikely(ret != 0)) {
			mutex_unlock(&dev->cam_mutex);
			pr_err("fail to init queue\n");
			goto exit;
		}
		ret = sprd_img_isp_reg_isr(dev);
		if (unlikely(ret)) {
			pr_err("fail to register isp isr\n");
			mutex_unlock(&dev->cam_mutex);
			goto exit;
		}
		atomic_set(&dev->stream_on, 1);
	}
	dev->raw_cap = 1;
	dev->raw_phase = 0;
	pr_info("raw_cap, idx: %d %d\n", idx, dev->raw_cap);

	if (!dev->isp_dev_handle) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}

	isp_dev = (struct isp_pipe_dev *)dev->isp_dev_handle;
	mutex_lock(&isp_dev->isp_mutex);
	pr_info("start isp_raw_cap_proc %p\n", isp_dev);
	memset((void *)&raw_cap, 0x00, sizeof(raw_cap));
	ret = copy_from_user(&raw_cap,
			(void __user *)arg,
			sizeof(struct isp_raw_proc_info));
	if (ret != 0) {
		pr_err("fail to copy_from_user\n");
		ret = -EFAULT;
		mutex_unlock(&isp_dev->isp_mutex);
		mutex_unlock(&dev->cam_mutex);
		goto exit;
	}
	ret = isp_raw_cap_proc(isp_dev, &raw_cap);
	mutex_unlock(&isp_dev->isp_mutex);
	mutex_unlock(&dev->cam_mutex);
exit:
	return ret;
}

static int sprd_img_io_dcam_path_size(struct camera_file *camerafile,
	struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	idx = camerafile->idx;
	info = &dev->cam_ctx;

	ret = sprd_dcam_out_size(dev, info, idx, arg);
	if (ret) {
		pr_err("fail to get dcam out size\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_isp_io_rst(struct camera_file *camerafile,
	struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	enum dcam_id idx = DCAM_ID_0;

	idx = camerafile->idx;

	dcam_quickstop_full_path(idx);
	sprd_dcam_reset(idx, 0);

	return ret;
}

static struct dcam_io_ctrl_fun s_cam_io_ctrl_fun_tab[] = {
	{SPRD_IMG_IO_SET_MODE, sprd_img_io_set_mode},
	{SPRD_IMG_IO_SET_CAP_SKIP_NUM, sprd_img_io_set_cap_skip_num},
	{SPRD_IMG_IO_SET_SENSOR_SIZE, sprd_img_io_set_sensor_size},
	{SPRD_IMG_IO_SET_SENSOR_TRIM, sprd_img_io_set_sensor_trim},
	{SPRD_IMG_IO_SET_FRM_ID_BASE, sprd_img_io_set_frm_id_base},
	{SPRD_IMG_IO_SET_CROP, sprd_img_io_set_crop},
	{SPRD_IMG_IO_SET_FLASH, sprd_img_io_set_flash},
	{SPRD_IMG_IO_SET_OUTPUT_SIZE, sprd_img_io_set_output_size},
	{SPRD_IMG_IO_SET_ZOOM_MODE, NULL},
	{SPRD_IMG_IO_SET_SENSOR_IF, sprd_img_io_set_sensor_if},
	{SPRD_IMG_IO_SET_FRAME_ADDR, sprd_img_io_set_frame_addr},
	{SPRD_IMG_IO_PATH_FRM_DECI, sprd_img_io_path_frm_deci},
	{SPRD_IMG_IO_PATH_PAUSE, NULL},
	{SPRD_IMG_IO_PATH_RESUME, NULL},
	{SPRD_IMG_IO_STREAM_ON, sprd_camera_stream_on},
	{SPRD_IMG_IO_STREAM_OFF, sprd_camera_stream_off},
	{SPRD_IMG_IO_GET_FMT, sprd_img_io_get_fmt},
	{SPRD_IMG_IO_GET_CH_ID, sprd_img_io_get_ch_id},
	{SPRD_IMG_IO_GET_TIME, sprd_img_io_get_time},
	{SPRD_IMG_IO_CHECK_FMT, sprd_img_io_check_fmt},
	{SPRD_IMG_IO_SET_SHRINK, sprd_img_io_set_shrink},
	{SPRD_IMG_IO_CFG_FLASH, sprd_img_io_cfg_flash},
	{SPRD_IMG_IO_PDAF_CONTROL, sprd_img_io_pdaf_control},
	{SPRD_IMG_IO_GET_IOMMU_STATUS, sprd_img_io_get_iommu_status},
	{SPRD_IMG_IO_START_CAPTURE, sprd_img_io_start_capture},
	{SPRD_IMG_IO_STOP_CAPTURE, sprd_img_io_stop_capture},
	{SPRD_IMG_IO_SET_PATH_SKIP_NUM, sprd_img_io_set_path_skip_num},
	{SPRD_IMG_IO_SBS_MODE, NULL},
	{SPRD_IMG_IO_DCAM_PATH_SIZE, sprd_img_io_dcam_path_size},
	{SPRD_IMG_IO_SET_SENSOR_MAX_SIZE, sprd_img_io_set_sensor_max_size},
	{SPRD_ISP_IO_CAPABILITY, sprd_isp_io_capability},
	{SPRD_ISP_IO_SET_STATIS_BUF, sprd_isp_io_set_statis_buf},
	{SPRD_ISP_IO_CFG_PARAM, sprd_isp_io_cfg_param},
	{SPRD_ISP_IO_RAW_CAP, sprd_isp_io_raw_cap},
	{SPRD_IMG_IO_GET_DCAM_RES, sprd_img_io_get_dcam_res},
	{SPRD_IMG_IO_PUT_DCAM_RES, sprd_img_io_put_dcam_res},
	{SPRD_ISP_IO_CFG_START, sprd_isp_io_cfg_start},
	{SPRD_IMG_IO_SET_FUNCTION_MODE, sprd_img_io_set_function_mode},
	{SPRD_ISP_IO_UPDATE_PARAM_START, sprd_isp_io_update_param_start},
	{SPRD_ISP_IO_UPDATE_PARAM_END, sprd_isp_io_update_param_end},
	{SPRD_ISP_IO_RST, sprd_isp_io_rst}
};

static dcam_io_fun cam_ioctl_get_fun(uint32_t cmd)
{
	dcam_io_fun io_ctrl = NULL;
	int total_num = 0;
	int i = 0;

	total_num = sizeof(s_cam_io_ctrl_fun_tab) /
		sizeof(struct dcam_io_ctrl_fun);
	for (i = 0; i < total_num; i++) {
		if (cmd == s_cam_io_ctrl_fun_tab[i].cmd) {
			io_ctrl = s_cam_io_ctrl_fun_tab[i].io_ctrl;
			break;
		}
	}

	return io_ctrl;
}
#endif


