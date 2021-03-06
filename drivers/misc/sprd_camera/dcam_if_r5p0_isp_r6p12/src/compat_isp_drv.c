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

#include "compat_isp_drv.h"

const char *cam_ioctl_str[] = {
	"SPRD_IMG_IO_SET_MODE",
	"SPRD_IMG_IO_SET_CAP_SKIP_NUM",
	"SPRD_IMG_IO_SET_SENSOR_SIZE",
	"SPRD_IMG_IO_SET_SENSOR_TRIM",
	"SPRD_IMG_IO_SET_FRM_ID_BASE",
	"SPRD_IMG_IO_SET_CROP",
	"SPRD_IMG_IO_SET_FLASH",
	"SPRD_IMG_IO_SET_OUTPUT_SIZE",
	"SPRD_IMG_IO_SET_ZOOM_MODE",
	"SPRD_IMG_IO_SET_SENSOR_IF",
	"SPRD_IMG_IO_SET_FRAME_ADDR",
	"SPRD_IMG_IO_PATH_FRM_DECI",
	"SPRD_IMG_IO_PATH_PAUSE",
	"SPRD_IMG_IO_PATH_RESUME",
	"SPRD_IMG_IO_STREAM_ON",
	"SPRD_IMG_IO_STREAM_OFF",
	"SPRD_IMG_IO_GET_FMT",
	"SPRD_IMG_IO_GET_CH_ID",
	"SPRD_IMG_IO_GET_TIME",
	"SPRD_IMG_IO_CHECK_FMT",
	"SPRD_IMG_IO_SET_SHRINK",
	"SPRD_IMG_IO_SET_FREQ_FLAG",
	"SPRD_IMG_IO_CFG_FLASH",
	"SPRD_IMG_IO_PDAF_CONTROL",
	"SPRD_IMG_IO_GET_IOMMU_STATUS",
	"SPRD_IMG_IO_DISABLE_MODE",
	"SPRD_IMG_IO_ENABLE_MODE",
	"SPRD_IMG_IO_START_CAPTURE",
	"SPRD_IMG_IO_STOP_CAPTURE",
	"SPRD_IMG_IO_SET_PATH_SKIP_NUM",
	"SPRD_IMG_IO_SBS_MODE",
	"SPRD_IMG_IO_DCAM_PATH_SIZE",
	"SPRD_IMG_IO_SET_SENSOR_MAX_SIZE",
	"SPRD_ISP_IO_CAPABILITY",
	"SPRD_ISP_IO_IRQ",
	"SPRD_ISP_IO_READ",
	"SPRD_ISP_IO_WRITE",
	"SPRD_ISP_IO_RST",
	"SPRD_ISP_IO_STOP",
	"SPRD_ISP_IO_INT",
	"SPRD_ISP_IO_SET_STATIS_BUF",
	"SPRD_ISP_IO_CFG_PARAM",
	"SPRD_ISP_REG_READ",
	"SPRD_ISP_IO_POST_3DNR",
	"SPRD_STATIS_IO_CFG_PARAM",
	"SPRD_ISP_IO_RAW_CAP",
	"SPRD_IMG_IO_GET_DCAM_RES",
	"SPRD_IMG_IO_PUT_DCAM_RES",
	"SPRD_ISP_IO_SET_PULSE_LINE",
	"SPRD_ISP_IO_CFG_START",
	"SPRD_ISP_IO_POST_YNR",
	"SPRD_ISP_IO_SET_NEXT_VCM_POS",
	"SPRD_ISP_IO_SET_VCM_LOG",
	"SPRD_IMG_IO_SET_3DNR",
	"SPRD_ISP_IO_MASK_3A",
	"SPRD_IMG_IO_GET_FLASH_INFO",
	"SPRD_IMG_IO_MAP_IOVA",
	"SPRD_IMG_IO_UNMAP_IOVA",
	"SPRD_IMG_IO_GET_SG",
	"SPRD_ISP_IO_UPDATE_PARAM_START",
	"SPRD_ISP_IO_UPDATE_PARAM_END",
	"SPRD_ISP_IO_REG_ISP_ISR",
	"SPRD_IMG_IO_DCAM_PATH_SIZE"
};

const int cam_ioctl_val[] = {
	SPRD_IMG_IO_SET_MODE,
	SPRD_IMG_IO_SET_CAP_SKIP_NUM,
	SPRD_IMG_IO_SET_SENSOR_SIZE,
	SPRD_IMG_IO_SET_SENSOR_TRIM,
	SPRD_IMG_IO_SET_FRM_ID_BASE,
	SPRD_IMG_IO_SET_CROP,
	SPRD_IMG_IO_SET_FLASH,
	SPRD_IMG_IO_SET_OUTPUT_SIZE,
	SPRD_IMG_IO_SET_ZOOM_MODE,
	SPRD_IMG_IO_SET_SENSOR_IF,
	SPRD_IMG_IO_SET_FRAME_ADDR,
	SPRD_IMG_IO_PATH_FRM_DECI,
	SPRD_IMG_IO_PATH_PAUSE,
	SPRD_IMG_IO_PATH_RESUME,
	SPRD_IMG_IO_STREAM_ON,
	SPRD_IMG_IO_STREAM_OFF,
	SPRD_IMG_IO_GET_FMT,
	SPRD_IMG_IO_GET_CH_ID,
	SPRD_IMG_IO_GET_TIME,
	SPRD_IMG_IO_CHECK_FMT,
	SPRD_IMG_IO_SET_SHRINK,
	SPRD_IMG_IO_SET_FREQ_FLAG,
	SPRD_IMG_IO_CFG_FLASH,
	SPRD_IMG_IO_PDAF_CONTROL,
	SPRD_IMG_IO_GET_IOMMU_STATUS,
	SPRD_IMG_IO_DISABLE_MODE,
	SPRD_IMG_IO_ENABLE_MODE,
	SPRD_IMG_IO_START_CAPTURE,
	SPRD_IMG_IO_STOP_CAPTURE,
	SPRD_IMG_IO_SET_PATH_SKIP_NUM,
	SPRD_IMG_IO_SBS_MODE,
	SPRD_IMG_IO_DCAM_PATH_SIZE,
	SPRD_IMG_IO_SET_SENSOR_MAX_SIZE,
	SPRD_ISP_IO_CAPABILITY,
	SPRD_ISP_IO_IRQ,
	SPRD_ISP_IO_READ,
	SPRD_ISP_IO_WRITE,
	SPRD_ISP_IO_RST,
	SPRD_ISP_IO_STOP,
	SPRD_ISP_IO_INT,
	SPRD_ISP_IO_SET_STATIS_BUF,
	SPRD_ISP_IO_CFG_PARAM,
	SPRD_ISP_REG_READ,
	SPRD_ISP_IO_POST_3DNR,
	SPRD_STATIS_IO_CFG_PARAM,
	SPRD_ISP_IO_RAW_CAP,
	SPRD_IMG_IO_GET_DCAM_RES,
	SPRD_IMG_IO_PUT_DCAM_RES,
	SPRD_ISP_IO_SET_PULSE_LINE,
	SPRD_ISP_IO_CFG_START,
	SPRD_ISP_IO_POST_YNR,
	SPRD_ISP_IO_SET_NEXT_VCM_POS,
	SPRD_ISP_IO_SET_VCM_LOG,
	SPRD_IMG_IO_SET_3DNR,
	SPRD_IMG_IO_SET_FUNCTION_MODE,
	SPRD_ISP_IO_MASK_3A,
	SPRD_IMG_IO_GET_FLASH_INFO,
	SPRD_IMG_IO_MAP_IOVA,
	SPRD_IMG_IO_UNMAP_IOVA,
	SPRD_IMG_IO_GET_SG,
	SPRD_ISP_IO_UPDATE_PARAM_START,
	SPRD_ISP_IO_UPDATE_PARAM_END,
	SPRD_ISP_IO_REG_ISP_ISR,
	SPRD_IMG_IO_DCAM_PATH_SIZE
};

static int compat_get_set_statis_buf(
		struct compat_isp_statis_buf_input __user *data32,
		struct isp_statis_buf_input __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	compat_ulong_t val;

	err  = get_user(tmp, &data32->buf_size);
	err |= put_user(tmp, &data->buf_size);
	compat_isp_trace("buf_size [0x%x]\n", tmp);

	err |= get_user(tmp, &data32->buf_num);
	err |= put_user(tmp, &data->buf_num);
	compat_isp_trace("buf_num [%d]\n", tmp);

	err |= get_user(val, &data32->phy_addr);
	err |= put_user(val, &data->phy_addr);
	compat_isp_trace("phy_addr [0x%x]\n", val);

	err |= get_user(val, &data32->vir_addr);
	err |= put_user(val, &data->vir_addr);
	compat_isp_trace("vir_addr [0x%x]\n", val);

	err |= get_user(val, &data32->addr_offset);
	err |= put_user(val, &data->addr_offset);
	compat_isp_trace("addr_offset [0x%x]\n", val);

	err |= get_user(tmp, &data32->kaddr[0]);
	err |= put_user(tmp, &data->kaddr[0]);
	compat_isp_trace("kaddr [%d]\n", tmp);

	err |= get_user(tmp, &data32->kaddr[1]);
	err |= put_user(tmp, &data->kaddr[1]);
	compat_isp_trace("kaddr [%d]\n", tmp);

	err |= get_user(val, &data32->mfd);
	err |= put_user(val, &data->mfd);
	compat_isp_trace("mfd [0x%x]\n", val);

	err |= get_user(val, &data32->dev_fd);
	err |= put_user(val, &data->dev_fd);
	compat_isp_trace("dev_fd [0x%x]\n", val);

	err |= get_user(tmp, &data32->buf_property);
	err |= put_user(tmp, &data->buf_property);
	compat_isp_trace("buf_property [%d]\n", tmp);

	err |= get_user(tmp, &data32->buf_flag);
	err |= put_user(tmp, &data->buf_flag);
	compat_isp_trace("buf_flag [%d]\n", tmp);

	err |= get_user(tmp, &data32->is_statis_buf_reserved);
	err |= put_user(tmp, &data->is_statis_buf_reserved);
	compat_isp_trace("is_statis_buf_reserved [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[0]);
	err |= put_user(tmp, &data->reserved[0]);
	compat_isp_trace("reserved[0] [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[1]);
	err |= put_user(tmp, &data->reserved[1]);
	compat_isp_trace("reserved[1] [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[2]);
	err |= put_user(tmp, &data->reserved[2]);
	compat_isp_trace("reserved[2] [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[3]);
	err |= put_user(tmp, &data->reserved[3]);
	compat_isp_trace("reserved[3] [%d]\n", tmp);

	return err;
}

static int compat_get_raw_proc_info(
		struct compat_isp_raw_proc_info __user *data32,
		struct isp_raw_proc_info __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	compat_ulong_t val;

	err  = get_user(tmp, &data32->in_size.width);
	err |= put_user(tmp, &data->in_size.width);
	compat_isp_trace("in_size.width [%d]\n", tmp);

	err  = get_user(tmp, &data32->in_size.height);
	err |= put_user(tmp, &data->in_size.height);
	compat_isp_trace("in_size.height [%d]\n", tmp);

	err  = get_user(tmp, &data32->out_size.width);
	err |= put_user(tmp, &data->out_size.width);
	compat_isp_trace("out_size.width [%d]\n", tmp);

	err  = get_user(tmp, &data32->out_size.height);
	err |= put_user(tmp, &data->out_size.height);
	compat_isp_trace("out_size.height [%d]\n", tmp);

	err  = get_user(val, &data32->img_vir.chn0);
	err |= put_user(val, &data->img_vir.chn0);
	compat_isp_trace("img_vir.chn0 [%d]\n", tmp);

	err  = get_user(val, &data32->img_vir.chn1);
	err |= put_user(val, &data->img_vir.chn1);
	compat_isp_trace("img_vir.chn1 [%d]\n", tmp);

	err  = get_user(val, &data32->img_vir.chn2);
	err |= put_user(val, &data->img_vir.chn2);
	compat_isp_trace("img_vir.chn2 [%d]\n", tmp);

	err  = get_user(val, &data32->img_offset.chn0);
	err |= put_user(val, &data->img_offset.chn0);
	compat_isp_trace("img_offset.chn0 [%d]\n", tmp);

	err  = get_user(val, &data32->img_offset.chn1);
	err |= put_user(val, &data->img_offset.chn1);
	compat_isp_trace("img_offset.chn1 [%d]\n", tmp);

	err  = get_user(val, &data32->img_offset.chn2);
	err |= put_user(val, &data->img_offset.chn2);
	compat_isp_trace("img_offset.chn2 [%d]\n", tmp);

	err  = get_user(tmp, &data32->img_fd);
	err |= put_user(tmp, &data->img_fd);
	compat_isp_trace("img_fd [%d]\n", tmp);

	return err;
}

static int compat_put_raw_proc_info(
		struct compat_isp_raw_proc_info __user *data32,
		struct isp_raw_proc_info __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	compat_ulong_t val;

	err  = get_user(tmp, &data->in_size.width);
	err |= put_user(tmp, &data32->in_size.width);
	err |= get_user(tmp, &data->in_size.height);
	err |= put_user(tmp, &data32->in_size.height);

	err |= get_user(tmp, &data->out_size.width);
	err |= put_user(tmp, &data32->out_size.width);
	err |= get_user(tmp, &data->out_size.height);
	err |= put_user(tmp, &data32->out_size.height);

	err |= get_user(val, &data->img_vir.chn0);
	err |= put_user(val, &data32->img_vir.chn0);
	err |= get_user(val, &data->img_vir.chn1);
	err |= put_user(val, &data32->img_vir.chn1);
	err |= get_user(val, &data->img_vir.chn2);
	err |= put_user(val, &data32->img_vir.chn2);

	err |= get_user(val, &data->img_offset.chn0);
	err |= put_user(val, &data32->img_offset.chn0);
	err |= get_user(val, &data->img_offset.chn1);
	err |= put_user(val, &data32->img_offset.chn1);
	err |= get_user(val, &data->img_offset.chn2);
	err |= put_user(val, &data32->img_offset.chn2);

	err |= get_user(tmp, &data->img_fd);
	err |= put_user(tmp, &data32->img_fd);

	return err;
}

static int compat_get_isp_io_param(
		struct compat_isp_io_param __user *data32,
		struct isp_io_param __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	unsigned long parm;

	err  = get_user(tmp, &data32->isp_id);
	err |= put_user(tmp, &data->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data32->scene_id);
	err |= put_user(tmp, &data->scene_id);
	compat_isp_trace("scene_id [%d]\n", tmp);

	err |= get_user(tmp, &data32->sub_block);
	err |= put_user(tmp, &data->sub_block);
	compat_isp_trace("sub_block [%d]\n", tmp);

	err |= get_user(tmp, &data32->property);
	err |= put_user(tmp, &data->property);
	compat_isp_trace("property [%d]\n", tmp);

	err |= get_user(parm, &data32->property_param);
	err |= put_user(((void *)parm), &data->property_param);
	compat_isp_trace("property_param [0x%lx]\n", parm);

	return err;
}

static int compat_put_isp_io_param(
		struct compat_isp_io_param __user *data32,
		struct isp_io_param __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	compat_caddr_t parm;

	err  = get_user(tmp, &data->isp_id);
	err |= put_user(tmp, &data32->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data->sub_block);
	err |= put_user(tmp, &data32->sub_block);
	compat_isp_trace("sub_block [%d]\n", tmp);

	err |= get_user(tmp, &data->property);
	err |= put_user(tmp, &data32->property);
	compat_isp_trace("property [%d]\n", tmp);

	err |= get_user(parm, (compat_caddr_t *)&data->property_param);
	err |= put_user(parm, &data32->property_param);

	compat_isp_trace("property_param [0x%x]\n", parm);

	return err;
}

static int compat_get_isp_capability(
		struct compat_sprd_isp_capability __user *data32,
		struct sprd_isp_capability __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	unsigned long parm;

	err  = get_user(tmp, &data32->isp_id);
	err |= put_user(tmp, &data->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data32->index);
	err |= put_user(tmp, &data->index);
	compat_isp_trace("index [%d]\n", tmp);

	err |= get_user(parm, &data32->property_param);
	err |= put_user(((void *)parm), &data->property_param);
	compat_isp_trace("property_param [0x%lx]\n", parm);

	return err;
}

static int compat_put_isp_capability(
		struct compat_sprd_isp_capability __user *data32,
		struct sprd_isp_capability __user *data)
{
	int err = 0;
	uint32_t tmp = 0;
	compat_caddr_t parm;

	err  = get_user(tmp, &data->isp_id);
	err |= put_user(tmp, &data32->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data->index);
	err |= put_user(tmp, &data32->index);
	compat_isp_trace("index [%d]\n", tmp);

	err |= get_user(parm, (compat_caddr_t *)&data->property_param);
	err |= put_user(parm, &data32->property_param);
	compat_isp_trace("property_param [0x%x]\n", parm);

	return err;
}

long compat_sprd_img_k_ioctl(struct file *file, unsigned int cmd,
		unsigned long param)
{
	long ret = 0;

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	pr_debug("cmd: 0x%x, %s 0x%x\n",
			cmd,
			cam_ioctl_str[_IOC_NR(cmd)],
			cam_ioctl_val[_IOC_NR(cmd)]);

	switch (cmd) {
	case COMPAT_SPRD_ISP_IO_SET_STATIS_BUF:
	{
		struct compat_isp_statis_buf_input __user *data32;
		struct isp_statis_buf_input __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(
			sizeof(struct isp_statis_buf_input));

		compat_get_set_statis_buf(data32, data);
		file->f_op->unlocked_ioctl(file,
			SPRD_ISP_IO_SET_STATIS_BUF, (unsigned long)data);
		break;
	}
	case COMPAT_SPRD_ISP_IO_CAPABILITY:
	{
		struct compat_sprd_isp_capability __user *data32;
		struct sprd_isp_capability __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(
			sizeof(struct sprd_isp_capability));

		compat_get_isp_capability(data32, data);
		file->f_op->unlocked_ioctl(file,
			SPRD_ISP_IO_CAPABILITY, (unsigned long)data);
		compat_put_isp_capability(data32, data);

		break;
	}
	case COMPAT_SPRD_ISP_IO_CFG_PARAM:
	{
		struct compat_isp_io_param __user *data32;
		struct isp_io_param __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(struct isp_io_param));
		compat_get_isp_io_param(data32, data);

		file->f_op->unlocked_ioctl(file,
				SPRD_ISP_IO_CFG_PARAM,
				(unsigned long)data);
		compat_put_isp_io_param(data32, data);
		break;
	}
	case COMPAT_SPRD_ISP_IO_RAW_CAP:
	{
		struct compat_isp_raw_proc_info __user *data32;
		struct isp_raw_proc_info __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(
			sizeof(struct isp_raw_proc_info));

		compat_get_raw_proc_info(data32, data);
		file->f_op->unlocked_ioctl(file,
				SPRD_ISP_IO_RAW_CAP,
				(unsigned long)data);

		compat_put_raw_proc_info(data32, data);

		break;
	}
	default:
		file->f_op->unlocked_ioctl(file, cmd, param);
		break;
	}

	return ret;
}
