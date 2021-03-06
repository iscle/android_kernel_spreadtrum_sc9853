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

#include <linux/err.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of.h>

#include "cam_common.h"
#include "cam_buf.h"

/*#define CAM_BUF_DEBUG*/

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "cam_buf: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define ION
#ifdef ION
#include "ion.h"
#include "ion_priv.h"
#endif

#ifdef CAM_BUF_DEBUG
#define CAM_BUF_TRACE                      pr_info
#else
#define CAM_BUF_TRACE                      pr_debug
#endif


struct fd_map_dma {
	struct list_head list;
	uint32_t type;
	union {
		struct {
			int fd;
			void *dma_buf;
		} user_type;
		struct {
			struct ion_client *client;
			struct ion_handle *handle;
		} kernel_type;
	};
};
static LIST_HEAD(dma_buffer_list);
static DEFINE_MUTEX(dma_buffer_lock);
static atomic_t s_cambuf_count = ATOMIC_INIT(0);
static atomic_t s_cambuf_size = ATOMIC_INIT(0);

#ifdef CAM_BUF_DEBUG
static unsigned long map_iova[0x100] = {0};
static DEFINE_MUTEX(map_iova_mutex);

static int find_used_iova(unsigned long iova)
{
	int i;

	for (i = 0; i < 0x100; i++) {
		if (map_iova[i] == iova)
			return i;
	}
	return -1;
}

static int find_unused_iova(unsigned long iova)
{
	int i;

	for (i = 0; i < 0x100; i++) {
		if (map_iova[i] == 0)
			return i;
	}
	return -1;
}
#endif

static void log_iova(unsigned long iova)
{
#ifdef CAM_BUF_DEBUG
	int i;

	mutex_lock(&map_iova_mutex);
	i = find_unused_iova(iova);
	if (i >= 0)
		map_iova[i] = iova;
	mutex_unlock(&map_iova_mutex);
#endif
}

static void unlog_iova(unsigned long iova)
{
#ifdef CAM_BUF_DEBUG
	int i;

	mutex_lock(&map_iova_mutex);
	i = find_used_iova(iova);
	if (i >= 0)
		map_iova[i] = 0;
	mutex_unlock(&map_iova_mutex);
#endif
}

static void print_iova(void)
{
#ifdef CAM_BUF_DEBUG
	int i;

	mutex_lock(&map_iova_mutex);
	for (i = 0; i < 0x100; i++) {
		if (map_iova[i] != 0)
			pr_info("ioval 0x%x\n", (uint32_t) map_iova[i]);
	}
	mutex_unlock(&map_iova_mutex);
#endif
}

static int dma_buffer_list_add_k(struct ion_client *client,
			struct ion_handle *handle, uint32_t type)
{
	struct fd_map_dma *fd_dma = NULL;
	struct list_head *g_dma_buffer_list = &dma_buffer_list;

	list_for_each_entry(fd_dma, g_dma_buffer_list, list) {
		if (fd_dma->type == type
			&& client == fd_dma->kernel_type.client
			&& handle == fd_dma->kernel_type.handle)
			return 0;
	}

	fd_dma = kzalloc(sizeof(struct fd_map_dma), GFP_KERNEL);
	if (!fd_dma)
		return -ENOMEM;

	fd_dma->type = type;
	fd_dma->kernel_type.client = client;
	fd_dma->kernel_type.handle = handle;
	mutex_lock(&dma_buffer_lock);
	list_add_tail(&fd_dma->list, g_dma_buffer_list);
	mutex_unlock(&dma_buffer_lock);
	pr_debug("add 0x%p 0x%p\n", fd_dma->kernel_type.client,
		fd_dma->kernel_type.handle);

	return 0;
}

static int dma_buffer_list_add(int fd, void *buf)
{
	struct fd_map_dma *fd_dma = NULL;
	struct list_head *g_dma_buffer_list = &dma_buffer_list;

	list_for_each_entry(fd_dma, g_dma_buffer_list, list) {
		if (fd_dma->type == CAM_BUF_USER_TYPE
			&& fd == fd_dma->user_type.fd
			&& buf == fd_dma->user_type.dma_buf)
			return 0;
	}

	fd_dma = kzalloc(sizeof(struct fd_map_dma), GFP_KERNEL);
	if (!fd_dma)
		return -ENOMEM;

	fd_dma->type = CAM_BUF_USER_TYPE;
	fd_dma->user_type.fd = fd;
	fd_dma->user_type.dma_buf = buf;
	mutex_lock(&dma_buffer_lock);
	list_add_tail(&fd_dma->list, g_dma_buffer_list);
	mutex_unlock(&dma_buffer_lock);
	dma_buf_get(fd_dma->user_type.fd);
	pr_debug("add 0x%x 0x%p\n", fd_dma->user_type.fd,
		((struct dma_buf *)fd_dma->user_type.dma_buf));

	return 0;
}

int cam_buf_alloc_k(struct cam_buf_info *buf_info, struct device *dev,
	size_t size, uint32_t num, uint32_t type)
{
	int rtn = 0;
	char name[32];

	struct ion_client *client;/*for ion alloc buffer*/
	struct ion_handle *handle;
	struct ion_buffer *ionbuffer = NULL;
	static int j;
	int i = 0;

	if (!buf_info || !dev || num > 3 || num == 0
		|| (type == CAM_BUF_SWAP_TYPE
		&& num > 1)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	memset(buf_info, 0x00, sizeof(*buf_info));
#ifdef ION
	CAM_BUF_TRACE("cb: %pS\n", __builtin_return_address(0));
	for (i = 0; i < num; i++) {
		sprintf(name, "sprd-cam-offline-%d", j++);
		client = sprd_ion_client_create(name);
		if (IS_ERR_OR_NULL(client)) {
			pr_err("failed to create offline ION client\n");
			return -EPERM;
		}

		if (sprd_iommu_attach_device(dev) == 0) {
			/* iommu enabled */
			handle = ion_alloc(client, size, 0,
				ION_HEAP_ID_MASK_SYSTEM, 0);
			if (IS_ERR_OR_NULL(handle)) {
				pr_err("failed to alloc offline tmp buf size = 0x%x\n",
					(int)size);
				return -EPERM;
			}
		} else {
			handle = ion_alloc(client, size, 0,
				ION_HEAP_ID_MASK_MM, 0);
			if (IS_ERR_OR_NULL(handle)) {
				pr_err("failed to alloc offline tmp buf size = 0x%x\n",
					(int)size);
				return -EPERM;
			}
		}
		buf_info->size[i] = size;
		buf_info->client[i] = client;
		buf_info->handle[i] = handle;
		ionbuffer = ion_handle_buffer(handle);
		if (IS_ERR_OR_NULL(ionbuffer)) {
			pr_err("fail to get ionbuffer ptr!\n");
			return -EPERM;
		}
		atomic_add(ionbuffer->size, &s_cambuf_size);
		CAM_BUF_TRACE("alloc buffer %p ok, total %d!\n", handle,
			atomic_read(&s_cambuf_size));
	}
	buf_info->dev = dev;
	buf_info->type = type;
	buf_info->num = num;
	CAM_BUF_TRACE("num %d\n", buf_info->num);
#else
	pr_info("Not support ion buf operation\n");
	rtn = -EFAULT;
#endif
	return rtn;
}

int cam_buf_free_k(struct cam_buf_info *buf_info)
{
	int rtn = 0;
	struct ion_client *client;/*for ion alloc buffer*/
	struct ion_handle *handle;
	struct ion_buffer *ionbuffer = NULL;
	int i = 0;

	if (!buf_info || buf_info->num > 3 || buf_info->num == 0
		|| (buf_info->type == CAM_BUF_SWAP_TYPE
		&& buf_info->num > 1)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

#ifdef ION
	for (i = 0; i < buf_info->num; i++) {
		client = buf_info->client[i];
		handle = buf_info->handle[i];
		if (client == NULL || handle == NULL) {
			pr_err("fail to get valid input ptr\n");
			return -EPERM;
		}

		ionbuffer = ion_handle_buffer(handle);
		ion_free(client, handle);
		ion_client_destroy(client);
		buf_info->client[i] = NULL;
		buf_info->handle[i] = NULL;
		if (IS_ERR_OR_NULL(ionbuffer))
			pr_err("fail to get ionbuffer ptr!\n");
		else
			atomic_sub(ionbuffer->size, &s_cambuf_size);
		CAM_BUF_TRACE("free buffer %p ok, total %d!\n", handle,
			atomic_read(&s_cambuf_size));
	}
#else
	pr_info("Not support ion buf operation\n");
#endif
	return rtn;
}

static int cam_buf_map_k(struct cam_buf_info *buf_info)
{
	int rtn = 0;
	struct ion_client *client;/*for ion alloc buffer*/
	struct ion_handle *handle;
	struct ion_buffer *ionbuffer = NULL;
	struct sprd_iommu_map_data iommu_data;
	uint32_t iova_i = 0;
	int i = 0;

	if (!buf_info || buf_info->num > 3
		|| (buf_info->type == CAM_BUF_SWAP_TYPE
		&& buf_info->num > 1)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	CAM_BUF_TRACE("num %d\n", buf_info->num);
	for (i = 0; i < buf_info->num; i++) {

		client = buf_info->client[i];
		handle = buf_info->handle[i];
		if (client == NULL || handle == NULL) {
			pr_err("fail to get valid input ptr\n");
			return -EPERM;
		}

		iova_i = i;
		if (sprd_iommu_attach_device(buf_info->dev) == 0) {
			memset(&iommu_data, 0x00, sizeof(iommu_data));
			ionbuffer = ion_handle_buffer(handle);
			if (IS_ERR_OR_NULL(ionbuffer)) {
				pr_err("fail to get ionbuffer ptr\n");
				return -EPERM;
			}
			iommu_data.buf = (void *)ionbuffer;
			iommu_data.iova_size = ionbuffer->size;
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.sg_offset = 0;
			rtn = sprd_iommu_map(buf_info->dev, &iommu_data);
			if (rtn) {
				pr_err("failed to get iommu kaddr\n");
				return -EFAULT;
			}
			if (buf_info->type == CAM_BUF_KERNEL_TYPE) {
				buf_info->kaddr[i] = (uint32_t *)
					ion_map_kernel(client, handle);
				if (CAM_ADDR_INVALID(buf_info->kaddr[i])) {
					pr_err("fail to map kernel vir_addr\n");
					return -EPERM;
				}
				buf_info->state |= CAM_BUF_STATE_MAPPING;
			} else if (buf_info->type == CAM_BUF_SWAP_TYPE) {
				if (strstr(buf_info->dev->of_node->name,
					"dcam")) {
					iova_i = 0;
					buf_info->state |=
						CAM_BUF_STATE_MAPPING_DCAM;
				} else if (strstr(buf_info->dev->of_node->name,
					"isp")) {
					iova_i = 1;
					buf_info->state |=
						CAM_BUF_STATE_MAPPING_ISP;
				}
			}
			buf_info->iova[iova_i] = iommu_data.iova_addr
				+ buf_info->offset[i];
		} else {
			unsigned long phys_addr = 0;

			if (ion_phys(client, handle,
					&phys_addr, &buf_info->size[i])) {
				pr_err("failed to phys offline tmp buf\n");
				return -EPERM;
			}
			if (buf_info->type == CAM_BUF_KERNEL_TYPE) {
				buf_info->state |= CAM_BUF_STATE_MAPPING;
				buf_info->kaddr[i] = phys_to_virt(
					(unsigned long)phys_addr);
			} else if (buf_info->type == CAM_BUF_SWAP_TYPE) {
				if (strstr(buf_info->dev->of_node->name,
					"dcam")) {
					iova_i = 0;
					buf_info->state |=
						CAM_BUF_STATE_MAPPING_DCAM;
				} else if (strstr(buf_info->dev->of_node->name,
					"isp"))  {
					iova_i = 1;
					buf_info->state |=
						CAM_BUF_STATE_MAPPING_ISP;
				}
			}
			buf_info->iova[iova_i] = phys_addr
				+ buf_info->offset[i];
		}
		dma_buffer_list_add_k(buf_info->client[i],
			buf_info->handle[i], buf_info->type);
		atomic_inc(&s_cambuf_count);
		if (iova_i)
			CAM_BUF_TRACE("map iova 0x%x => 0x%x total cnt %d %s\n",
				(uint32_t)buf_info->iova[0],
				(uint32_t)buf_info->iova[1],
				atomic_read(&s_cambuf_count),
				buf_info->dev->of_node->name);
		else
			CAM_BUF_TRACE("map iova 0x%x total cnt %d %s\n",
				(uint32_t)buf_info->iova[iova_i],
				atomic_read(&s_cambuf_count),
				buf_info->dev->of_node->name);
		log_iova(buf_info->iova[iova_i]);
	}
	return rtn;
}

static int cam_buf_unmap_k(struct cam_buf_info *buf_info)
{
	int rtn = 0;
	struct ion_client *client;/*for ion alloc buffer*/
	struct ion_handle *handle;
	struct ion_buffer *ionbuffer = NULL;
	struct sprd_iommu_unmap_data iommu_data;
	const char *name = NULL;
	uint32_t iova_i = 0;
	int i = 0;

	if (!buf_info || buf_info->num > 3 || buf_info->num == 0
		|| (buf_info->type == CAM_BUF_SWAP_TYPE
		&& buf_info->num > 1)) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	CAM_BUF_TRACE("num %d\n", buf_info->num);
	for (i = 0; i < buf_info->num; i++) {

		client = buf_info->client[i];
		handle = buf_info->handle[i];
		name = buf_info->dev->of_node->name;
		if (client == NULL || handle == NULL) {
			pr_err("fail to get valid input ptr\n");
			return -EPERM;
		}

		iova_i = i;
		memset(&iommu_data, 0x00, sizeof(iommu_data));
		if (sprd_iommu_attach_device(buf_info->dev) == 0) {

			if (buf_info->type == CAM_BUF_KERNEL_TYPE) {
				buf_info->kaddr[i] = NULL;
				buf_info->state &= ~CAM_BUF_STATE_MAPPING;
			} else if (buf_info->type == CAM_BUF_SWAP_TYPE) {
				if (strstr(name, "dcam")) {
					iova_i = 0;
					buf_info->state &=
						~CAM_BUF_STATE_MAPPING_DCAM;
				} else if (strstr(name, "isp")) {
					iova_i = 1;
					buf_info->state &=
						~CAM_BUF_STATE_MAPPING_ISP;
				}
			}
			ionbuffer = ion_handle_buffer(handle);
			if (IS_ERR_OR_NULL(ionbuffer)) {
				pr_err("fail to get ionbuffer ptr\n");
				return -EPERM;
			}
			iommu_data.iova_addr = buf_info->iova[iova_i];
			iommu_data.iova_size = ionbuffer->size;
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.buf = NULL;
			if (buf_info->iova[iova_i] > 0) {
				rtn = sprd_iommu_unmap(buf_info->dev,
					&iommu_data);
				if (rtn) {
					pr_err("failed to unmap iommu kaddr\n");
					return -EFAULT;
				}
				buf_info->iova[iova_i] = 0;
			}
		} else {
			if (buf_info->type == CAM_BUF_KERNEL_TYPE) {
				buf_info->kaddr[i] = NULL;
				buf_info->state &= ~CAM_BUF_STATE_MAPPING;
			} else if (buf_info->type == CAM_BUF_SWAP_TYPE) {
				if (strstr(name, "dcam")) {
					iova_i = 0;
					buf_info->state &=
						~CAM_BUF_STATE_MAPPING_DCAM;
				} else if (strstr(name, "isp")) {
					iova_i = 1;
					buf_info->state &=
						~CAM_BUF_STATE_MAPPING_ISP;
				}
			}
			if (buf_info->iova[iova_i] > 0) {
				iommu_data.iova_addr = buf_info->iova[iova_i];
				buf_info->iova[iova_i] = 0;
			}
		}
		atomic_dec(&s_cambuf_count);
		CAM_BUF_TRACE("unmap iova 0x%x	total cnt %d %s\n",
			(uint32_t)iommu_data.iova_addr,
			atomic_read(&s_cambuf_count),
			buf_info->dev->of_node->name);
		unlog_iova(iommu_data.iova_addr);
	}
	return rtn;
}

int cam_buf_get_sg_table(struct cam_buf_info *buf_info)
{
	int i = 0, count = 2, ret = 0;

	if (!buf_info) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (buf_info->type != CAM_BUF_USER_TYPE)
		return 0;

	if (buf_info->offset[0] == buf_info->offset[1])
		count = 1;
	for (i = 0; i < count; i++) {
		if (buf_info->mfd[i] > 0) {
			ret = sprd_ion_get_buffer(buf_info->mfd[i],
					NULL,
					&buf_info->buf[i],
					&buf_info->size[i]);
			if (ret) {
				pr_err("failed to get sg table %d mfd 0x%x\n",
					i, buf_info->mfd[i]);
				return -EFAULT;
			}

			buf_info->dmabuf_p[i] = dma_buf_get(buf_info->mfd[i]);
			if (IS_ERR_OR_NULL(buf_info->dmabuf_p[i])) {
				pr_err("failed to get dma buf %p\n",
					buf_info->dmabuf_p[i]);
				return -EFAULT;
			}
			dma_buf_put(buf_info->dmabuf_p[i]);
			dma_buffer_list_add(buf_info->mfd[i],
					buf_info->dmabuf_p[i]);
			pr_debug("ok!\n");
		}
	}

	return 0;
}

int  cam_buf_put_sg_table(void)
{
	struct fd_map_dma *fd_dma = NULL;
	struct fd_map_dma *fd_dma_next = NULL;
	struct list_head *buffer_list = &dma_buffer_list;
	struct cam_buf_info info;

	list_for_each_entry_safe(fd_dma, fd_dma_next, buffer_list, list) {
		mutex_lock(&dma_buffer_lock);
		list_del(&fd_dma->list);
		mutex_unlock(&dma_buffer_lock);
		if (fd_dma->type == CAM_BUF_USER_TYPE) {
			dma_buf_put(fd_dma->user_type.dma_buf);
			pr_debug("del: 0x%x 0x%p\n",
				fd_dma->user_type.fd,
			((struct dma_buf *)fd_dma->user_type.dma_buf));
		} else {
			info.client[0] = fd_dma->kernel_type.client;
			info.handle[0] = fd_dma->kernel_type.handle;
			info.type = fd_dma->type;
			info.num = 1;
			cam_buf_free_k(&info);
			pr_debug("del: %p 0x%p\n",
				fd_dma->kernel_type.client,
				fd_dma->kernel_type.handle);
		}
		kfree(fd_dma);
	}
	pr_info("total size: %d\n", atomic_read(&s_cambuf_size));
	pr_info("map cnt: %d\n", atomic_read(&s_cambuf_count));
	print_iova();
	return 0;
}

int cam_buf_map_addr(struct cam_buf_info *buf_info)
{
	int i = 0, count = 2, ret = 0;
	struct sprd_iommu_map_data iommu_data;

	if (!buf_info) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (buf_info->type != CAM_BUF_USER_TYPE) {
		CAM_BUF_TRACE("cb: %pS\n", __builtin_return_address(0));
		return cam_buf_map_k(buf_info);
	}

	CAM_BUF_TRACE("cb: %pS\n", __builtin_return_address(0));
	if (buf_info->offset[0] == buf_info->offset[1])
		count = 1;
	for (i = 0; i < count; i++) {
		if (buf_info->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(buf_info->dev) == 0) {
			memset(&iommu_data, 0x00, sizeof(iommu_data));
			iommu_data.buf = buf_info->buf[i];
			iommu_data.iova_size = buf_info->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.sg_offset = buf_info->offset[i];

			ret = sprd_iommu_map(buf_info->dev, &iommu_data);
			if (ret) {
				pr_err("failed to get iommu kaddr %d\n", i);
				return -EFAULT;
			}

			buf_info->iova[i] = iommu_data.iova_addr;
		} else {
			ret = sprd_ion_get_phys_addr(-1, buf_info->dmabuf_p[i],
					&buf_info->iova[i],
					&buf_info->size[i]);
			buf_info->iova[i] += buf_info->offset[i];
		}
		atomic_inc(&s_cambuf_count);
		CAM_BUF_TRACE("map iova 0x%x total cnt %d\n",
			(uint32_t)buf_info->iova[i],
			atomic_read(&s_cambuf_count));
		log_iova(buf_info->iova[i]);
	}
	buf_info->state |= CAM_BUF_STATE_MAPPING;

	return ret;
}

int cam_buf_check_addr(struct cam_buf_info *buf_info)
{
	struct fd_map_dma *fd_dma = NULL;
	struct list_head *g_dma_buffer_list = &dma_buffer_list;

	list_for_each_entry(fd_dma, g_dma_buffer_list, list) {
		if (fd_dma->type == CAM_BUF_USER_TYPE
			&& buf_info->mfd[0] == fd_dma->user_type.fd
			&& buf_info->dmabuf_p[0] == fd_dma->user_type.dma_buf)
			break;
		else if (fd_dma->type != CAM_BUF_USER_TYPE
			&& buf_info->client[0] == fd_dma->kernel_type.client
			&& buf_info->handle[0] == fd_dma->kernel_type.handle)
			return  0;
	}

	if (&fd_dma->list == g_dma_buffer_list) {
		pr_err("invalid mfd: 0x%x, dma_buf:0x%p!\n",
			buf_info->mfd[0],
			buf_info->dmabuf_p[0]);
		return -1;
	}
	return sprd_ion_check_phys_addr(buf_info->dmabuf_p[0]);
}

int cam_buf_unmap_addr(struct cam_buf_info *buf_info)
{
	int i = 0, count = 2, ret = 0;
	struct sprd_iommu_unmap_data iommu_data;

	if (buf_info->type != CAM_BUF_USER_TYPE) {
		CAM_BUF_TRACE("cb: %pS\n", __builtin_return_address(0));
		return cam_buf_unmap_k(buf_info);
	}

	CAM_BUF_TRACE("cb: %pS\n", __builtin_return_address(0));
	if (buf_info->offset[0] == buf_info->offset[1])
		count = 1;
	for (i = 0; i < count; i++) {
		if (buf_info->size[i] <= 0 || buf_info->iova[i] == 0)
			continue;

		if (sprd_iommu_attach_device(buf_info->dev) == 0) {
			iommu_data.iova_addr = buf_info->iova[i];
			iommu_data.iova_size = buf_info->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.buf = NULL;

			ret = sprd_iommu_unmap(buf_info->dev,
							&iommu_data);
			if (!ret) {
				buf_info->iova[i] = 0;
				buf_info->size[i] = 0;
			} else {
				pr_err("failed to free iommu 0x%x\n",
					(uint32_t)iommu_data.iova_addr);
				return -EFAULT;
			}

			atomic_dec(&s_cambuf_count);
			CAM_BUF_TRACE("unmap iova 0x%x  total cnt %d\n",
				(uint32_t)iommu_data.iova_addr,
				atomic_read(&s_cambuf_count));
			unlog_iova(iommu_data.iova_addr);
		}
	}
	buf_info->state &= ~CAM_BUF_STATE_MAPPING;

	return 0;
}

int cam_buf_unmap_addr_with_id(struct cam_buf_info *buf_info,
	enum sprd_iommu_chtype ctype, uint32_t cid)
{
	int i, ret = 0;
	struct sprd_iommu_unmap_data iommu_data;

	pr_debug("cb: %pS, iova 0x%lx\n",
		 __builtin_return_address(0), buf_info->iova[0]);

	if (buf_info->type != CAM_BUF_USER_TYPE)
		return cam_buf_unmap_k(buf_info);

	for (i = 0; i < 2; i++) {
		if (buf_info->size[i] <= 0 || buf_info->iova[i] == 0)
			continue;

		if (sprd_iommu_attach_device(buf_info->dev) == 0) {
			iommu_data.iova_addr = buf_info->iova[i];
			iommu_data.iova_size = buf_info->size[i];
			iommu_data.ch_type = ctype;
			iommu_data.buf = NULL;
			iommu_data.channel_id = cid;

			ret = sprd_iommu_unmap(buf_info->dev,
					&iommu_data);
			if (!ret) {
				buf_info->iova[i] = 0;
				buf_info->size[i] = 0;
			} else {
				pr_err("failed to free iommu %d\n", i);
				return -EFAULT;
			}
		}
	}

	return 0;
}
