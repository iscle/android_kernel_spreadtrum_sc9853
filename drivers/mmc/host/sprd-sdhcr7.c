/*
 * linux/drivers/mmc/host/sprd-sdhcr7.c - Secure Digital Host Controller
 * Interface driver
 *
 * Copyright (C) 2015-2016 Spreadtrum corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include "sprd-sdhcr7.h"

#define DRIVER_NAME "sprd_sdhcr7"

#ifdef CONFIG_PM
static int sprd_sdhc_runtime_pm_get(struct sprd_sdhc_host *host)
{
	return pm_runtime_get_sync(host->mmc->parent);
}

static int sprd_sdhc_runtime_pm_put(struct sprd_sdhc_host *host)
{
	pm_runtime_mark_last_busy(host->mmc->parent);
	return pm_runtime_put_autosuspend(host->mmc->parent);
}
#else
static inline int sprd_sdhc_runtime_pm_get(struct sprd_sdhc_host *host)
{
	return 0;
}

static inline int sprd_sdhc_runtime_pm_put(struct sprd_sdhc_host *host)
{
	return 0;
}
#endif

static void dump_adma_info(struct sprd_sdhc_host *host)
{
	void *desc_header = host->adma_desc;
	u64 desc_ptr;
	unsigned long start = jiffies;

	desc_ptr = (u64)sprd_sdhc_readl(host,
				SPRD_SDHC_REG_32_ADMA2_ADDR_H) << 32;
	desc_ptr |= sprd_sdhc_readl(host, SPRD_SDHC_REG_32_ADMA2_ADDR_L);
	pr_info(" %s ADMA Err: 0x%08x EMMC_DEBUG0:0x%08x EMMC_DEBUG1:0x%08x \
		EMMC_DEBUG2:0x%08x desc_ptr: 0x%llx\n",
		host->device_name,
		sprd_sdhc_readl(host, SPRD_SDHC_REG_ADMA_ERROR),
		sprd_sdhc_readl(host, SPRD_SDHC_REG_EMMC_DEBUG0),
		sprd_sdhc_readl(host, SPRD_SDHC_REG_EMMC_DEBUG1),
		sprd_sdhc_readl(host, SPRD_SDHC_REG_EMMC_DEBUG2),
		desc_ptr);

	while (desc_ptr) {
		struct sprd_adma2_128_desc *dma_desc = desc_header;

		pr_info("%s: all desc information 0x%p: \
			DMA 0x%08x%08x, LEN 0x%x,  Attr=0x%x\n",
			host->device_name, desc_header,
			le32_to_cpu(dma_desc->addr_hi),
			le32_to_cpu(dma_desc->addr_lo),
			le32_to_cpu(dma_desc->len_cmd) >> 16,
			le32_to_cpu(dma_desc->len_cmd) & 0xFFFF);

		desc_header += SPRD_ADMA2_64_DESC_SZ;
		if (dma_desc->len_cmd & cpu_to_le16(ADMA2_END))
			break;

		if (time_after(jiffies, start + HZ)) {
			pr_err("%s %s ADMA error have no end desc\n",
				__func__, host->device_name);
			break;
		}
	}

	desc_ptr = (u64)sprd_sdhc_readl(host,
				SPRD_SDHC_REG_ADMA_PROCESS_H) << 32;
	desc_ptr |= sprd_sdhc_readl(host, SPRD_SDHC_REG_ADMA_PROCESS_L);
	pr_info(" %s ADMA current ADMA  desc_ptr: 0x%llx\n",
		host->device_name, desc_ptr);
}

static void dump_sdio_reg(struct sprd_sdhc_host *host)
{
	if (!host->mmc->card)
		return;

	print_hex_dump(KERN_INFO, "sdhc register: ", DUMP_PREFIX_OFFSET,
			16, 4, host->ioaddr, 64, 0);

	if ((host->flags & SPRD_USE_ADMA) &&
			(strcmp(host->device_name, "sdio_wifi") != 0))
		dump_adma_info(host);

	pr_info(" ===========================================\n");
}

static int sprd_get_delay_value(struct platform_device *pdev)
{
	int ret = 0;
	u32 dly_vl[4];
	struct device_node *np = pdev->dev.of_node;
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	struct timing_delay_value *timing_dly;

	timing_dly = devm_kzalloc(&pdev->dev, sizeof(*timing_dly), GFP_KERNEL);
	if (!timing_dly)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "sprd,mmchs-dly", dly_vl, 4);
	if (!ret)
		timing_dly->mmchs_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	ret = of_property_read_u32_array(np, "sprd,sdr50-dly", dly_vl, 4);
	if (!ret)
		timing_dly->sdr50_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	ret = of_property_read_u32_array(np, "sprd,sdr104-dly", dly_vl, 4);
	if (!ret)
		timing_dly->sdr104_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	ret = of_property_read_u32_array(np, "sprd,ddr52-dly", dly_vl, 4);
	if (!ret)
		timing_dly->ddr52_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	ret = of_property_read_u32_array(np, "sprd,hs200-dly", dly_vl, 4);
	if (!ret)
		timing_dly->hs200_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	ret = of_property_read_u32_array(np, "sprd,hs400-dly", dly_vl, 4);
	if (!ret)
		timing_dly->hs400_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	ret = of_property_read_u32_array(np, "sprd,hs400es-dly", dly_vl, 4);
	if (!ret)
		timing_dly->hs400es_dly = SPRD_SDHC_DLY_TIMING(dly_vl[0],
					dly_vl[1], dly_vl[2], dly_vl[3]);

	host->timing_dly = timing_dly;

	return 0;
}

static void sprd_set_delay_value(struct sprd_sdhc_host *host)
{
	switch (host->ios.timing) {
	case MMC_TIMING_MMC_HS:
		if (host->ios.clock == 52000000) {
			host->dll_dly = host->timing_dly->mmchs_dly;
			sprd_sdhc_writel(host, host->dll_dly,
					SPRD_SDHC_REG_32_DLL_DLY);
			pr_info("(%s) mmchs default timing delay value 0x%08x\n",
				host->device_name, host->dll_dly);
		}
		break;
	case MMC_TIMING_UHS_SDR50:
		if (host->ios.clock == 100000000) {
			host->dll_dly = host->timing_dly->sdr50_dly;
			sprd_sdhc_writel(host, host->dll_dly,
					SPRD_SDHC_REG_32_DLL_DLY);
			pr_info("(%s) sdr50 default timing delay value 0x%08x\n",
				host->device_name, host->dll_dly);
		}
		break;
	case MMC_TIMING_UHS_SDR104:
		if (host->ios.clock == 208000000) {
			host->dll_dly = host->timing_dly->sdr104_dly;
			sprd_sdhc_writel(host, host->dll_dly,
					SPRD_SDHC_REG_32_DLL_DLY);
			pr_info("(%s) sdr104 default timing delay value 0x%08x\n",
				host->device_name, host->dll_dly);
		}
		break;
	case MMC_TIMING_MMC_DDR52:
		if (host->ios.clock == 52000000) {
			host->dll_dly = host->timing_dly->ddr52_dly;
			sprd_sdhc_writel(host, host->dll_dly,
					SPRD_SDHC_REG_32_DLL_DLY);
			pr_info("(%s) ddr52 default timing delay value: 0x%08x\n",
				host->device_name, host->dll_dly);
		}
		break;
	case MMC_TIMING_MMC_HS200:
		if (host->ios.clock == 200000000) {
			host->dll_dly = host->timing_dly->hs200_dly;
			sprd_sdhc_writel(host, host->dll_dly,
					SPRD_SDHC_REG_32_DLL_DLY);
			pr_info("(%s) hs200 default timing delay value: 0x%08x\n",
				host->device_name, host->dll_dly);
		}
		break;
	case MMC_TIMING_MMC_HS400:
		if (host->ios.clock == 200000000) {
			host->dll_dly = host->timing_dly->hs400_dly;
			sprd_sdhc_writel(host, host->dll_dly,
					SPRD_SDHC_REG_32_DLL_DLY);
			pr_info("(%s) hs400 final timing delay value: 0x%08x\n",
				host->device_name, host->dll_dly);
		}
		break;
	default:
		break;
	}
}

static void sprd_reset_ios(struct sprd_sdhc_host *host)
{
	sprd_sdhc_disable_all_int(host);

	host->ios.clock = 0;
	host->ios.vdd = 0;
	host->ios.power_mode = MMC_POWER_OFF;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	host->ios.signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_ALL);
	if (!strcmp("sdio_wifi", host->device_name))
		sprd_sdhc_writel(host, host->dll_dly, SPRD_SDHC_REG_32_DLL_DLY);
}

static void sprd_get_rsp(struct sprd_sdhc_host *host)
{
	u32 i, offset;
	unsigned int flags = host->cmd->flags;
	u32 *resp = host->cmd->resp;

	if (!(flags & MMC_RSP_PRESENT))
		return;

	if (flags & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0, offset = 12; i < 3; i++, offset -= 4) {
			resp[i] = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_RESP + offset) << 8;
			resp[i] |= sprd_sdhc_readb(host,
					SPRD_SDHC_REG_32_RESP + offset - 1);
		}
		resp[3] = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_RESP) << 8;
	} else
		resp[0] = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_RESP);
}

static int sdhci_pre_dma_transfer(struct sprd_sdhc_host *host,
				       struct mmc_data *data)
{
	int sg_count;

	if (data->host_cookie == COOKIE_MAPPED) {
		data->host_cookie = COOKIE_GIVEN;
		return data->sg_count;
	}

	WARN_ON(data->host_cookie == COOKIE_GIVEN);

	sg_count = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				data->flags & MMC_DATA_WRITE ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (sg_count == 0)
		return -ENOSPC;

	data->sg_count = sg_count;
	data->host_cookie = COOKIE_MAPPED;

	return sg_count;
}

static char *sprd_sdhc_kmap_atomic(struct scatterlist *sg, unsigned long *flags)
{
	local_irq_save(*flags);
	return kmap_atomic(sg_page(sg)) + sg->offset;
}

static void sprd_sdhc_kunmap_atomic(void *buffer, unsigned long *flags)
{
	kunmap_atomic(buffer);
	local_irq_restore(*flags);
}

static void sprd_adma_write_desc(void *desc,
				  dma_addr_t addr, int len, unsigned cmd)
{
	struct sprd_adma2_128_desc *dma_desc = desc;

	dma_desc->len_cmd = cpu_to_le16(cmd);
	dma_desc->len_cmd |= cpu_to_le32(len) << 16;
	dma_desc->addr_lo = cpu_to_le32((u32)addr);
	dma_desc->addr_hi = cpu_to_le32((u64)addr >> 32);
	dma_desc->reserve = cpu_to_le32(0);
}

static void sprd_adma_mark_end(void *desc)
{
	struct sprd_adma2_128_desc *dma_desc = desc;

	/* 32-bit and 64-bit descriptors have 'cmd' in same position */
	dma_desc->len_cmd |= cpu_to_le16(ADMA2_END);
}

static int sprd_sdhc_adma_table_pre(struct sprd_sdhc_host *host,
	struct mmc_data *data)
{
	int direction;

	u8 *desc;
	u8 *align;

	dma_addr_t addr;
	dma_addr_t align_addr;
	int len, offset, align_len;

	struct scatterlist *sg;
	int i;
	char *buffer;
	unsigned long flags;

	/*
	 * The spec does not specify endianness of descriptor table.
	 * We currently guess that it is LE.
	 */
	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	host->sg_count = sdhci_pre_dma_transfer(host, data);

	if (host->sg_count <= 0)
		return -EINVAL;

	desc = host->adma_desc;
	align = host->align_buffer;
	align_addr = host->align_addr;

	for_each_sg(data->sg, sg, host->sg_count, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		offset = (SPRD_ADMA2_64_ALIGN -
			(addr & (SPRD_ADMA2_64_ALIGN - 1))) &
			(SPRD_ADMA2_64_ALIGN - 1);
		align_len = min(len, offset);

		if (align_len) {
			if (data->flags & MMC_DATA_WRITE) {
				buffer = sprd_sdhc_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & (PAGE_SIZE - 1)) >
					(PAGE_SIZE - align_len));
				memcpy(align, buffer, align_len);
				sprd_sdhc_kunmap_atomic(buffer, &flags);
			}
			/* tran, valid */
			sprd_adma_write_desc(desc, align_addr,
					align_len, ADMA2_TRAN_VALID);

			WARN_ON(offset > 65536);

			align += SPRD_ADMA2_64_ALIGN;
			align_addr += SPRD_ADMA2_64_ALIGN;

			desc += SPRD_ADMA2_64_DESC_SZ;

			addr += align_len;
			len -= align_len;
		}

		WARN_ON(len > 65536);

		/* tran, valid */
		if (len > 0) {
			sprd_adma_write_desc(desc, addr, len, ADMA2_TRAN_VALID);
			desc += SPRD_ADMA2_64_DESC_SZ;
		}

		/*
		 * If this triggers then we have a calculation bug
		 * somewhere.
		 */
		WARN_ON((desc - host->adma_desc) >= ADMA_SIZE);
	}

		/*
		* Mark the last descriptor as the terminating descriptor
		*/

		if (desc != host->adma_desc) {
			desc -= SPRD_ADMA2_64_DESC_SZ;
			sprd_adma_mark_end(desc);
		}

	return 0;
}

static void sprd_sdhc_adma_table_post(struct sprd_sdhc_host *host,
				  struct mmc_data *data)
{
	int direction;

	struct scatterlist *sg;
	int i, size;
	u8 *align;
	char *buffer;
	unsigned long flags;
	bool has_unaligned;
	int align_len, len;

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	has_unaligned = false;
	for_each_sg(data->sg, sg, host->sg_count, i)
		if (sg_dma_address(sg) & (SPRD_ADMA2_64_ALIGN - 1)) {
			has_unaligned = true;
			break;
		}

	if (has_unaligned && (data->flags & MMC_DATA_READ)) {
		dma_sync_sg_for_cpu(mmc_dev(host->mmc),
			data->sg, data->sg_len, direction);

		align = host->align_buffer;

		for_each_sg(data->sg, sg, host->sg_count, i) {
			if (sg_dma_address(sg) & (SPRD_ADMA2_64_ALIGN - 1)) {
				size = SPRD_ADMA2_64_ALIGN -
					(sg_dma_address(sg) &
					(SPRD_ADMA2_64_ALIGN - 1));
				len = sg_dma_len(sg);
				align_len = min(len, size);
				buffer = sprd_sdhc_kmap_atomic(sg, &flags);
				memcpy(buffer, align, align_len);
				sprd_sdhc_kunmap_atomic(buffer, &flags);

				align += SPRD_ADMA2_64_ALIGN;
			}
		}
	}

	if (data->host_cookie == COOKIE_MAPPED) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg,
			data->sg_len, direction);
		data->host_cookie = COOKIE_UNMAPPED;
	}
}

static void sprd_admd_mode(struct sprd_sdhc_host *host, struct mmc_data *data)
{
	sprd_sdhc_set_64bit_addr(host, 1);
	sprd_sdhc_set_adma2_len(host);
	sprd_sdhc_set_dma(host, SPRD_SDHC_BIT_32ADMA_MOD);
	sprd_sdhc_adma_table_pre(host, data);
	sprd_sdhc_writel(host,
		(u32)host->adma_addr,
		SPRD_SDHC_REG_32_ADMA2_ADDR_L);
	sprd_sdhc_writel(host,
		(u32)((u64)host->adma_addr >> 32),
		SPRD_SDHC_REG_32_ADMA2_ADDR_H);
}

static void sprd_sdmd_mode(struct sprd_sdhc_host *host, struct mmc_data *data)
{
	int sg_cnt;

	sprd_sdhc_set_dma(host, SPRD_SDHC_BIT_SDMA_MOD);
	sg_cnt = sdhci_pre_dma_transfer(host, data);

	if (sg_cnt == 1) {
		if (host->version == SPRD_SDHC_BIT_SPEC_300) {
			sprd_sdhc_writel(host,
				sg_dma_address(data->sg),
				SPRD_SDHC_REG_32_SYS_ADDR);
		} else if (host->version >= SPRD_SDHC_BIT_SPEC_400) {
			u32 addr;

			addr = (u32)(sg_dma_address(data->sg));
			sprd_sdhc_writel(host, addr,
				SPRD_SDHC_REG_32_SDMA_ADDR);
			addr = (u32)((u64)(sg_dma_address(data->sg)) >> 32);
			sprd_sdhc_writel(host, addr,
				SPRD_SDHC_REG_32_ADMA2_ADDR_H);
		}
	}
}

static void sprd_sdhc_dma_unmap(struct sprd_sdhc_host *host)
{
	struct mmc_data *data = host->data;

	if (host->flags & SPRD_USE_ADMA)
		sprd_sdhc_adma_table_post(host, data);
	else {
		if (data->host_cookie == COOKIE_MAPPED) {
			dma_unmap_sg(mmc_dev(host->mmc),
				data->sg, data->sg_len,
				(data->flags & MMC_DATA_READ) ?
				DMA_FROM_DEVICE : DMA_TO_DEVICE);
			data->host_cookie = COOKIE_UNMAPPED;
		}
	}
}

static void sprd_send_cmd(struct sprd_sdhc_host *host, struct mmc_command *cmd)
{
	struct mmc_data *data = cmd->data;

	u32 flag = 0;
	u16 rsp_type = 0;
	int if_has_data = 0;
	int if_mult = 0;
	int if_read = 0;
	int if_dma = 0;
	u16 auto_cmd = SPRD_SDHC_BIT_ACMD_DIS;

	pr_debug("%s(%s)  CMD%d, arg 0x%x, flag 0x%x\n", __func__,
		host->device_name, cmd->opcode, cmd->arg, cmd->flags);
	if (data)
		pr_debug("%s(%s) block size %d, cnt %d\n", __func__,
			host->device_name, data->blksz, data->blocks);

	sprd_sdhc_disable_all_int(host);

	if (cmd->opcode == MMC_ERASE) {
		/*
		 * if it is erase command , it's busy time will long,
		 * so we set long timeout value here.
		 */
		mod_timer(&host->timer, jiffies +
			msecs_to_jiffies(host->mmc->max_busy_timeout + 1000));
		sprd_sdhc_writeb(host,
				SPRD_SDHC_DATA_TIMEOUT_MAX_VAL,
				SPRD_SDHC_REG_8_TIMEOUT);
	} else {
		mod_timer(&host->timer,
			jiffies + (SPRD_SDHC_MAX_TIMEOUT + 1) * HZ);
		sprd_sdhc_writeb(host, host->data_timeout_val,
				SPRD_SDHC_REG_8_TIMEOUT);
	}

	host->cmd = cmd;
	host->data = data;
	if (data) {
		/* set data param */
		WARN_ON((data->blksz * data->blocks > 524288) ||
			(data->blksz > host->mmc->max_blk_size) ||
			(data->blocks > 65535));

		data->bytes_xfered = 0;

		if_has_data = 1;
		if_read = (data->flags & MMC_DATA_READ);
		if_mult = (mmc_op_multi(cmd->opcode) || data->blocks > 1);
		if (if_read && !if_mult)
			flag = SPRD_SDHC_DAT_FILTER_RD_SIGLE;
		else if (if_read && if_mult)
			flag = SPRD_SDHC_DAT_FILTER_RD_MULTI;
		else if (!if_read && !if_mult)
			flag = SPRD_SDHC_DAT_FILTER_WR_SIGLE;
		else
			flag = SPRD_SDHC_DAT_FILTER_WR_MULTI;

		if (!host->auto_cmd_mode)
			flag |= SPRD_SDHC_BIT_INT_ERR_ACMD;

		if_dma = 1;

		sprd_sdhc_set_blk_size(host, data->blksz);
		if (host->version == SPRD_SDHC_BIT_SPEC_300)
			sprd_sdhc_set_16_blk_cnt(host, data->blocks);
		else if (host->version >= SPRD_SDHC_BIT_SPEC_400)
			sprd_sdhc_set_32_blk_cnt(host, data->blocks);
		if (if_mult) {
			if (!host->mrq->sbc && SPRD_SDHC_FLAG_ENABLE_ACMD12)
				auto_cmd = host->auto_cmd_mode;
			else if (host->mrq->sbc && SPRD_SDHC_FLAG_ENABLE_ACMD23)
				auto_cmd = host->auto_cmd_mode;
				sprd_sdhc_set_autocmd23(host);
		}
		if (host->flags & SPRD_USE_ADMA) {
			sprd_admd_mode(host, data);
			flag |= SPRD_SDHC_BIT_INT_ERR_ADMA;
		} else
			sprd_sdmd_mode(host, data);
	}

	sprd_sdhc_writel(host, cmd->arg, SPRD_SDHC_REG_32_ARG);
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1B:
		rsp_type = SPRD_SDHC_RSP1B_5B;
		flag |= SPRD_SDHC_CMD_FILTER_R1B;
		break;
	case MMC_RSP_NONE:
		rsp_type = SPRD_SDHC_RSP0;
		flag |= SPRD_SDHC_CMD_FILTER_R0;
		break;
	case MMC_RSP_R2:
		rsp_type = SPRD_SDHC_RSP2;
		flag |= SPRD_SDHC_CMD_FILTER_R2;
		break;
	case MMC_RSP_R4:
		rsp_type = SPRD_SDHC_RSP3_4;
		flag |= SPRD_SDHC_CMD_FILTER_R1_R4_R6_R7;
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1 & ~MMC_RSP_CRC:
		rsp_type = SPRD_SDHC_RSP1_5_6_7;
		flag |= SPRD_SDHC_CMD_FILTER_R1_R4_R6_R7;
		break;
	default:
		WARN_ON(1);
		break;
	}

	host->int_filter = flag;
	sprd_sdhc_enable_int(host, flag);
	pr_debug("sprd_sdhc %s CMD%d rsp:0x%x intflag:0x%x\n"
		"if_mult:0x%x if_read:0x%x auto_cmd:0x%x if_dma:0x%x\n",
		host->device_name, cmd->opcode, mmc_resp_type(cmd),
		flag, if_mult, if_read, auto_cmd, if_dma);

	sprd_sdhc_set_trans_and_cmd(host, if_mult, if_read, auto_cmd,
		if_mult, if_dma, cmd->opcode, if_has_data, rsp_type);

}

static int irq_err_handle(struct sprd_sdhc_host *host, u32 intmask)
{
	/* some error happened in command */
	if (SPRD_SDHC_INT_FILTER_ERR_CMD & intmask) {
		if (SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT & intmask)
			host->cmd->error = -ETIMEDOUT;
		else
			host->cmd->error = -EILSEQ;
	}

	/* some error happened in data token or command  with R1B */
	if (SPRD_SDHC_INT_FILTER_ERR_DAT & intmask) {
		if (host->data) {
			/* current error is happened in data token */
			if (SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT & intmask)
				host->data->error = -ETIMEDOUT;
			else
				host->data->error = -EILSEQ;
		} else {
			/* current error is happend in response with busy */
			if (SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT & intmask)
				host->cmd->error = -ETIMEDOUT;
			else
				host->cmd->error = -EILSEQ;
		}
	}

	if (host->data) {
		/* Auto cmd12 and cmd23 error is belong to data token error */
		if (SPRD_SDHC_BIT_INT_ERR_ACMD & intmask)
			host->data->error = -EILSEQ;
		else if (SPRD_SDHC_BIT_INT_ERR_ADMA & intmask)
			host->data->error = -EIO;

		sprd_sdhc_dma_unmap(host);
	}

	if ((host->cmd->opcode != MMC_SEND_TUNING_BLOCK) &&
		(host->cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200)) {
		pr_info("sprd_sdhc %s CMD%d int 0x%x\n",
			host->device_name, host->cmd->opcode, intmask);
		dump_sdio_reg(host);
	}
	sprd_sdhc_disable_all_int(host);

	/* if current error happened in data token, we send cmd12 to stop it */
	if ((host->mrq->cmd == host->cmd) && (host->mrq->stop)) {
		sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD |
				SPRD_SDHC_BIT_RST_DAT);
		sprd_send_cmd(host, host->mrq->stop);
	} else {
		/* request finish with error, so reset and stop it */
		sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD |
				SPRD_SDHC_BIT_RST_DAT);
		tasklet_schedule(&host->finish_tasklet);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

static int irq_normal_handle(struct sprd_sdhc_host *host, u32 intmask,
			u32 *ret_intmask)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data = host->data;
	u32 sys_addr;

	/* delete irq that wanted in filter */
	host->int_filter &= ~(SPRD_SDHC_FILTER_NORMAL & intmask);

	if (SPRD_SDHC_BIT_INT_DMA_END & intmask) {
		if (host->version == SPRD_SDHC_BIT_SPEC_300) {
			sys_addr = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_SYS_ADDR);
			sprd_sdhc_writel(host, sys_addr,
					SPRD_SDHC_REG_32_SYS_ADDR);
		} else if (host->version >= SPRD_SDHC_BIT_SPEC_400) {
			sys_addr = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_SDMA_ADDR);
			sprd_sdhc_writel(host, sys_addr,
					SPRD_SDHC_REG_32_SDMA_ADDR);
			sys_addr = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_ADMA2_ADDR_H);
			sprd_sdhc_writel(host, sys_addr,
					SPRD_SDHC_REG_32_ADMA2_ADDR_H);
		}
	}

	if (SPRD_SDHC_BIT_INT_CMD_END & intmask) {
		cmd->error = 0;
		sprd_get_rsp(host);
	}

	if (SPRD_SDHC_BIT_INT_TRAN_END & intmask) {
		if (data) {
			sprd_sdhc_dma_unmap(host);
			data->error = 0;
			data->bytes_xfered = data->blksz * data->blocks;
		} else {
			/* R1B also can produce transfer complete interrupt */
			cmd->error = 0;
		}
	}

	if (!(SPRD_SDHC_FILTER_NORMAL & host->int_filter)) {
		/* current cmd finished */
		sprd_sdhc_disable_all_int(host);
		if (mrq->sbc == cmd)
			sprd_send_cmd(host, mrq->cmd);
		else if ((mrq->cmd == cmd) && (mrq->stop))
			sprd_send_cmd(host, mrq->stop);
		else {
			/* finish with success and stop the request */
			tasklet_schedule(&host->finish_tasklet);
			return IRQ_WAKE_THREAD;
		}
	}

	intmask = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_INT_STATE);
	sprd_sdhc_clear_int(host, intmask);
	intmask &= host->int_filter;
	*ret_intmask = intmask;

	return IRQ_HANDLED;
}

static irqreturn_t sprd_sdhc_irq(int irq, void *param)
{
	u32 intmask;
	struct sprd_sdhc_host *host = (struct sprd_sdhc_host *)param;
	u32 ret_intmask = 0;

	spin_lock(&host->lock);
	/*
	 * maybe sprd_sdhc_timeout() run in one core and _irq() run in
	 * another core, this will panic if access cmd->data
	 */
	if ((!host->mrq) || (!host->cmd)) {
		spin_unlock(&host->lock);
		return IRQ_NONE;
	}

	intmask = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_INT_STATE);
	if (!intmask) {
		spin_unlock(&host->lock);
		return IRQ_NONE;
	} else if (((intmask & SPRD_SDHC_BIT_INT_ERR_CMD_CRC) ||
		(intmask & SPRD_SDHC_BIT_INT_ERR_DATA_CRC)) &&
		(host->cmd->opcode != MMC_SEND_TUNING_BLOCK) &&
		(host->cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200))
		pr_info("%s(%s): CMD%d, dll delay reg: 0x%x\n",
			__func__, host->device_name, host->cmd->opcode,
			sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_DLY));

	pr_debug("%s(%s) CMD%d, intmask 0x%x, filter = 0x%x\n",
		__func__, host->device_name, host->cmd->opcode,
		intmask, host->int_filter);

	/*
	 * sometimes an undesired interrupt will happen, so we must clear
	 * this unused interrupt.
	 */
	sprd_sdhc_clear_int(host, intmask);
	/* just care about the interrupt that we want */
	intmask &= host->int_filter;

	while (intmask) {
		int ret = 0;

		if (SPRD_SDHC_INT_FILTER_ERR & intmask) {
			ret = irq_err_handle(host, intmask);
			if (ret == IRQ_WAKE_THREAD)
				goto out_to_irq_thread;
			else if (ret == IRQ_HANDLED)
				goto out_to_irq;
		} else {
			ret = irq_normal_handle(host, intmask, &ret_intmask);
			if (ret == IRQ_WAKE_THREAD)
				goto out_to_irq_thread;
			else
				intmask = ret_intmask;
		}
	};

out_to_irq:
	spin_unlock(&host->lock);
	return IRQ_HANDLED;

out_to_irq_thread:
	spin_unlock(&host->lock);
	return IRQ_WAKE_THREAD;
}

static void sprd_sdhc_finish_tasklet(unsigned long param)
{
	struct sprd_sdhc_host *host = (struct sprd_sdhc_host *)param;
	unsigned long flags;
	struct mmc_request *mrq;

	del_timer(&host->timer);

	spin_lock_irqsave(&host->lock, flags);
	if (!host->mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	mrq = host->mrq;

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
	sprd_sdhc_runtime_pm_put(host);


}

static void sprd_sdhc_timeout(unsigned long data)
{
	struct sprd_sdhc_host *host = (struct sprd_sdhc_host *)data;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	if (host->mrq) {
		pr_info("%s(%s) Timeout waiting for hardware interrupt!\n",
			__func__, host->device_name);
		dump_sdio_reg(host);
		if (host->data)
			host->data->error = -ETIMEDOUT;
		else if (host->cmd)
			host->cmd->error = -ETIMEDOUT;
		else
			host->mrq->cmd->error = -ETIMEDOUT;

		if (host->data)
			sprd_sdhc_dma_unmap(host);

		sprd_sdhc_disable_all_int(host);
		sprd_sdhc_reset(host,
			SPRD_SDHC_BIT_RST_CMD | SPRD_SDHC_BIT_RST_DAT);
		tasklet_schedule(&host->finish_tasklet);
	}
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sprd_sdhc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int present;

	sprd_sdhc_runtime_pm_get(host);
	present = mmc_gpio_get_cd(host->mmc);
	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;
	/* 1 find whether card is still in slot */
	if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
		if (!present) {
			mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);
			return;
		}
		/* else asume sdcard is present */
	}

	/*
	 * in our control we can not use auto cmd12 and auto cmd23 together
	 * so in following program we use auto cmd23 prior to auto cmd12
	 */
	pr_debug("%s(%s) CMD%d request %d %d %d\n", __func__,
		host->device_name, mrq->cmd->opcode,
		!!mrq->sbc, !!mrq->cmd, !!mrq->stop);
	host->auto_cmd_mode = SPRD_SDHC_BIT_ACMD_DIS;
	if (!mrq->sbc && mrq->stop && SPRD_SDHC_FLAG_ENABLE_ACMD12) {
		host->auto_cmd_mode = SPRD_SDHC_BIT_ACMD12;
		mrq->data->stop = NULL;
		mrq->stop = NULL;
	}

	/* 3 send cmd list */
	if ((mrq->sbc) && SPRD_SDHC_FLAG_ENABLE_ACMD23) {
		host->auto_cmd_mode = SPRD_SDHC_BIT_ACMD23;
		mrq->stop = NULL;
		sprd_send_cmd(host, mrq->cmd);
	} else if (mrq->sbc) {
		mrq->stop = NULL;
		sprd_send_cmd(host, mrq->sbc);
	} else {
		sprd_send_cmd(host, mrq->cmd);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sprd_signal_voltage_on_off(struct sprd_sdhc_host *host, u32 on_off)
{
	if (IS_ERR(host->mmc->supply.vqmmc)) {
		pr_err("%s(%s) there is no signal voltage!\n",
			__func__, host->device_name);
		return;
	}

	if (on_off) {
		if (!regulator_is_enabled(host->mmc->supply.vqmmc)) {
			if (regulator_enable(host->mmc->supply.vqmmc))
				pr_info("%s signal voltage enable fail!\n",
					host->device_name);
			else if (regulator_is_enabled(host->mmc->supply.vqmmc))
				pr_info("%s signal voltage enable success!\n",
					host->device_name);
			else
				pr_info("%s signal voltage enable hw fail!\n",
					host->device_name);
		}
	} else {
		if ((strcmp(host->device_name, "sdio_sd") == 0)) {
			if (pinctrl_select_state(
				host->pinctrl, host->pins_default))
				pr_err("%s switch vddsdio ms pinctrl failed\n",
					__func__);
			else
				pr_info("%s set vddsdio ms bit as 0\n",
					__func__);
		}

		if (regulator_is_enabled(host->mmc->supply.vqmmc)) {
			if (regulator_disable(host->mmc->supply.vqmmc))
				pr_info("%s signal voltage disable fail\n",
					host->device_name);
			else if (!regulator_is_enabled(
					host->mmc->supply.vqmmc))
				pr_info("%s signal voltage disable success!\n",
					host->device_name);
			else
				pr_info("%s signal voltage disable hw fail\n",
					host->device_name);
		}
	}
}

/*
 * 1 This votage is always poweron
 * 2 initial votage is 2.7v~3.6v
 * 3 It can be reconfig to 1.7v~1.95v
 */
static int sprd_sdhc_set_vqmmc(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int err;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (IS_ERR(mmc->supply.vqmmc)) {
		/* there are no 1.8v signal votage. */
		err = 0;
		pr_err("%s(%s) There is no signalling voltage\n",
			__func__, host->device_name);
		goto out;
	}

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		/*
		 * SPRD LDO can't support 3.0v signal voltage for emmc,
		 * so we must return
		 */
		if (strcmp(host->device_name, "sdio_emmc") == 0) {
			err = -EINVAL;
			goto out;
		}
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = regulator_set_voltage(mmc->supply.vqmmc,
					    3000000, 3000000);
		spin_lock_irqsave(&host->lock, flags);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = regulator_set_voltage(mmc->supply.vqmmc,
					1800000, 1800000);
		spin_lock_irqsave(&host->lock, flags);
		if ((strcmp(host->device_name, "sdio_sd") == 0)) {
			err = pinctrl_select_state(
				host->pinctrl, host->pins_uhs);
			if (err)
				pr_err("switch vddsdio ms pinctrl failed\n");
			else
				pr_info("set vddsdio ms bit as 1 ok\n");

			udelay(300);
			sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD |
					SPRD_SDHC_BIT_RST_DAT);
		}
		break;
	case MMC_SIGNAL_VOLTAGE_120:
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = regulator_set_voltage(mmc->supply.vqmmc,
					1100000, 1300000);
		spin_lock_irqsave(&host->lock, flags);
		break;
	default:
		/* No signal voltage switch required */
		pr_err("%s(%s) 0x%x is an unsupportted signal voltage\n",
			__func__, host->device_name, ios->signal_voltage);
		err = EIO;
		break;
	}

	if (likely(!err))
		host->ios.signal_voltage = ios->signal_voltage;
	else
		pr_warn("%s(%s): switching to signalling voltage failed\n",
			__func__, host->device_name);

out:
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	sprd_sdhc_runtime_pm_put(host);
	return err;
}

static void sprd_sdhc_enable_dpll(struct sprd_sdhc_host *host)
{
	u32 tmp = 0;

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	tmp &= ~(SPRD_SDHC_DLL_EN | SPRD_SDHC_DLL_ALL_CPST_EN);
	sprd_sdhc_writel(host, tmp, SPRD_SDHC_REG_32_DLL_CFG);
	mdelay(1);

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	tmp |= SPRD_SDHC_DLL_ALL_CPST_EN | SPRD_SDHC_DLL_SEARCH_MODE |
		SPRD_SDHC_DLL_INIT_COUNT | SPRD_SDHC_DLL_PHA_INTERNAL;
	sprd_sdhc_writel(host, tmp, SPRD_SDHC_REG_32_DLL_CFG);
	mdelay(1);

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	tmp |= SPRD_SDHC_DLL_EN;
	sprd_sdhc_writel(host, tmp, SPRD_SDHC_REG_32_DLL_CFG);
	mdelay(1);

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS0);
	while (((tmp & SPRD_SDHC_DLL_LOCKED) != SPRD_SDHC_DLL_LOCKED) ||
		((tmp & SPRD_SDHC_DLL_ERROR) == SPRD_SDHC_DLL_ERROR)) {
		pr_info("+++++++++ sprd sdhc dpll locked faile +++++++++!\n");
		pr_info("sprd sdhc dpll register DLL_STS0 : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
		pr_info("sprd sdhc dpll register DLL_CFG : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_DLY);
		pr_info("sprd sdhc dpll register DLL_DLY : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS1);
		pr_info("sprd sdhc dpll register DLL_STS1 : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS0);
		mdelay(1);
	}

	pr_info("%s(%s): dpll locked done\n", __func__, host->device_name);
}

static void sprd_sdhc_aon_clksource(struct sprd_sdhc_host *host,
	struct mmc_ios *ios)
{

	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	clk_disable_unprepare(host->sdio_ckg);
	if (ios->clock <= 400000)
		clk_set_parent(host->clk, host->clk_source2);
	else
		clk_set_parent(host->clk, host->clk_source);

	clk_set_rate(host->clk,  2 * ios->clock);

	clk_prepare_enable(host->clk);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->sdio_ckg);
}

static void sprd_sdhc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u8 ctrl = 0;
	u8 clkchg_flag = 0;

	pr_debug("++++++++++sprd-sdhc %s ios : ++++++++++\n"
		"sprd-sdhc clock = %d---->%d\n"
		"sprd-sdhc vdd = %d---->%d\n"
		"sprd-sdhc bus_mode = %d---->%d\n"
		"sprd-sdhc chip_select = %d---->%d\n"
		"sprd-sdhc power_mode = %d---->%d\n"
		"sprd-sdhc bus_width = %d---->%d\n"
		"sprd-sdhc timing = %d---->%d\n"
		"sprd-sdhc signal_voltage = %d---->%d\n"
		"sprd-sdhc drv_type = %d---->%d\n",
		host->device_name,
		host->ios.clock, ios->clock,
		host->ios.vdd, ios->vdd,
		host->ios.bus_mode, ios->bus_mode,
		host->ios.chip_select, ios->chip_select,
		host->ios.power_mode, ios->power_mode,
		host->ios.bus_width, ios->bus_width,
		host->ios.timing, ios->timing,
		host->ios.signal_voltage, ios->signal_voltage,
		host->ios.drv_type, ios->drv_type);

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (ios->clock == 0) {
		sprd_sdhc_all_clk_off(host);
		host->ios.clock = 0;
	} else if (ios->clock != host->ios.clock) {
		sprd_sdhc_sd_clk_off(host);
		clkchg_flag = 1;
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		sprd_sdhc_aon_clksource(host, ios);
		spin_lock_irqsave(&host->lock, flags);
		sprd_sdhcr7_clk_set_and_on(host);
		sprd_sdhc_sd_clk_on(host);
		host->ios.clock = ios->clock;
		host->data_timeout_val = sprd_sdhc_calc_timeout(ios->clock,
						SPRD_SDHC_MAX_TIMEOUT);
		mmc->max_busy_timeout = (1 << 31) / (ios->clock / 1000);
	}

	if (ios->power_mode != host->ios.power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);
			sprd_signal_voltage_on_off(host, 0);
			if (!IS_ERR(mmc->supply.vmmc))
				mmc_regulator_set_ocr(host->mmc,
						mmc->supply.vmmc, 0);
			spin_lock_irqsave(&host->lock, flags);
			sprd_reset_ios(host);
			host->ios.power_mode = ios->power_mode;
			break;
		case MMC_POWER_ON:
		case MMC_POWER_UP:
			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);
			if (!IS_ERR(mmc->supply.vmmc))
				mmc_regulator_set_ocr(host->mmc,
					mmc->supply.vmmc, ios->vdd);
			sprd_signal_voltage_on_off(host, 1);
			spin_lock_irqsave(&host->lock, flags);
			host->ios.power_mode = ios->power_mode;
			host->ios.vdd = ios->vdd;
			break;
		}
	}

	/* flash power voltage select */
	if (ios->vdd != host->ios.vdd) {
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		if (!IS_ERR(mmc->supply.vmmc)) {
			pr_info("%s(%s) 3.0 %d!\n", __func__,
				host->device_name, ios->vdd);
			mmc_regulator_set_ocr(host->mmc,
				mmc->supply.vmmc, ios->vdd);
		}
		spin_lock_irqsave(&host->lock, flags);
		host->ios.vdd = ios->vdd;
	}

	if (ios->bus_width != host->ios.bus_width) {
		sprd_sdhc_set_buswidth(host, ios->bus_width);
		host->ios.bus_width = ios->bus_width;
	}

	if (ios->timing != host->ios.timing) {
		/* 1 first close SD clock */
		sprd_sdhc_sd_clk_off(host);
		/* 2 set timing mode, timing specification used*/
		switch (ios->timing) {
		case MMC_TIMING_MMC_HS:
		case MMC_TIMING_SD_HS:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR25;
			break;
		case MMC_TIMING_UHS_SDR12:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR12;
			break;
		case MMC_TIMING_UHS_SDR25:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR25;
			break;
		case MMC_TIMING_UHS_SDR50:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR50;
			break;
		case MMC_TIMING_UHS_SDR104:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR104;
			break;
		case MMC_TIMING_UHS_DDR50:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_DDR50;
			break;
		case MMC_TIMING_MMC_DDR52:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_DDR50;
			break;
		case MMC_TIMING_MMC_HS200:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_HS200;
			break;
		case MMC_TIMING_MMC_HS400:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_HS400;
			break;
		default:
			break;
		}
		sprd_sdhc_set_uhs_mode(host, ctrl);

		/* 3 open SD clock */
		sprd_sdhc_sd_clk_on(host);
		host->ios.timing = ios->timing;
	}

    /*
     * Under enhanced strobe mode, emmc has alreadly set delay timing in
     * ops->hs400_enhanced_strobe().
     * After ops->hs400_enhanced_strobe(), mmc_set_bus_speed which set clock
     * to 200MHz will be called. In this case sprd_set_delay_value() may think
     * emmc is running in hs400 mode, and it will set hs400 delay timing not
     * enhanced strobe.
     */
	if (!ios->enhanced_strobe)
		sprd_set_delay_value(host);

	if ((ios->clock > 52000000) && (clkchg_flag == 1)) {
		clkchg_flag = 0;
		sprd_sdhc_enable_dpll(host);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
}

int sprd_sdhc_set_nonremovable(struct mmc_host *host)
{
	host->caps |= MMC_CAP_NONREMOVABLE;
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_sdhc_set_nonremovable);

static int sprd_sdhc_get_cd(struct mmc_host *mmc)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int gpio_cd;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (host->mmc->caps & MMC_CAP_NONREMOVABLE) {
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		sprd_sdhc_runtime_pm_put(host);
		return 1;
	}

	gpio_cd = mmc_gpio_get_cd(host->mmc);
	if (IS_ERR_VALUE(gpio_cd))
		gpio_cd = 1;
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
	return !!gpio_cd;
}

static int sprd_sdhc_card_busy(struct mmc_host *mmc)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 present_state;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	/* Check whether DAT[3:0] is 0000 */
	present_state = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_PRES_STATE);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
	return !(present_state & SPRD_SDHC_DATA_LVL_MASK);
}

static void sprd_sdhc_emmc_hw_reset(struct mmc_host *mmc)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int val;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	val = sprd_sdhc_readb(host, SPRD_SDHC_REG_8_RST);
	val &= ~SPRD_SDHC_BIT_RST_EMMC;
	sprd_sdhc_writeb(host, val, SPRD_SDHC_REG_8_RST);
	udelay(10);

	val |= SPRD_SDHC_BIT_RST_EMMC;
	sprd_sdhc_writeb(host, val, SPRD_SDHC_REG_8_RST);
	udelay(300);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
}

static int sprd_sdhc_prepare_hs400_tuning(struct mmc_host *mmc,
	struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	host->flags |= SPRD_HS400_TUNING;
	host->dll_dly = host->timing_dly->hs400_dly;
	sprd_sdhc_writel(host, host->dll_dly, SPRD_SDHC_REG_32_DLL_DLY);
	pr_info("%s (%s) hs400 default timing delay value: 0x%08x\n",
		__func__, host->device_name, host->dll_dly);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);

	return 0;
}

static int sprd_sdhc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);

	unsigned long flags;
	int err = 0;
	int i = 0;
	bool value, first_vl, prev_vl = 0;
	struct ranges_t {
		int start;
		int end;
	};
	struct ranges_t *ranges;

	int length = 0;
	unsigned int range_count = 0;
	int longest_range_len = 0;
	int longest_range = 0;
	int mid_step;
	int final_phase = 0;
	u32 dll_cfg = 0;
	u32 dll_cnt = 0;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD | SPRD_SDHC_BIT_RST_DAT);

	dll_cfg = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	dll_cfg &= ~(0xf << 24);
	sprd_sdhc_writel(host, dll_cfg, SPRD_SDHC_REG_32_DLL_CFG);
	dll_cnt = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS0) & 0xff;
	length = (dll_cnt * 105) / 100;
	pr_info("%s(%s): dll config 0x%08x, dll count %d, tuning length: %d\n",
		__func__, host->device_name, dll_cfg, dll_cnt, length);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	ranges = kmalloc_array(length + 1, sizeof(*ranges), GFP_KERNEL);
	if (!ranges)
		return -ENOMEM;
	spin_lock_irqsave(&host->lock, flags);

	do {
		if (host->flags & SPRD_HS400_TUNING)
			sprd_sdhc_writeb(host, i, SPRD_SDHC_REG_8_CMDRD_DLY);
		else
			sprd_sdhc_writeb(host, i, SPRD_SDHC_REG_8_POSRD_DLY);

		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		value = !mmc_send_tuning(mmc, opcode, NULL);
		spin_lock_irqsave(&host->lock, flags);
		if (i == 0)
			first_vl = value;

		if ((!prev_vl) && value) {
			range_count++;
			ranges[range_count - 1].start = i;
		}
		if (value) {
			pr_debug("%s tuning ok: %d\n", host->device_name, i);
			ranges[range_count - 1].end = i;
		} else
			pr_debug("%s tuning fail: %d\n", host->device_name, i);

		prev_vl = value;
	} while (++i <= length);

	if (range_count == 0) {
		pr_warn("%s(%s): all tuning phases fail!\n",
			__func__, host->device_name);
		err = -EIO;
		goto out;
	}

	if ((range_count > 1) && first_vl && value) {
		ranges[0].start = ranges[range_count - 1].start;
		range_count--;
	}

	if (ranges[0].start == 0 && ranges[0].end == length - 1) {
		pr_warn("%s(%s): no tuning phases fail!\n",
			__func__, host->device_name);
		err = -EIO;
		goto out;
	}

	for (i = 0; i < range_count; i++) {
		int len = (ranges[i].end - ranges[i].start + 1);

		if (len < 0)
			len += length;

		pr_info("%s(%s): good tuning phase range %d ~ %d\n",
			__func__, host->device_name,
			ranges[i].start, ranges[i].end);

		if (longest_range_len < len) {
			longest_range_len = len;
			longest_range = i;
		}

	}
	pr_info("%s(%s): the best tuning step range %d-%d(the length is %d)\n",
		__func__, host->device_name, ranges[longest_range].start,
		ranges[longest_range].end, longest_range_len);

	mid_step = ranges[longest_range].start + longest_range_len / 2;
	mid_step %= length;

	dll_cfg |= 0xf << 24;
	sprd_sdhc_writel(host, dll_cfg, SPRD_SDHC_REG_32_DLL_CFG);

	final_phase = (mid_step * 256) / dll_cnt;

	if (host->flags & SPRD_HS400_TUNING) {
		host->dll_dly &= ~SPRD_CMD_DLY_MASK;
		host->dll_dly |= (final_phase << 8);
		host->timing_dly->hs400_dly = host->dll_dly;
	} else {
		host->dll_dly &= ~(SPRD_CMD_DLY_MASK | SPRD_POSRD_DLY_MASK |
				SPRD_NEGRD_DLY_MASK);
		host->dll_dly |= (((final_phase << 8) & SPRD_CMD_DLY_MASK) |
				((final_phase << 16) & SPRD_POSRD_DLY_MASK) |
				((final_phase << 24) & SPRD_NEGRD_DLY_MASK));
	}

	pr_info("%s(%s): the best step %d, phase 0x08%x, delay value 0x%08x\n",
		__func__, host->device_name, mid_step,
		final_phase, host->dll_dly);
	sprd_sdhc_writel(host, host->dll_dly, SPRD_SDHC_REG_32_DLL_DLY);
	err = 0;

out:
	host->flags &= ~SPRD_HS400_TUNING;
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);

	return err;
}

static void sprd_sdhc_hs400_enhanced_strobe(struct mmc_host *mmc,
	struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (ios->enhanced_strobe) {
		sprd_sdhc_sd_clk_off(host);
		sprd_sdhc_set_uhs_mode(host, SPRD_SDHC_BIT_TIMING_MODE_HS400ES);
		sprd_sdhc_sd_clk_on(host);
		host->dll_dly = host->timing_dly->hs400es_dly;
		sprd_sdhc_writel(host, host->dll_dly, SPRD_SDHC_REG_32_DLL_DLY);
		pr_info("%s(%s): hs400es final timing delay value: 0x%08x\n",
			__func__, host->device_name, host->dll_dly);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
}

static void sprd_sdhc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
				int err)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (data->host_cookie == COOKIE_GIVEN ||
			data->host_cookie == COOKIE_MAPPED) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				 data->flags & MMC_DATA_WRITE ?
				 DMA_TO_DEVICE : DMA_FROM_DEVICE);
		data->host_cookie = COOKIE_UNMAPPED;
	}
}

static void sprd_sdhc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq,
			       bool is_first_req)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);

	mrq->data->host_cookie = COOKIE_UNMAPPED;
	sdhci_pre_dma_transfer(host, mrq->data);
}

static const struct mmc_host_ops sprd_sdhc_ops = {
	.request = sprd_sdhc_request,
	.post_req = sprd_sdhc_post_req,
	.pre_req = sprd_sdhc_pre_req,
	.set_ios = sprd_sdhc_set_ios,
	.get_cd = sprd_sdhc_get_cd,
	.start_signal_voltage_switch = sprd_sdhc_set_vqmmc,
	.card_busy = sprd_sdhc_card_busy,
	.hw_reset = sprd_sdhc_emmc_hw_reset,
	.prepare_hs400_tuning = sprd_sdhc_prepare_hs400_tuning,
	.execute_tuning = sprd_sdhc_execute_tuning,
	.hs400_enhanced_strobe = sprd_sdhc_hs400_enhanced_strobe,
};

static int sprd_get_dt_resource(struct platform_device *pdev,
		struct sprd_sdhc_host *host)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	host->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->res)
		return -ENOENT;

	host->ioaddr = devm_ioremap_resource(&pdev->dev, host->res);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		dev_err(&pdev->dev, "can not map iomem: %d\n", ret);
		goto err;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto err;
	}

	host->clk = devm_clk_get(&pdev->dev, "sdio");
	if (IS_ERR(host->clk)) {
		dev_err(&pdev->dev,
			"can't get the clock dts config: sdio\n");
		goto err;
	}

	host->clk_source = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(host->clk_source)) {
		dev_err(&pdev->dev,
			"can't get the clock dts config: source\n");
		goto err;
	}

	host->clk_source2 = devm_clk_get(&pdev->dev, "source2");
	if (IS_ERR(host->clk_source2)) {
		dev_err(&pdev->dev,
			"can't get the clock dts config: source2\n");
		goto err;
	}

	clk_set_parent(host->clk, host->clk_source);
	host->base_clk = clk_get_rate(host->clk_source);
	host->sdio_ahb = devm_clk_get(&pdev->dev, "enable");

	if (IS_ERR(host->sdio_ahb)) {
		dev_err(&pdev->dev,
			"sdio can't get the clock dts config: enable\n");
		goto err;
	}

	host->sdio_ckg = devm_clk_get(&pdev->dev, "ckg_eb");
	if (IS_ERR(host->sdio_ckg)) {
		dev_err(&pdev->dev,
			"sdio can't get the clock dts config: ckg_eb\n");
	}

	ret = of_property_read_string(np, "sprd,name", &host->device_name);
	if (ret) {
		dev_err(&pdev->dev,
			"can not read the property of sprd name\n");
		goto err;
	}

	host->detect_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	if (!gpio_is_valid(host->detect_gpio))
		host->detect_gpio = -1;

	if (sprd_get_delay_value(pdev))
		goto out;

	if ((strcmp(host->device_name, "sdio_sd") == 0)) {
		host->pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(host->pinctrl)) {
			dev_err(&pdev->dev, "can not find pinctrl\n");
			goto out;
		}

		host->pins_uhs =
			pinctrl_lookup_state(host->pinctrl, "sd0_ms_1");
		if (IS_ERR(host->pins_uhs)) {
			dev_err(&pdev->dev, "can not find pinctrl uhs\n");
			goto out;
		}

		host->pins_default =
			pinctrl_lookup_state(host->pinctrl, "sd0_ms_0");
		if (IS_ERR(host->pins_default)) {
			dev_err(&host->pdev->dev, "can not find pins_default\n");
			goto out;
		}
	}

out:
	return 0;

err:
	dev_err(&pdev->dev, "sprd_sdhc get basic resource fail\n");
	return ret;
}

static int sprd_get_ext_resource(struct sprd_sdhc_host *host)
{
	int err;
	struct mmc_host *mmc = host->mmc;

	host->data_timeout_val = 0;

	mmc_regulator_get_supply(mmc);
	host->mmc = mmc;
	err = request_irq(host->irq, sprd_sdhc_irq,
			IRQF_SHARED, mmc_hostname(host->mmc), host);

	if (err) {
		pr_err("%s: can not request irq\n", host->device_name);
		return err;
	}

	tasklet_init(&host->finish_tasklet,
		sprd_sdhc_finish_tasklet, (unsigned long)host);
	/* init timer */
	setup_timer(&host->timer, sprd_sdhc_timeout, (unsigned long)host);

	return 0;
}

static void sprd_set_mmc_struct(struct sprd_sdhc_host *host,
				struct mmc_host *mmc)
{
	struct device_node *np = host->pdev->dev.of_node;
	u32 host_caps;

	mmc = host->mmc;
	mmc->ops = &sprd_sdhc_ops;
	mmc->f_max = host->base_clk;
	mmc->f_min = 400000;
	host->flags = 0;

	mmc->caps = MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED |
		MMC_CAP_ERASE | MMC_CAP_CMD23;

	mmc_of_parse(mmc);
	mmc_of_parse_voltage(np, &host->ocr_mask);
    /*
     * if there is not regulator device in some of the SPRD platform, the
     * regulator driver will not be used and the REGULATOR will not be configed
     * in defconfig, in this case the mmc->supply.vmmc will be NULL.
     */

	host_caps = sprd_sdhc_readw(host, SPRD_SDHC_REG_16_HOST_CAP);
	if ((host_caps & SPRD_SDHC_BIT_ADMA_SUPPORT) &&
			SPRD_SDHC_FLAG_USE_ADMA)
		host->flags |= SPRD_USE_ADMA;

	mmc->ocr_avail = 0x40000;
	mmc->ocr_avail_sdio = mmc->ocr_avail;
	mmc->ocr_avail_sd = mmc->ocr_avail;
	mmc->ocr_avail_mmc = mmc->ocr_avail;

	mmc->max_current_330 = SPRD_SDHC_MAX_CUR;
	mmc->max_current_300 = SPRD_SDHC_MAX_CUR;
	mmc->max_current_180 = SPRD_SDHC_MAX_CUR;

	mmc->max_req_size = 524288;	/* 512k */
	mmc->max_blk_count = 65535;
	mmc->max_blk_size =  512 << (host_caps & SPRD_SDHC_BIT_BLOCK_SIZE_MASK);
	if (host->flags & SPRD_USE_ADMA) {
		mmc->max_segs = 128;
		mmc->max_seg_size = 65536;
		host->adma_desc = dma_alloc_coherent(mmc_dev(mmc),
							ADMA_SIZE,
							&host->adma_addr,
							GFP_KERNEL);
		host->align_buffer = dma_alloc_coherent(mmc_dev(mmc),
						      SPRD_ALIGN_BUFFER_SZ,
						      &host->align_addr,
						      GFP_KERNEL);

		if (!host->adma_desc || !host->align_buffer) {
			if (host->adma_desc)
				dma_free_coherent(mmc_dev(mmc), ADMA_SIZE,
					  host->adma_desc, host->adma_addr);
			if (host->align_buffer)
				dma_free_coherent(mmc_dev(mmc),
				SPRD_ALIGN_BUFFER_SZ,
				host->align_buffer, host->align_addr);

			host->flags &= ~SPRD_USE_ADMA;
			mmc->max_segs = 1;
			mmc->max_seg_size = mmc->max_req_size;
			host->adma_desc = NULL;
			host->align_buffer = NULL;
			pr_warn("%s: Unable to allocate ADMA buffers. back to SDMA.\n",
				host->device_name);
		} else
			pr_info("%s using ADMA2\n", host->device_name);
	} else {
		mmc->max_segs = 1;
		mmc->max_seg_size = mmc->max_req_size;
		host->adma_desc = NULL;
		host->align_buffer = NULL;
		host->dma_mask = DMA_BIT_MASK(64);
		mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
	}

	pr_info("%s(%s): ocr avail = 0x%x, base clock = %u\n"
		"pm_caps = 0x%x, caps: 0x%x, caps2: 0x%x,\n"
		"host->flags:0x%x, host_caps:0x%x\n",
		__func__, host->device_name, mmc->ocr_avail,
		host->base_clk, mmc->pm_caps, mmc->caps,
		mmc->caps2, host->flags, host_caps);
}

#ifdef CONFIG_PM_SLEEP
static int sprd_sdhc_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);

	sprd_sdhc_runtime_pm_get(host);
	disable_irq(host->irq);
	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	clk_disable_unprepare(host->sdio_ckg);

	return 0;
}

static int sprd_sdhc_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	struct mmc_ios ios;

	clk_prepare_enable(host->sdio_ckg);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->clk);
	enable_irq(host->irq);

	ios = host->mmc->ios;
	sprd_reset_ios(host);
	host->mmc->ops->set_ios(host->mmc, &ios);

	sprd_sdhc_runtime_pm_put(host);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int sprd_sdhc_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	sprd_sdhc_disable_all_int(host);
	sprd_sdhc_all_clk_off(host);
	spin_unlock_irqrestore(&host->lock, flags);

	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	clk_disable_unprepare(host->sdio_ckg);
	synchronize_irq(host->irq);

	return 0;
}

static int sprd_sdhc_runtime_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	unsigned long flags;

	clk_prepare_enable(host->sdio_ckg);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->clk);
	spin_lock_irqsave(&host->lock, flags);
	if (host->ios.clock) {
		sprd_sdhc_sd_clk_off(host);
		sprd_sdhcr7_clk_set_and_on(host);
		sprd_sdhc_sd_clk_on(host);
	}
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	return 0;
}
#endif

static const struct dev_pm_ops sprd_sdhc_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sprd_sdhc_suspend, sprd_sdhc_resume)
	SET_RUNTIME_PM_OPS(sprd_sdhc_runtime_suspend,
		sprd_sdhc_runtime_resume, NULL)
};

static const struct of_device_id sprd_sdhc_of_match[] = {
	{.compatible = "sprd,sdhc-r7"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sprd_sdhc_of_match);

static int sprd_sdhc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct sprd_sdhc_host *host;
	int ret;

	/* globe resource */
	mmc = mmc_alloc_host(sizeof(struct sprd_sdhc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "no memory for mmc host\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;
	spin_lock_init(&host->lock);
	platform_set_drvdata(pdev, host);

	/* get basic resource from device tree */
	ret = sprd_get_dt_resource(pdev, host);
	if (ret) {
		dev_err(&pdev->dev, "fail to get basic resource: %d\n", ret);
		goto err_free_host;
	}

	ret = sprd_get_ext_resource(host);
	if (ret) {
		dev_err(&pdev->dev, "fail to get external resource: %d\n", ret);
		goto err_free_host;
	}

	clk_prepare_enable(host->clk);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->sdio_ckg);
	sprd_reset_ios(host);

	sprd_set_mmc_struct(host, mmc);

	host->version = sprd_sdhc_readw(host, SPRD_SDHC_REG_16_HOST_VER);
	if (host->version == SPRD_SDHC_BIT_SPEC_300)
		sprd_sdhc_set_64bit_addr(host, 0);
	else if (host->version >= SPRD_SDHC_BIT_SPEC_400)
		sprd_sdhc_set_64bit_addr(host, 1);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, 1);

	mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
	/* add host */
	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host: %d\n", ret);
		goto err_free_host;
	}

	if (-1 != host->detect_gpio) {
		mmc->caps &= ~MMC_CAP_NONREMOVABLE;
		mmc_gpio_request_cd(mmc, host->detect_gpio, 0);
	}

	dev_info(&pdev->dev,
		"%s[%s] host controller, irq %d\n",
		host->device_name, mmc_hostname(mmc), host->irq);

	return 0;

err_free_host:
	mmc_free_host(mmc);
	return ret;
}

static int sprd_sdhc_remove(struct platform_device *pdev)
{
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc = host->mmc;

	mmc_remove_host(mmc);
	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);

	if (host->adma_desc)
		dma_free_coherent(mmc_dev(mmc), ADMA_SIZE,
			host->adma_desc, host->adma_addr);
	if (host->align_buffer)
		dma_free_coherent(mmc_dev(mmc), SPRD_ALIGN_BUFFER_SZ,
			  host->align_buffer, host->align_addr);
	host->adma_desc = NULL;
	host->align_buffer = NULL;

	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver sprd_sdhc_driver = {
	.probe = sprd_sdhc_probe,
	.remove = sprd_sdhc_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(sprd_sdhc_of_match),
		.pm = &sprd_sdhc_pmops,
	},
};

module_platform_driver(sprd_sdhc_driver);

MODULE_DESCRIPTION("Spreadtrum sdio host controller driver");
MODULE_LICENSE("GPL");
