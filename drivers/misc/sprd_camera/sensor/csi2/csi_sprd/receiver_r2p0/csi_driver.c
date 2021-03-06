/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <video/sprd_mm.h>
#include "csi_api.h"
#include "csi_driver.h"
#include "sprd_sensor_core.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "csi_driver: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


#define CSI_MASK0			0x1FFFFFF
#define CSI_MASK1			0xFFFFFF

#define PHY_TESTCLR			BIT_0
#define PHY_TESTCLK			BIT_1
#define PHY_TESTDIN			0xFF
#define PHY_TESTDOUT			0xFF00
#define PHY_TESTEN			BIT_16

#define IPG_IMAGE_H_MASK		(0x1ff << 21)
#define IPG_COLOR_BAR_W_MASK		(0xff << 13)
#define IPG_IMAGE_W_MASK		(0x1FF << 4)
#define IPG_HSYNC_EN_MASK		BIT_3
#define IPG_COLOR_BAR_MODE_MASK	BIT_2
#define IPG_IMAGE_MODE_MASK		BIT_1
#define IPG_ENABLE_MASK			BIT_0

#define IPG_IMAGE_W			1280
#define IPG_IMAGE_H			960

#define IPG_IMAGE_H_REG			(((IPG_IMAGE_H)/8) << 21)
#define IPG_COLOR_BAR_W			(((IPG_IMAGE_W)/24) << 13)
#define IPG_IMAGE_W_REG			(((IPG_IMAGE_W)/16) << 4)
#define IPG_HSYNC_EN			(0 << 3)
#define IPG_COLOR_BAR_MODE		(0 << 2)
#define IPG_IMAGE_MODE			(1 << 1)   /*0: YUV 1:RAW*/
#define IPG_ENABLE				(1 << 0)

#define IPG_BAYER_PATTERN_MASK		0x3
#define IPG_BAYER_PATTERN_BGGR		0
#define IPG_BAYER_PATTERN_RGGB		1
#define IPG_BAYER_PATTERN_GBRG		2
#define IPG_BAYER_PATTERN_GRBG		3

#define IPG_BAYER_B_MASK		(0x3FF << 20)
#define IPG_BAYER_G_MASK		(0x3FF << 10)
#define IPG_BAYER_R_MASK		(0x3FF << 0)

#define IPG_RAW10_CFG0_B		(0 << 20)
#define IPG_RAW10_CFG0_G		(0 << 10)
#define IPG_RAW10_CFG0_R		0x3ff

#define IPG_RAW10_CFG1_B		(0 << 20)
#define IPG_RAW10_CFG1_G		(0x3FF << 10)
#define IPG_RAW10_CFG1_R		0

#define IPG_RAW10_CFG2_B		(0x3ff << 20)
#define IPG_RAW10_CFG2_G		(0 << 10)
#define IPG_RAW10_CFG2_R		0

#define IPG_YUV_CFG0_B		(0x51 << 16)
#define IPG_YUV_CFG0_G		(0x5f << 8)
#define IPG_YUV_CFG0_R		0xf0

#define IPG_YUV_CFG1_B		(0x91 << 16)
#define IPG_YUV_CFG1_G		(0x36 << 8)
#define IPG_YUV_CFG1_R		0x22

#define IPG_YUV_CFG2_B		(0xd2 << 16)
#define IPG_YUV_CFG2_G		(0x10 << 8)
#define IPG_YUV_CFG2_R		0x92

#define IPG_V_BLANK_MASK		(0xFFF << 13)
#define IPG_H_BLANK_MASK		0x1FFF
#define IPG_V_BLANK			(0x400 << 13)
#define IPG_H_BLANK			(0x500)

static unsigned long s_csi_regbase[SPRD_SENSOR_ID_MAX];
static unsigned long csi_dump_regbase[CSI_MAX_COUNT];

int csi_reg_base_save(struct csi_dt_node_info *dt_info, int32_t idx)
{
	if (!dt_info) {
		pr_err("fail to get valid dt_info ptr\n");
		return -EINVAL;
	}

	s_csi_regbase[idx] = dt_info->reg_base;
	csi_dump_regbase[dt_info->controller_id] = dt_info->reg_base;
	return 0;
}

void csi_ipg_mode_cfg(uint32_t idx, int enable)
{
	if (enable) {
		CSI_REG_MWR(idx, MODE_CFG,
			IPG_IMAGE_H_MASK, IPG_IMAGE_H_REG);
		CSI_REG_MWR(idx, MODE_CFG,
			IPG_COLOR_BAR_W_MASK, IPG_COLOR_BAR_W);
		CSI_REG_MWR(idx, MODE_CFG, IPG_IMAGE_W_MASK, IPG_IMAGE_W_REG);
		CSI_REG_MWR(idx, MODE_CFG, IPG_HSYNC_EN_MASK, IPG_HSYNC_EN);
		CSI_REG_MWR(idx, MODE_CFG, IPG_COLOR_BAR_MODE_MASK,
						IPG_COLOR_BAR_MODE);
		CSI_REG_MWR(idx, MODE_CFG, IPG_IMAGE_MODE_MASK, IPG_IMAGE_MODE);

		CSI_REG_MWR(idx, IPG_RAW10_CFG0,
			IPG_BAYER_B_MASK, IPG_RAW10_CFG0_B);
		CSI_REG_MWR(idx, IPG_RAW10_CFG0,
			IPG_BAYER_G_MASK, IPG_RAW10_CFG0_G);
		CSI_REG_MWR(idx, IPG_RAW10_CFG0,
			IPG_BAYER_R_MASK, IPG_RAW10_CFG0_R);
		CSI_REG_MWR(idx, IPG_RAW10_CFG1,
			IPG_BAYER_B_MASK, IPG_RAW10_CFG1_B);
		CSI_REG_MWR(idx, IPG_RAW10_CFG1,
			IPG_BAYER_G_MASK, IPG_RAW10_CFG1_G);
		CSI_REG_MWR(idx, IPG_RAW10_CFG1,
			IPG_BAYER_R_MASK, IPG_RAW10_CFG1_R);
		CSI_REG_MWR(idx, IPG_RAW10_CFG2,
			IPG_BAYER_B_MASK, IPG_RAW10_CFG2_B);
		CSI_REG_MWR(idx, IPG_RAW10_CFG2,
			IPG_BAYER_G_MASK, IPG_RAW10_CFG2_G);
		CSI_REG_MWR(idx, IPG_RAW10_CFG2,
			IPG_BAYER_R_MASK, IPG_RAW10_CFG2_R);

		CSI_REG_MWR(idx, IPG_RAW10_CFG3, IPG_BAYER_PATTERN_MASK,
						IPG_BAYER_PATTERN_BGGR);
		if (!IPG_IMAGE_MODE) {
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG0,
				0x00FF0000, IPG_YUV_CFG0_B);
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG0,
				0x0000FF00, IPG_YUV_CFG0_G);
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG0,
				0x000000FF, IPG_YUV_CFG0_R);

			CSI_REG_MWR(idx, IPG_YUV422_8_CFG1,
				0x00FF0000, IPG_YUV_CFG1_B);
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG1,
				0x0000FF00, IPG_YUV_CFG1_G);
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG1,
				0x000000FF, IPG_YUV_CFG1_R);

			CSI_REG_MWR(idx, IPG_YUV422_8_CFG2,
				0x00FF0000, IPG_YUV_CFG2_B);
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG2,
				0x0000FF00, IPG_YUV_CFG2_G);
			CSI_REG_MWR(idx, IPG_YUV422_8_CFG2,
				0x000000FF, IPG_YUV_CFG2_R);
		}

		CSI_REG_MWR(idx, IPG_OTHER_CFG0, IPG_V_BLANK_MASK, IPG_V_BLANK);
		CSI_REG_MWR(idx, IPG_OTHER_CFG0, IPG_H_BLANK_MASK, IPG_H_BLANK);

		CSI_REG_MWR(idx, MODE_CFG, IPG_ENABLE_MASK, IPG_ENABLE);
	} else
		CSI_REG_MWR(idx, MODE_CFG, IPG_ENABLE_MASK, ~IPG_ENABLE);

	pr_info("CSI IPG enable %d\n", enable);
}

void csi_reg_trace(unsigned int idx)
{
	unsigned long addr = 0;

	if (csi_dump_regbase[idx] == 0) {
		pr_info("CSI %d not used no need to dump\n", idx);
		return;
	}

	pr_info("CSI %d reg list\n", idx);
	for (addr = IP_REVISION; addr <= IPG_OTHER_CFG0; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			REG_RD(csi_dump_regbase[idx] + addr),
			REG_RD(csi_dump_regbase[idx] + addr + 4),
			REG_RD(csi_dump_regbase[idx] + addr + 8),
			REG_RD(csi_dump_regbase[idx] + addr + 12));
	}
}

/* phy testclear used to reset phy to right default state */
static void dphy_cfg_clr(int32_t idx)
{
	CSI_REG_MWR(idx, PHY_TEST_CRTL0, PHY_TESTCLR, 1);
	udelay(1);
	CSI_REG_MWR(idx, PHY_TEST_CRTL0, PHY_TESTCLR, 0);
	udelay(1);
}

static void csi_dphy_2p2_testclr_init(struct csi_phy_info *phy)
{
	unsigned int mask = 0;

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S_EN;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, mask);

	mask = BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_IF_SEL_S
		| BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_CSI_IF_SEL_M
		|BIT_ANLG_PHY_G1_DBG_SEL_ANALOG_MIPI_CSI_2P2LANE_DSI_IF_SEL_DB;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_REG_SEL_CFG_0,
		mask, mask);

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_IF_SEL_DB;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_DB,
		mask, ~mask);

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_IF_SEL_S;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_S,
		mask, ~mask);

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_IF_SEL_M;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_CTRL_M,
		mask, ~mask);

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M_SEL
		| BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S_SEL;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, mask);
}

static void csi_dphy_2p2_testclr_set(struct csi_phy_info *phy)
{
	unsigned int mask = 0;

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_TESTCLR_DB;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_TEST_DB,
		mask, mask);
	udelay(1);
	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M
		| BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, mask);
}

static void csi_dphy_2p2_testclr_clear(struct csi_phy_info *phy)
{
	unsigned int mask = 0;

	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_DSI_TESTCLR_DB;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_2P2L_TEST_DB,
		mask, ~mask);
	mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M
		| BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S;
	regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, ~mask);
}

static void csi_dphy_2p2_reset(struct csi_dt_node_info *csi_info)
{
	struct csi_phy_info *phy = NULL;
	uint32_t cphy_sel_mask;
	uint32_t cphy_sel_val;
	uint32_t mask = 0;

	if (!csi_info) {
		pr_err("fail to get valid dt_info ptr\n");
		return;
	}

	phy = &csi_info->phy;
	if (!phy) {
		pr_err("fail to get valid phy ptr\n");
		return;
	}

	switch (csi_info->controller_id) {
	case CSI_RX0:
	case CSI_RX1:
		cphy_sel_mask = 7 << (csi_info->controller_id * 4);
		cphy_sel_val  = 3 << csi_info->controller_id * 4;

		regmap_hwlock_update_bits(phy->cam_ahb_syscon,
				REG_MM_AHB_MIPI_CSI2_CTRL,
				cphy_sel_mask, cphy_sel_val);
		mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M
		| BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S;
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, mask);
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, ~mask);
		regmap_hwlock_update_bits(phy->cam_ahb_syscon,
				REG_MM_AHB_MIPI_CSI2_CTRL,
				cphy_sel_mask, 0x0);
		break;
	case CSI_RX2:
		cphy_sel_mask = 7 << (csi_info->controller_id * 4);
		cphy_sel_val  = 1 << csi_info->controller_id * 4;

		regmap_hwlock_update_bits(phy->cam_ahb_syscon,
				REG_MM_AHB_MIPI_CSI2_CTRL,
				cphy_sel_mask, cphy_sel_val);

		mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_S;
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, mask);
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, ~mask);
		regmap_hwlock_update_bits(phy->cam_ahb_syscon,
				REG_MM_AHB_MIPI_CSI2_CTRL,
				cphy_sel_mask, 0x0);
		cphy_sel_val  = 2 << csi_info->controller_id * 4;
		regmap_hwlock_update_bits(phy->cam_ahb_syscon,
				REG_MM_AHB_MIPI_CSI2_CTRL,
				cphy_sel_mask, cphy_sel_val);
		mask = BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_CSI_2P2L_TESTCLR_M;
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, mask);
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
		mask, ~mask);
		regmap_hwlock_update_bits(phy->cam_ahb_syscon,
				REG_MM_AHB_MIPI_CSI2_CTRL,
				cphy_sel_mask, 0x0);
		break;
	default:
		pr_err("fail to get valid csi_rx id\n");
	}
}

void csi_phy_power_down(struct csi_dt_node_info *csi_info,
			unsigned int sensor_id, int is_eb)
{

	uint32_t ps_pd_l = 0;
	uint32_t ps_pd_s = 0;
	uint32_t iso_sw = 0;
	uint32_t shutdownz = 0;
	uint32_t reg = 0;
	uint32_t dphy_eb = 0;
	struct csi_phy_info *phy = &csi_info->phy;

	if (!phy || !csi_info) {
		pr_err("fail to get valid phy ptr\n");
		return;
	}

	switch (csi_info->controller_id) {
	case CSI_RX0:
		shutdownz =
		BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_FORCE_CSI_PHY_SHUTDOWNZ;
		break;
	case CSI_RX1:
		shutdownz =
		BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_FORCE_CSI_S_PHY_SHUTDOWNZ;
		break;
	case CSI_RX2:
		shutdownz =
		BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2LANE_FORCE_CSI_PHY_SHUTDOWNZ;
		break;
	default:
		pr_err("fail to get valid csi_rx id\n");
	}
	reg = REG_AON_APB_PWR_CTRL;

	switch (phy->phy_id) {
	case PHY_2P2:
		/* 2p2lane phy as a 4lane phy  */
		ps_pd_l = BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L;
		ps_pd_s = BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S;
		iso_sw = BIT_AON_APB_MIPI_CSI_2P2LANE_ISO_SW_EN;

		regmap_update_bits(phy->anlg_phy_g1_syscon,
			REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CTRL_CSI_2P2L,
			BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_MODE_SEL,
			BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_MODE_SEL);
		break;
	case PHY_4LANE:
		/* phy: 4lane phy */
		ps_pd_l = BIT_AON_APB_MIPI_CSI_4LANE_PS_PD_L;
		ps_pd_s = BIT_AON_APB_MIPI_CSI_4LANE_PS_PD_S;
		iso_sw = BIT_AON_APB_MIPI_CSI_4LANE_ISO_SW_EN;
		break;
	case PHY_2LANE:
		/* phy: 2lane phy */
		ps_pd_l = BIT_AON_APB_MIPI_CSI_2LANE_PS_PD_L;
		ps_pd_s = BIT_AON_APB_MIPI_CSI_2LANE_PS_PD_S;
		iso_sw = BIT_AON_APB_MIPI_CSI_2LANE_ISO_SW_EN;
		break;
	case PHY_2P2_M:
	case PHY_2P2_S:
		/* 2p2lane phy as a 2lane phy  */
		ps_pd_l = BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_L;
		ps_pd_s = BIT_AON_APB_MIPI_CSI_2P2LANE_PS_PD_S;
		iso_sw = BIT_AON_APB_MIPI_CSI_2P2LANE_ISO_SW_EN;

		regmap_update_bits(phy->anlg_phy_g1_syscon,
			REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CTRL_CSI_2P2L,
			BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_MODE_SEL,
		    (int)~BIT_ANLG_PHY_G1_ANALOG_MIPI_CSI_2P2LANE_CSI_MODE_SEL);
		break;
	default:
		pr_err("fail to get valid phy id %d\n", phy->phy_id);
		return;
	}

	dphy_eb = BIT_AON_APB_SERDES_DPHY_EB;
	if (is_eb) {
		regmap_update_bits(phy->aon_apb_syscon,
				reg,
				ps_pd_l | ps_pd_s | iso_sw,
				ps_pd_l | ps_pd_s | iso_sw);
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
			shutdownz,
			~shutdownz);
		if (phy->phy_id == PHY_2P2 ||
			phy->phy_id == PHY_2P2_M ||
			phy->phy_id == PHY_2P2_S)
			regmap_update_bits(phy->aon_apb_syscon,
					REG_AON_APB_APB_EB1,
					dphy_eb, ~dphy_eb);
	} else {
		/* According to the time sequence of CSI-DPHY INIT,
		 * need pull down POWER, DPHY-reset and CSI-2 controller reset
		*/
		csi_shut_down_phy(1, sensor_id);
		csi_reset_shut_down(1, sensor_id);
		if (phy->phy_id == PHY_2P2 ||
			phy->phy_id == PHY_2P2_M ||
			phy->phy_id == PHY_2P2_S) {
			csi_dphy_2p2_testclr_init(phy);
			csi_dphy_2p2_testclr_set(phy);
		}
		else if (phy->phy_id == PHY_4LANE ||
				phy->phy_id == PHY_2LANE)
			CSI_REG_MWR(sensor_id, PHY_TEST_CRTL0, PHY_TESTCLR, 1);
		udelay(1);

		regmap_update_bits(phy->aon_apb_syscon,
				reg,
				ps_pd_s,
				~ps_pd_s);
		udelay(200);

		regmap_update_bits(phy->aon_apb_syscon,
				reg,
				ps_pd_l,
				~ps_pd_l);
		regmap_update_bits(phy->aon_apb_syscon,
				reg,
				iso_sw,
				~iso_sw);
		regmap_update_bits(phy->anlg_phy_g1_syscon,
		REG_ANLG_PHY_G1_ANALOG_MIPI_CSI_4LANE_MIPI_PHY_BIST_TEST,
				shutdownz,
				shutdownz);
		if (phy->phy_id == PHY_2P2 ||
			phy->phy_id == PHY_2P2_M ||
			phy->phy_id == PHY_2P2_S) {
			regmap_update_bits(phy->aon_apb_syscon,
					REG_AON_APB_APB_EB1,
					dphy_eb, dphy_eb);
			csi_dphy_2p2_testclr_clear(phy);
			csi_dphy_2p2_reset(csi_info);
		} else if (phy->phy_id == PHY_4LANE ||
				phy->phy_id == PHY_2LANE) {
			CSI_REG_MWR(sensor_id, PHY_TEST_CRTL0, PHY_TESTCLR, 0);
			udelay(1);
		}
		/* According to the time sequence of CSI-DPHY INIT,
		 * need pull up POWER, DPHY-reset and CSI-2 controller reset
		*/
		csi_shut_down_phy(0, sensor_id);
		csi_reset_shut_down(0, sensor_id);
	}
}

int csi_ahb_reset(struct csi_phy_info *phy, unsigned int csi_id)
{
	unsigned int flag = 0;

	if (!phy) {
		pr_err("fail to get valid phy ptr\n");
		return -EINVAL;
	}
	pr_info("%s csi, id %d dphy %d\n", __func__, csi_id, phy->phy_id);

	csi_dump_regbase[0] = 0;
	csi_dump_regbase[1] = 0;
	csi_dump_regbase[2] = 0;

	switch (csi_id) {
	case CSI_RX0:
		flag = BIT_MM_AHB_CSI_SOFT_RST;
		break;
	case CSI_RX1:
		flag = BIT_MM_AHB_CSI_S_SOFT_RST;
		break;
	case CSI_RX2:
		flag = BIT_MM_AHB_CSI_T_SOFT_RST;
		break;
	default:
		pr_err("fail to get valid csi id %d\n", csi_id);
	}
	regmap_update_bits(phy->cam_ahb_syscon,
			   REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(phy->cam_ahb_syscon,
			   REG_MM_AHB_AHB_RST, flag, ~flag);

	return 0;
}

void csi_controller_enable(struct csi_dt_node_info *dt_info, int32_t idx)
{
	struct csi_phy_info *phy = NULL;
	uint32_t cphy_sel_mask;
	uint32_t cphy_sel_val;
	uint32_t mask_eb = 0;
	uint32_t mask_rst = 0;

	if (!dt_info) {
		pr_err("fail to get valid dt_info ptr\n");
		return;
	}

	phy = &dt_info->phy;
	if (!phy) {
		pr_err("fail to get valid phy ptr\n");
		return;
	}

	pr_info("%s csi, id %d dphy %d\n", __func__, dt_info->controller_id,
		phy->phy_id);

	switch (dt_info->controller_id) {
	case CSI_RX0: {
		csi_dump_regbase[0] = dt_info->reg_base;
		mask_eb = BIT_MM_AHB_CSI_EB;
		mask_rst = BIT_MM_AHB_CSI_SOFT_RST;
		break;
	}
	case CSI_RX1: {
		csi_dump_regbase[1] = dt_info->reg_base;
		mask_eb = BIT_MM_AHB_CSI_S_EB;
		mask_rst = BIT_MM_AHB_CSI_S_SOFT_RST;
		break;
	}
	case CSI_RX2: {
		csi_dump_regbase[2] = dt_info->reg_base;
		mask_eb = BIT_MM_AHB_CSI_T_EB;
		mask_rst = BIT_MM_AHB_CSI_T_SOFT_RST;
		break;
	}
	default:
		pr_err("fail to get valid csi id\n");
		break;
	}

	regmap_update_bits(phy->cam_ahb_syscon, REG_MM_AHB_AHB_EB,
			mask_eb, mask_eb);
	regmap_update_bits(phy->cam_ahb_syscon, REG_MM_AHB_AHB_RST,
			mask_rst, mask_rst);
	udelay(1);
	regmap_update_bits(phy->cam_ahb_syscon, REG_MM_AHB_AHB_RST,
			mask_rst, ~mask_rst);

	if  (dt_info->controller_id == CSI_RX0 ||
			dt_info->controller_id == CSI_RX1) {
		cphy_sel_mask = 7 << (dt_info->controller_id * 4);

		switch (phy->phy_id) {
		case PHY_2P2: {
			cphy_sel_val = 3;
			break;
		}
		case PHY_4LANE: {
			cphy_sel_val = 2;
			break;
		}
		case PHY_2LANE: {
			cphy_sel_val = 4;
			break;
		}
		case PHY_2P2_S: {
			cphy_sel_val = 0;
			break;
		}
		case PHY_2P2_M: {
			cphy_sel_val = 1;
			break;
		}
		default:
			pr_err("fail to get valid csi phy id\n");
			return;
		}
		cphy_sel_val  <<= dt_info->controller_id * 4;
	} else {
		cphy_sel_mask = 7 << 8;

		switch (phy->phy_id) {
		case PHY_2LANE: {
			cphy_sel_val = 0;
			break;
		}
		case PHY_2P2_S: {
			cphy_sel_val = 1;
			break;
		}
		case PHY_2P2_M: {
			cphy_sel_val = 2;
			break;
		}
		default:
			pr_err("fail to get valid csi phy id\n");
			return;
		}
		cphy_sel_val  <<= 8;
	}
	regmap_hwlock_update_bits(phy->cam_ahb_syscon,
			REG_MM_AHB_MIPI_CSI2_CTRL,
			cphy_sel_mask,
			cphy_sel_val);
}

void dphy_init(struct csi_phy_info *phy, int32_t idx)
{
	if (!phy) {
		pr_err("fail to get valid phy ptr\n");
		return;
	}

	if (phy->phy_id == PHY_4LANE || phy->phy_id == PHY_2LANE)
		dphy_cfg_clr(idx);
}

void csi_set_on_lanes(uint8_t lanes, int32_t idx)
{
	CSI_REG_MWR(idx, LANE_NUMBER, 0x7, (lanes - 1));
}

void csi_reset_shut_down(uint8_t shutdown, int32_t idx)
{
	/* DPHY reset output, active low */
	CSI_REG_MWR(idx, RST_DPHY_N, BIT_0, shutdown ? 0 : 1);
	/* CSI-2 controller reset output, active low */
	CSI_REG_MWR(idx, RST_CSI2_N, BIT_0, shutdown ? 0 : 1);
}

/* PHY power down input, active low */
void csi_shut_down_phy(uint8_t shutdown, int32_t idx)
{
	CSI_REG_MWR(idx, PHY_PD_N, BIT_0, shutdown ? 0 : 1);
}

void csi_reset_controller(int32_t idx)
{
	CSI_REG_MWR(idx, RST_CSI2_N, BIT_0, 0);
	CSI_REG_MWR(idx, RST_CSI2_N, BIT_0, 1);
}

void csi_reset_phy(int32_t idx)
{
	CSI_REG_MWR(idx, RST_DPHY_N, BIT_0, 0);
	CSI_REG_MWR(idx, RST_DPHY_N, BIT_0, 1);
}

void csi_event_enable(int32_t idx)
{
	CSI_REG_WR(idx, MASK0, CSI_MASK0);
	CSI_REG_WR(idx, MASK1, CSI_MASK1);
}

void csi_close(int32_t idx)
{
	csi_shut_down_phy(1, idx);
	csi_reset_controller(idx);
	csi_reset_phy(idx);
}
