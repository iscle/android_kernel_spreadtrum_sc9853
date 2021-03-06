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

#ifndef _DCAM_DRV_H_
#define _DCAM_DRV_H_

#include <video/sprd_img.h>

#include "dcam_reg.h"
#include "cam_common.h"
#include "dcam_int.h"
#include "cam_buf.h"
#include "cam_queue.h"
#include "cam_statistic.h"
#include "isp_3dnr_drv.h"

#define DCAM_LOWEST_ADDR               0x800
#define DCAM_ADDR_INVALID(addr) \
	((unsigned long)(addr) < DCAM_LOWEST_ADDR)
#define DCAM_YUV_ADDR_INVALID(y, u, v) \
	(DCAM_ADDR_INVALID(y) && \
	DCAM_ADDR_INVALID(u) && \
	DCAM_ADDR_INVALID(v))


#define DCAM_STATE_QUICKQUIT           0x01
#define DCAM2_SOFT_RST                 BIT(4)
#define DCAM_AXIM_AQOS_MASK		(0x30FFFF)
#define BPC_CLK_EB_LEP			BIT(7)

#define REG_RD(a) readl_relaxed((IO_PTR)(a))

enum chip_id {
	SHARKL3 = 0,
	SHARKLEP
};

enum dcam_drv_rtn {
	DCAM_RTN_SUCCESS = 0,
	DCAM_RTN_PARA_ERR = 0x10,
	DCAM_RTN_IO_ID_ERR,
	DCAM_RTN_ISR_ID_ERR,
	DCAM_RTN_MODE_ERR,
	DCAM_RTN_TIMEOUT,

	DCAM_RTN_CAP_IN_BITS_ERR = 0x20,
	DCAM_RTN_CAP_IN_PATTERN_ERR,
	DCAM_RTN_CAP_SKIP_FRAME_ERR,
	DCAM_RTN_CAP_FRAME_DECI_ERR,
	DCAM_RTN_CAP_XY_DECI_ERR,
	DCAM_RTN_CAP_FRAME_SIZE_ERR,
	DCAM_RTN_CAP_SENSOR_MODE_ERR,
	DCAM_RTN_CAP_IF_MODE_ERR,

	DCAM_RTN_PATH_ADDR_ERR = 0x30,
	DCAM_RTN_PATH_FRAME_LOCKED,
	DCAM_RTN_PATH_GEN_COEFF_ERR,
	DCAM_RTN_PATH_ENDIAN_ERR,
	DCAM_RTN_PATH_OUT_SIZE_ERR,
	DCAM_RTN_PATH_FRM_DECI_ERR,
	DCAM_RTN_MAX
};

enum dcam_cfg_id {
	DCAM_CAP_INTERFACE = 0,
	DCAM_CAP_SENSOR_MODE,
	DCAM_CAP_SYNC_POL,
	DCAM_CAP_DATA_BITS,
	DCAM_CAP_DATA_PATTERN,
	DCAM_CAP_DATA_PACKET,
	DCAM_CAP_PRE_SKIP_CNT,
	DCAM_CAP_FRM_DECI,
	DCAM_CAP_FRM_COUNT_CLR,
	DCAM_CAP_FRM_COUNT_GET,
	DCAM_CAP_INPUT_RECT,
	DCAM_CAP_IMAGE_XY_DECI,
	DCAM_CAP_TO_ISP,
	DCAM_CAP_SAMPLE_MODE,
	DCAM_CAP_SBS_MODE,
	DCAM_CAP_4IN1_BYPASS,

	DCAM_PATH_INPUT_SIZE,
	DCAM_PATH_INPUT_RECT,
	DCAM_PATH_INPUT_ADDR,
	DCAM_PATH_OUTPUT_SIZE,
	DCAM_PATH_OUTPUT_FORMAT,
	DCAM_PATH_OUTPUT_LOOSE,
	DCAM_PATH_OUTPUT_ADDR,
	DCAM_PATH_OUTPUT_RESERVED_ADDR,
	DCAM_PATH_FRAME_BASE_ID,
	DCAM_PATH_DATA_ENDIAN,
	DCAM_PATH_ENABLE,
	DCAM_PATH_FRM_DECI,
	DCAM_PATH_SRC_SEL,
	DCAM_PDAF_CTRL,

	DCAM_FETCH_DATA_PACKET,
	DCAM_FETCH_DATA_ENDIAN,
	DCAM_FETCH_INPUT_RECT,
	DCAM_FETCH_INPUT_ADDR,
	DCAM_FETCH_START,
	DCAM_CFG_ID_E_MAX
};

enum dcam_data_endian {
	DCAM_ENDIAN_LITTLE = 0,
	DCAM_ENDIAN_BIG,
	DCAM_ENDIAN_HALFBIG,
	DCAM_ENDIAN_HALFLITTLE,
	DCAM_ENDIAN_MAX
};

enum dcam_glb_reg_id {
	DCAM_CFG_REG = 0,
	DCAM_CONTROL_REG,
	DCAM_INIT_MASK_REG,
	DCAM_INIT_CLR_REG,
	DCAM_AHBM_STS_REG,
	DCAM_ENDIAN_REG,
	DCAM_AXIM_REG,
	DCAM_REG_MAX
};

enum {
	DCAM_CLK_128M_INDEX = 0,
	DCAM_CLK_256M_INDEX,
	DCAM_CLK_307M2_INDEX,
	DCAM_CLK_384M_INDEX
};

enum path_status {
	DCAM_ST_STOP = 0,
	DCAM_ST_START,
	DCAM_ST_PAUSE,
};

struct dcam_cap_sync_pol {
	unsigned char vsync_pol;
	unsigned char hsync_pol;
	unsigned char pclk_pol;
	unsigned char need_href;
	unsigned char pclk_src;
	unsigned char reserved[3];
};

struct dcam_cap_dec {
	unsigned char x_factor;
	unsigned char y_factor;
	unsigned char x_mode;
	unsigned char reserved;
};

struct dcam_cap_desc {
	enum dcam_cap_if_mode cam_if;
	enum dcam_cap_sensor_mode input_format;
	enum dcam_capture_mode cap_mode;
	struct camera_rect cap_rect;
};

struct dcam_fetch_desc {
	int is_loose;
	struct camera_rect input_rect;
	struct camera_frame frame;
};

struct dcam_path_valid {
	uint32_t input_rect:1;
	uint32_t output_format:1;
	uint32_t src_sel:1;
	uint32_t data_endian:1;
	uint32_t output_size:1;
	uint32_t frame_deci:1;
	uint32_t v_deci:1;
	uint32_t pdaf_ctrl:1;
};

struct dcam_path_desc {
	enum camera_path_id id;
	struct camera_size input_size;
	struct camera_rect input_rect;
	struct camera_size sc_input_size;
	struct camera_size output_size;
	struct camera_frame ion_buffer[DCAM_FRM_QUEUE_LENGTH];
	struct cam_buf_queue buf_queue;/*todo link*/
	struct cam_frm_queue frame_queue;/*done link*/
	struct camera_frame reserved_frame;
	struct camera_endian_sel data_endian;
	struct dcam_path_valid valid_param;
	uint32_t frame_base_id;
	uint32_t output_frame_count;
	uint32_t output_format;
	uint32_t is_loose;
	uint32_t src_sel;
	uint32_t frame_deci;
	uint32_t valid;
	enum path_status status;
	struct completion tx_done_com;
	struct completion sof_com;
	uint32_t wait_for_done;
	uint32_t is_update;
	uint32_t wait_for_sof;
	uint32_t need_stop;
	uint32_t need_wait;
	uint32_t ion_buf_cnt;
	int sof_cnt;
	int done_cnt;
	void *private_data;
};

struct dcam_3dnr_me {
	int mv_x;
	int mv_y;
	uint32_t mv_ready_cnt;
	uint32_t bin_frame_cnt;
	uint32_t full_frame_cnt;
	struct camera_frame bin_frame;
	struct camera_frame full_frame;
};

struct dcam_fast_me_param {
	uint32_t nr3_channel_sel;
	uint32_t nr3_project_mode;
	uint32_t nr3_ping_pang_en;
	uint32_t nr3_bypass;
	uint32_t roi_start_x;
	uint32_t roi_start_y;
	uint32_t roi_width;
	uint32_t roi_height;
	uint32_t cap_in_size_w;
	uint32_t cap_in_size_h;
};

struct dcam_module {
	enum dcam_id id;
	struct dcam_cap_desc dcam_cap;
	struct dcam_fetch_desc dcam_fetch;
	struct dcam_path_desc full_path;
	struct dcam_path_desc bin_path;
	struct cam_statis_module statis_module_info;
	struct dcam_3dnr_me fast_me;
	struct dcam_fast_me_param me_param;
	uint32_t need_nr3;
	uint32_t err_happened;
	uint32_t state;
	uint32_t frame_id;
};

struct flash_led_task {
	struct sprd_img_set_flash set_flash;
	uint32_t frame_skipped;
	uint32_t after_af;
	struct timeval timestamp;
	uint32_t skip_number;/*cap skip*/
	enum dcam_id dcam_idx;
	struct completion flash_thread_com;
	struct task_struct *flash_thread;
	uint32_t is_flash_thread_stop;
};

int sprd_dcam_module_init(enum dcam_id idx);
int sprd_dcam_module_deinit(enum dcam_id idx);
int sprd_dcam_module_en(enum dcam_id idx);
int sprd_dcam_module_dis(enum dcam_id idx);
int sprd_dcam_start(enum dcam_id idx);
int sprd_dcam_stop(enum dcam_id idx, int is_irq);
int sprd_dcam_reset(enum dcam_id idx, int is_irq);
int sprd_dcam_path_pause(enum dcam_id idx, uint32_t channel_id);
int sprd_dcam_path_resume(enum dcam_id idx, uint32_t channel_id);
int sprd_dcam_drv_init(struct platform_device *p_dev);
void sprd_dcam_drv_deinit(void);
int sprd_camera_get_path_id(struct camera_get_path_id *path_id,
	uint32_t *channel_id, uint32_t scene_mode);
int sprd_dcam_get_path_capability(struct cam_path_capability *capacity);
int sprd_dcam_parse_dt(struct device_node *dn, uint32_t *dcam_count);
int sprd_dcam_update_clk(uint32_t clk_index, struct device_node *dn);
void sprd_dcam_glb_reg_awr(enum dcam_id idx, unsigned long addr,
			uint32_t val, uint32_t reg_id);
void sprd_dcam_glb_reg_owr(enum dcam_id idx, unsigned long addr,
			uint32_t val, uint32_t reg_id);
void sprd_dcam_glb_reg_mwr(enum dcam_id idx, unsigned long addr,
			uint32_t mask, uint32_t val,
			uint32_t reg_id);
void dcam_force_copy(enum dcam_id idx, enum camera_copy_id copy_id);
void dcam_auto_copy(enum dcam_id idx, enum camera_copy_id copy_id);
struct dcam_module *get_dcam_module(enum dcam_id idx);

struct dcam_cap_desc *get_dcam_cap(enum dcam_id idx);
int set_dcam_cap_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);

struct dcam_fetch_desc *get_dcam_fetch(enum dcam_id idx);
int set_dcam_fetch_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);

struct dcam_path_desc *get_dcam_full_path(enum dcam_id idx);
int dcam_full_path_init(enum dcam_id idx);
int dcam_full_path_deinit(enum dcam_id idx);
int set_dcam_full_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int dcam_start_full_path(enum dcam_id idx);
void dcam_quickstop_full_path(enum dcam_id idx);
void dcam_full_path_sof(enum dcam_id idx);
void dcam_full_path_done(enum dcam_id idx, enum dcam_irq_id irq_id,
	void *param);
int dcam_full_path_set_next_frm(enum dcam_id idx);
int dcam_full_path_clear(enum dcam_id idx);

struct dcam_path_desc *get_dcam_bin_path(enum dcam_id idx);
int dcam_bin_path_init(enum dcam_id idx);
int dcam_bin_path_deinit(enum dcam_id idx);
int set_dcam_bin_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int dcam_start_bin_path(enum dcam_id idx);
void dcam_quickstop_bin_path(enum dcam_id idx);
void dcam_bin_path_sof(enum dcam_id idx);
void dcam_bin_path_done(enum dcam_id idx, enum dcam_irq_id irq_id,
	void *param);
int dcam_bin_path_set_next_frm(enum dcam_id idx);
int gen_rawsizer_coeff(uint16_t src_width, uint16_t src_height,
		uint16_t dst_width, uint16_t dst_height, uint32_t *coeff_buf);

int dcam_set_flash(enum dcam_id idx,
			struct sprd_img_set_flash *set_flash);
int dcam_start_flash(struct camera_frame *frame, void *param);
int dcam_create_flash_task(struct flash_led_task **task, enum dcam_id cam_idx);
int dcam_destroy_flash_task(struct flash_led_task *task);

int sprd_dcam_set_3dnr_me(uint32_t idx, void *size);
void sprd_dcam_cfg_fast_me(uint32_t idx, void *data);
int sprd_dcam_fast_me_info(enum dcam_id idx,
	uint32_t need_nr3, struct camera_size *size);
struct dcam_fast_me_param *get_dcam_me_param(enum dcam_id idx);

int sprd_dcam_get_chip_id(void);
void dcam_reg_trace(enum dcam_id idx);

#endif /* _DCAM_DRV_H_ */
