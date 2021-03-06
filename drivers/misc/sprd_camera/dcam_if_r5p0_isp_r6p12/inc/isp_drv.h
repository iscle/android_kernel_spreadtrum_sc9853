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

#ifndef _ISP_DRV_HEADER_
#define _ISP_DRV_HEADER_

#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/sprd_img.h>
#include <linux/sprd_ion.h>
#include <video/sprd_isp_r6p12.h>

#include "isp_reg.h"
#include "cam_common.h"
#include "cam_queue.h"
#include "isp_cfg.h"
#include "dcam_drv.h"


#define ISP_PIXEL_ALIGN_WIDTH          4
#define ISP_PIXEL_ALIGN_HEIGHT         2
#define ISP_FRGB_GAMMA_BUF_SIZE        (257 * 4 + 4)
#define ISP_ZSL_BUF_NUM                2
#define ISP_HDR_NUM                    3
#define ISP_3DNR_NUM                   5

#define ISP_LOWEST_ADDR                0x800
#define ISP_ADDR_INVALID(addr)         \
		((unsigned long)(addr) < (unsigned long)ISP_LOWEST_ADDR)


#define ISP_SC_COEFF_BUF_COUNT         10
#define ISP_SC_COEFF_BUF_SIZE          (24 << 10)
#define ISP_SC_COEFF_COEF_SIZE         (1 << 10)
#define ISP_SC_COEFF_TMP_SIZE          (21 << 10)
#define ISP_PATH_SCL_COUNT             5

#define ISP_SC_H_COEF_SIZE             0xC0
#define ISP_SC_V_COEF_SIZE             0x210
#define ISP_SC_V_CHROM_COEF_SIZE       0x210

#define ISP_SC_COEFF_H_NUM             (ISP_SC_H_COEF_SIZE / 6)
#define ISP_SC_COEFF_H_CHROMA_NUM      (ISP_SC_H_COEF_SIZE / 12)
#define ISP_SC_COEFF_V_NUM             (ISP_SC_V_COEF_SIZE / 4)
#define ISP_SC_COEFF_V_CHROMA_NUM      (ISP_SC_V_CHROM_COEF_SIZE / 4)

/*should be replaced by the irq id*/
#define ISP_IRQ_ERR_MASK0              ((1 << ISP_INT_FMCU_CMD_ERROR))

#define ISP_IRQ_ERR_MASK1              ((1 << ISP_INT_FMCU_CMD_ERROR))

#define ISP_IRQ_ERR_MASK2              ((1 << ISP_INT_FMCU_CMD_ERROR))

#define ISP_INT_LINE_MASK0 \
	((1 << ISP_INT_SHADOW_DONE) | \
	(1 << ISP_INT_ISP_ALL_DONE) | \
	(1 << ISP_INT_STORE_DONE_PRE) | \
	(1 << ISP_INT_DISPATCH_DONE))

#define ISP_INT_LINE_MASK1 \
	((1 << ISP_INT_STORE_DONE_OUT) | \
	(1 << ISP_INT_STORE_DONE_VID) | \
	(1 << ISP_INT_STORE_DONE_PRE))

#define ISP_INT_LINE_MASK2 \
	((1 << ISP_INT_FMCU_LOAD_DONE) | \
	(1 << ISP_INT_FMCU_CONFIG_DONE) | \
	(1 << ISP_INT_FMCU_SHADOW_DONE))

#define ISP_AXI_ITI2AXIM_ISP_ROSTD_MASK	0xFF00
#define ISP_AXI_ARBITER_WQOS_MASK	0x37FF
#define ISP_AXI_ARBITER_RQOS_MASK	0x1FF

enum {
	ISP_ST_STOP = 0,
	ISP_ST_START,
};

/* isp CFG mode is default */
enum isp_work_mode {
	ISP_CFG_MODE,
	ISP_AP_MODE,
	ISP_WM_MAX
};

enum isp_glb_reg_id {
	ISP_AXI_REG = 0,
	ISP_INIT_MASK_REG,
	ISP_INIT_CLR_REG,
	ISP_REG_MAX
};

enum isp_drv_rtn {
	ISP_RTN_SUCCESS = 0,
	ISP_RTN_PARA_ERR = 0x10,
	ISP_RTN_FRM_DECI_ERR,
	ISP_RTN_OUT_FMT_ERR,
	ISP_RTN_PATH_ADDR_ERR,
	ISP_RTN_PATH_FRAME_LOCKED,
	ISP_RTN_PATH_SC_ERR,
	ISP_RTN_PATH_IN_SIZE_ERR,
	ISP_RTN_PATH_TRIM_SIZE_ERR,
	ISP_RTN_PATH_OUT_SIZE_ERR,
	ISP_RTN_PATH_ENDIAN_ERR,
	ISP_RTN_IRQ_NUM_ERR,
	ISP_RTN_TIME_OUT,
	ISP_RTN_MAX
};

enum isp_path_mode {
	ISP_PRE_OFFLINE,
	ISP_VID_OFFLINE,
	ISP_CAP_OFFLINE,
	ISP_MODE_MAX,
};

enum isp_id {
	ISP_ID_0 = 0,
	ISP_ID_1,
	ISP_ID_MAX,
};

/* definition of scene id for each isp instance */
enum isp_scene_id {
	ISP_SCENE_PRE,
	ISP_SCENE_CAP,
	ISP_SCENE_MAX,
};

enum isp_scl_id {
	ISP_SCL_PRE = 0,
	ISP_SCL_VID,
	ISP_SCL_CAP,
	ISP_SCL_MAX,
};

enum isp_path_id {
	ISP_PATH_PRE = 0,
	ISP_PATH_VID,
	ISP_PATH_CAP,
	ISP_PATH_MAX
};

enum isp_path_index {
	ISP_PATH_IDX_0 = 0,
	ISP_PATH_IDX_PRE = 1,
	ISP_PATH_IDX_VID = 2,
	ISP_PATH_IDX_CAP = 4,
	ISP_PATH_IDX_ALL = 0x07,
};

enum isp_config_param {
	ISP_PATH_INPUT_SIZE,
	ISP_PATH_INPUT_RECT,
	ISP_PATH_OUTPUT_SIZE,
	ISP_PATH_OUTPUT_ADDR,
	ISP_PATH_OUTPUT_RESERVED_ADDR,
	ISP_PATH_STORE_CCE_ADDR,
	ISP_PATH_OUTPUT_FORMAT,
	ISP_PATH_FRM_DECI,
	ISP_PATH_WORK_MODE,
	ISP_PATH_ENABLE,
	ISP_PATH_SHRINK,
	ISP_PATH_DATA_ENDIAN,
	ISP_PATH_MODE,
	ISP_PATH_SKIP_NUM,
	ISP_NR3_ENABLE,
	ISP_NR3_ME_CONV_SIZE,
	ISP_SNS_MAX_SIZE,
	ISP_DUAL_CAM_EN,
	ISP_CFG_MAX
};

enum isp_irq_id {
	ISP_PATH_PRE_DONE,
	ISP_PATH_VID_DONE,
	ISP_PATH_CAP_DONE,
	ISP_SHADOW_DONE,
	ISP_DCAM_SOF,
	ISP_3DNR_CAP_DONE,
	ISP_IMG_MAX
};

enum isp_fetch_format {
	ISP_FETCH_YUV422_3FRAME = 0,
	ISP_FETCH_YUYV,
	ISP_FETCH_UYVY,
	ISP_FETCH_YVYU,
	ISP_FETCH_VYUY,
	ISP_FETCH_YUV422_2FRAME,
	ISP_FETCH_YVU422_2FRAME,
	ISP_FETCH_YUV420_2FRAME,
	ISP_FETCH_YVU420_2FRAME,
	ISP_FETCH_NORMAL_RAW10,
	ISP_FETCH_CSI2_RAW10,/* MIPI RAW */
	ISP_FETCH_FORMAT_MAX
};

enum isp_fmcu_cmd {
	PRE0_SHADOW_DONE = 0x10,
	PRE0_ALL_DONE,
	PRE0_LENS_LOAD_DONE,
	CAP0_SHADOW_DONE,
	CAP0_ALL_DONE,
	CAP0_LENS_LOAD_DONE,
	PRE1_SHADOW_DONE,
	PRE1_ALL_DONE,
	PRE1_LENS_LOAD_DONE,
	CAP1_SHADOW_DONE,
	CAP1_ALL_DONE,
	CAP1_LENS_LOAD_DONE,
	CFG_TRIGGER_PULSE,
	SW_TRIGGER,
};

/*ISP_CLK_MAX_INDEX: Sharkl2 468M, Sharklj1 576M*/
enum {
	ISP_CLK_128M_INDEX = 0,
	ISP_CLK_256M_INDEX,
	ISP_CLK_307M2_INDEX,
	ISP_CLK_384M_INDEX,
	ISP_CLK_MAX_INDEX
};

enum isp_status {
	ISP_START = 0,
	ISP_STOP,
	ISP_STATUS_MAX,
};

struct isp_group {
	uint32_t dual_cam;
	uint32_t dual_cap_sts; /*every cap frame status*/
	uint32_t dual_fullpath_stop;
	struct completion first_wait_com;
	uint32_t first_need_wait; /*first frame in progress*/
	uint32_t wait_isp_id;
	uint32_t dual_cap_cnt;
	struct sprd_img_capture_param capture_param;
	uint32_t dual_cap_total;
	void *isp_dev[ISP_MAX_COUNT];
};

struct isp_ch_irq {
	int irq0;
	int irq1;
};

struct isp_clk_tag {
	char *clock;
	char *clk_name;
};

struct isp_deci_info {
	uint32_t deci_y_eb;
	uint32_t deci_y;
	uint32_t deci_x_eb;
	uint32_t deci_x;
};

struct isp_trim_info {
	uint32_t start_x;
	uint32_t start_y;
	uint32_t size_x;
	uint32_t size_y;
};

struct isp_endian_sel {
	unsigned char y_endian;
	unsigned char uv_endian;
};

struct isp_sc_tap {
	uint32_t y_tap;
	uint32_t uv_tap;
};

struct isp_regular_info {
	uint32_t regular_mode;
	uint32_t shrink_uv_dn_th;
	uint32_t shrink_uv_up_th;
	uint32_t shrink_y_dn_th;
	uint32_t shrink_y_up_th;
	uint32_t effect_v_th;
	uint32_t effect_u_th;
	uint32_t effect_y_th;
	uint32_t shrink_c_range;
	uint32_t shrink_c_offset;
	uint32_t shrink_y_range;
	uint32_t shrink_y_offset;
};

struct isp_scaler_info {
	uint32_t scaler_bypass;
	uint32_t scaler_y_ver_tap;
	uint32_t scaler_uv_ver_tap;
	uint32_t scaler_ip_int;
	uint32_t scaler_ip_rmd;
	uint32_t scaler_cip_int;
	uint32_t scaler_cip_rmd;
	uint32_t scaler_factor_in;
	uint32_t scaler_factor_out;
	uint32_t scaler_ver_ip_int;
	uint32_t scaler_ver_ip_rmd;
	uint32_t scaler_ver_cip_int;
	uint32_t scaler_ver_cip_rmd;
	uint32_t scaler_ver_factor_in;
	uint32_t scaler_ver_factor_out;
	uint32_t scaler_in_width;
	uint32_t scaler_in_height;
	uint32_t scaler_out_width;
	uint32_t scaler_out_height;
};

struct isp_k_block {
	uint32_t lsc_load_buf_id_prv;
	uint32_t lsc_load_buf_id_cap;
	uint32_t lsc_update_buf_id;
	uint32_t full_gamma_buf_id_prv;
	uint32_t full_gamma_buf_id_cap;
	uint32_t yuv_ygamma_buf_id;
	unsigned long full_gamma_buf_addr;
	uint32_t lsc_buf_phys_addr;
	uint32_t anti_flicker_buf_phys_addr;
	uint32_t raw_nlm_buf_id_prv;
	uint32_t raw_nlm_buf_id_cap;
	uint32_t lsc_1d_buf_id;
	uint32_t hsv_buf_id_prv;
	uint32_t hsv_buf_id_cap;
	uint32_t lsc_2d_weight_en;
	uint32_t fetch_raw_phys_addr;
	uint32_t lsc_updated;
	unsigned long fetch_mfd;
	unsigned long lsc_mfd;
	struct cam_buf_info fetch_pfinfo;
	struct cam_buf_info lsc_pfinfo;
	unsigned int param_update_flag;
	uint32_t isp_status;
};

struct isp_store_info {
	uint32_t bypass;
	uint32_t endian;
	uint32_t speed_2x;
	uint32_t mirror_en;
	uint32_t color_format;
	uint32_t max_len_sel;
	uint32_t shadow_clr;
	uint32_t store_res;
	uint32_t rd_ctrl;
	uint32_t shadow_clr_sel;
	struct camera_size size;
	struct store_border border;
	struct isp_pitch_fs pitch;
};

struct isp_zoom_param {
	struct camera_size in_size;
	struct camera_rect in_rect;
	struct camera_size out_size;
};

struct isp_coeff {
	struct isp_zoom_param param;
	uint32_t coeff_buf[ISP_SC_COEFF_BUF_SIZE];
};

struct isp_path_desc {
	uint32_t valid;
	uint32_t uv_sync_v;
	uint32_t scaler_bypass;
	uint32_t status;
	uint32_t path_mode;
	uint32_t frm_deci;
	uint32_t input_format;
	uint32_t output_format;
	uint32_t odata_mode;
	uint32_t frame_base_id;
	uint32_t output_frame_count;
	uint32_t path_sel;
	uint32_t frm_cnt;
	uint32_t skip_num;
	uint32_t buf_cnt;
	struct camera_size src;
	struct camera_size dst;
	struct isp_endian_sel data_endian;
	struct isp_deci_info deci_info;
	struct isp_trim_info trim0_info;
	struct isp_trim_info trim1_info;
	struct isp_regular_info regular_info;
	struct isp_scaler_info scaler_info;
	struct isp_store_info store_info;
	struct camera_frame path_reserved_frame;
	struct isp_coeff coeff_latest;
	struct cam_buf_queue coeff_queue;
	struct cam_buf_queue buf_queue;
	struct cam_frm_queue frame_queue;
	struct isp_nr3_param nr3_param;
};

struct isp_fmcu_slice_desc {
	void *slice_handle;
	uint32_t fmcu_num;
	uint32_t capture_state;
	struct cam_buf_info buf_info;
};

struct isp_sc_coeff {
	uint32_t buf[ISP_SC_COEFF_BUF_SIZE];
	uint32_t flag;
	struct isp_path_desc path;
};

struct isp_sc_coeff_queue {
	struct isp_sc_coeff coeff[ISP_SC_COEFF_BUF_COUNT];
	struct isp_sc_coeff *write;
	struct isp_sc_coeff *read;
	int w_index;
	int r_index;
	spinlock_t lock;
};

struct isp_sc_array {
	struct isp_sc_coeff_queue *pre_queue;
	struct isp_sc_coeff_queue *vid_queue;
	struct isp_sc_coeff_queue *cap_queue;
	struct isp_sc_coeff coeff[ISP_SCL_MAX];
};

struct isp_module {
	uint32_t com_idx;
	struct completion scale_coeff_mem_com;
	struct isp_cfg_ctx_desc isp_cfg_contex;
	struct isp_path_desc isp_path[ISP_SCL_MAX];
	struct camera_frame path_reserved_frame[ISP_SCL_MAX];
	struct cam_buf_queue bin_buf_queue;
	struct cam_frm_queue bin_frm_queue;
	struct cam_buf_queue full_buf_queue;
	struct cam_frm_queue full_frm_queue;
	struct cam_frm_queue full_zsl_queue;
	uint32_t isp_state;
	uint32_t frm_cnt;
};

struct isp_pipe_dev {
	uint32_t com_idx;
	uint32_t scene_id;
	uint32_t work_mode;
	uint32_t isp_chn_id[2];
	uint32_t is_raw_capture;
	uint32_t pdaf_status[2];
	uint32_t cap_flag;
	uint32_t clr_queue;
	uint32_t cap_cur_cnt;
	uint32_t frame_id;
	struct mutex isp_mutex;
	struct mutex offline_thread_mutex;
	struct completion fmcu_com;
	struct completion irq_com;
	struct completion isr_done_lock;
	struct completion offline_thread_com;
	struct task_struct *offline_thread;
	struct isp_k_block isp_k_param;
	struct isp_fmcu_slice_desc fmcu_slice;
	struct isp_module module_info;
	struct camera_frame capture_frame;
	struct camera_frame preview_frame;
	void *isp_handle_addr;
	uint32_t is_offline_thread_stop;
	uint32_t is_wait_fmcu;
	uint32_t isp_busy;
	uint32_t isp_offline_state;
	uint32_t dcam_full_path_stop;
	uint32_t isp_offline_thread_flag;
	uint32_t offline_proc_cap;
};

typedef void(*isp_isr)(enum isp_id id, void *param);
typedef int(*isp_isr_func)(struct camera_frame *frame, void *param);

int sprd_isp_stop(void *isp_handle, int is_irq);
int sprd_isp_offline_proc(void *handle, uint32_t is_irq);
int sprd_isp_start_fmcu(void *handle,
			struct sprd_img_capture_param param, uint32_t is_irq,
			enum isp_path_index path_index);
int sprd_isp_fmcu_slice_stop(void *handle);
int sprd_isp_start(void *isp_handle);
int sprd_isp_start_path(void *isp_handle, enum isp_path_index path_index,
	struct camera_frame *frame);
int isp_raw_cap_proc(void *isp_handle, struct isp_raw_proc_info *raw_cap);
int sprd_isp_update_zoom_param(void *isp_handle,
			enum isp_path_index path_index,
			struct camera_size *in_size,
			struct camera_rect *in_rect,
			struct camera_size *out_size);
int set_isp_path_cfg(void *isp_handle, enum isp_path_index path_index,
	enum isp_config_param id, void *param);
int sprd_isp_module_en(void *isp_handle, enum isp_id id);
int sprd_isp_module_dis(void *isp_handle, enum isp_id id);
int sprd_isp_dev_init(void **isp_pipe_dev_handle, enum isp_id id);
int sprd_isp_dev_deinit(void *isp_dev_handle, enum isp_id id);
int sprd_isp_drv_init(struct platform_device *pdev);
void sprd_isp_drv_deinit(void);
int sprd_isp_parse_dt(struct device_node *dn, uint32_t *isp_count);
int sprd_isp_reg_isr(enum isp_id id, enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data);
int isp_cfg_param(void *param,
	struct isp_k_block *isp_k_param, void *isp_handle);
void isp_reg_trace(enum isp_id idx);
int32_t isp_capability(void *param);
int sprd_isp_update_clk(uint32_t clk_index, struct device_node *dn);
int isp_get_status(void *isp_handle);
int isp_set_status(void *isp_handle);
uint32_t sprd_isp_get_dual_cam_mode(void);

#endif
