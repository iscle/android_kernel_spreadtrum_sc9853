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

#include <video/sprd_mm.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "dcam_drv.h"
#include "flash_drv.h"

int dcam_set_flash(enum dcam_id idx,
			struct sprd_img_set_flash *set_flash)
{
	sprd_flash_ctrl(set_flash);
	DCAM_TRACE("%d set flash\n", idx);

	return 0;
}

int dcam_start_flash(struct camera_frame *frame, void *param)
{
	uint32_t need_light = 1;
	uint32_t led0_ctrl = 0, led1_ctrl = 0;
	uint32_t led0_status = 0, led1_status = 0;
	struct flash_led_task *falsh_task = (struct flash_led_task *)param;

	if (falsh_task == NULL) {
		DCAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}

	led0_ctrl = falsh_task->set_flash.led0_ctrl;
	led1_ctrl = falsh_task->set_flash.led1_ctrl;
	led0_status = falsh_task->set_flash.led0_status;
	led1_status = falsh_task->set_flash.led1_status;

	if ((led0_ctrl && led0_status < FLASH_STATUS_MAX) ||
		(led1_ctrl && led1_status < FLASH_STATUS_MAX)) {
		if ((led0_ctrl && FLASH_HIGH_LIGHT == led0_status) ||
			(led1_ctrl && FLASH_HIGH_LIGHT == led1_status)) {
			falsh_task->frame_skipped++;
			if (falsh_task->frame_skipped >=
				falsh_task->skip_number) {
				/* flash lighted at the last SOF before
				* the right capture frame
				*/
				DCAM_TRACE("waiting finished\n");
			} else {
				need_light = 0;
				DCAM_TRACE("wait for the next SOF, %d %d\n",
					falsh_task->frame_skipped,
					falsh_task->skip_number);
			}
		}
		if (need_light)
			complete(&falsh_task->flash_thread_com);
	}

	return 0;
}

static int __img_opt_flash(struct camera_frame *frame, void *param)
{
	uint32_t led0_ctrl = 0, led1_ctrl = 0;
	uint32_t led0_status = 0, led1_status = 0;
	struct flash_led_task *falsh_task = (struct flash_led_task *)param;

	if (falsh_task == NULL) {
		DCAM_TRACE("fail to get valid input ptr\n");
		return 0;
	}

	led0_ctrl = falsh_task->set_flash.led0_ctrl;
	led1_ctrl = falsh_task->set_flash.led1_ctrl;
	led0_status = falsh_task->set_flash.led0_status;
	led1_status = falsh_task->set_flash.led1_status;

	if ((led0_ctrl && led0_status < FLASH_STATUS_MAX) ||
		(led1_ctrl && led1_status < FLASH_STATUS_MAX)) {
		DCAM_TRACE("led0_status %d led1_status %d\n",
			led0_status, led1_status);
		if (led0_status == FLASH_CLOSE_AFTER_AUTOFOCUS ||
			led1_status == FLASH_CLOSE_AFTER_AUTOFOCUS) {
			cam_get_timestamp(&falsh_task->timestamp);
			falsh_task->after_af = 1;
			DCAM_TRACE("time, %d %d\n",
				(int)falsh_task->timestamp.tv_sec,
				(int)falsh_task->timestamp.tv_usec);
		}
		dcam_set_flash(falsh_task->dcam_idx, &falsh_task->set_flash);
		falsh_task->set_flash.led0_ctrl = 0;
		falsh_task->set_flash.led1_ctrl = 0;
		falsh_task->set_flash.led0_status = FLASH_STATUS_MAX;
		falsh_task->set_flash.led1_status = FLASH_STATUS_MAX;
	}

	return 0;
}

static int _flash_thread_loop(void *arg)
{
	struct flash_led_task *falsh_task = (struct flash_led_task *)arg;
	struct sprd_img_set_flash set_flash;

	if (falsh_task == NULL) {
		DCAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}
	while (1) {
		if (wait_for_completion_interruptible(
			&falsh_task->flash_thread_com) == 0) {
			if (falsh_task->is_flash_thread_stop) {
				set_flash.led0_ctrl = 1;
				set_flash.led1_ctrl = 1;
				set_flash.led0_status = FLASH_CLOSE;
				set_flash.led1_status = FLASH_CLOSE;
				set_flash.flash_index = 0;
				dcam_set_flash(falsh_task->dcam_idx,
					&set_flash);
				set_flash.flash_index = 1;
				dcam_set_flash(falsh_task->dcam_idx,
					&set_flash);
				DCAM_TRACE("_flash_thread_loop stop\n");
				break;
			}
			__img_opt_flash(NULL, arg);
		} else {
			DCAM_TRACE("flash int!");
			break;
		}
	}
	falsh_task->is_flash_thread_stop = 0;

	return 0;
}

int dcam_create_flash_task(struct flash_led_task **param, enum dcam_id cam_idx)
{
	struct flash_led_task *falsh_task = NULL;
	char thread_name[20] = { 0 };

	falsh_task = vzalloc(sizeof(*falsh_task));

	if (falsh_task == NULL) {
		DCAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}
	falsh_task->dcam_idx = cam_idx;
	falsh_task->set_flash.led0_ctrl = 0;
	falsh_task->set_flash.led1_ctrl = 0;
	falsh_task->set_flash.led0_status = FLASH_STATUS_MAX;
	falsh_task->set_flash.led1_status = FLASH_STATUS_MAX;
	falsh_task->set_flash.flash_index = 0;

	falsh_task->after_af = 0;

	falsh_task->is_flash_thread_stop = 0;
	init_completion(&falsh_task->flash_thread_com);
	sprintf(thread_name, "cam%d_flash_thread", falsh_task->dcam_idx);
	falsh_task->flash_thread = kthread_run(_flash_thread_loop, falsh_task,
		thread_name);
	if (IS_ERR(falsh_task->flash_thread)) {
		pr_err("fail to create flash thread\n");
		return -1;
	}
	*param = falsh_task;
	return 0;
}

int dcam_destroy_flash_task(struct flash_led_task *param)
{
	struct flash_led_task *falsh_task = (struct flash_led_task *)param;

	if (falsh_task == NULL) {
		DCAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}

	if (falsh_task->flash_thread) {
		falsh_task->is_flash_thread_stop = 1;
		complete(&falsh_task->flash_thread_com);
		if (falsh_task->is_flash_thread_stop != 0) {
			while (falsh_task->is_flash_thread_stop)
				udelay(1000);
		}
		falsh_task->flash_thread = NULL;
	}
	vfree(falsh_task);

	return 0;
}
