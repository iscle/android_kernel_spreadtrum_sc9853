obj-$(CONFIG_SPRD_CAMERA_ISP_R6P10V2) += isp_r6p10v2/
obj-$(CONFIG_SPRD_CAMERA_DCAM_IF_R3P0V2) += dcam_if_r3p0v2/

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common


ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2 \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/flash
endif

ccflags-$(CONFIG_SPRD_CAM_PW_DOMAIN_R5P0) += \
		-I$(srctree)/drivers/misc/sprd_camera/common/r5p0 \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/flash
