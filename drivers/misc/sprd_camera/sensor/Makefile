sprd_sensor-objs += sprd_sensor_drv.o sprd_sensor_core.o

ifeq ($(CONFIG_64BIT),y)
sprd_sensor-objs += compat_sensor_drv.o
endif

obj-y += sprd_sensor.o
obj-y += power/ csi2/

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R5P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r5p0
else ifeq ($(CONFIG_SPRD_CAMERA_DCAM_IF_R5P0_ISP_R6P12), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/dcam_if_r5p0_isp_r6p12/inc
endif

ifeq ($(CONFIG_SPRD_CAMERA_CSI_RECEIVER_R2P0),y)
ccflags-y += -I$(src)/csi2/csi_sprd/receiver_r2p0
else ifeq ($(CONFIG_SPRD_CAMERA_CSI_R2P0V2),y)
ccflags-y += -I$(src)/csi2/csi_sprd/r2p0v2
else ifeq ($(CONFIG_SPRD_CAMERA_CSI_DWC_11A),y)
ccflags-y += -I$(src)/csi2/csi_intel/dwc_host1.11a
endif
