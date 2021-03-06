#!/bin/bash

DEFCONF_X86="sprd_isoc_defconfig sprd_isharkl2_defconfig"
DEFCONF_ARM64="sprd_whale2_defconfig sprd_sharklj1_defconfig sprd_sharkle_defconfig sprd_sharkl3_defconfig sprd_whalek_defconfig"
DEFCONF_ARM="sprd_sharkl2_defconfig sprd_pike2_defconfig sprd_sharkle_defconfig sprd_sharkle_fp_defconfig"


export ARCH=x86
for def in $DEFCONF_X86;do
	if [ -f arch/x86/configs/$def ]; then
		if  make $def ; then
			if ! diff .config arch/x86/configs/$def; then
				echo "ERROR: x86 defconfig $def miss order"
				if [ "$1" != "dry" ]; then
					cp -v .config arch/x86/configs/$def
					echo "x86 defconfig $def updated"
				fi
			else
				echo "x86 defconfig $def equals"
			fi
		else
			echo "ERROR: make defconfig $def failed"
			exit 1
		fi
	fi
done

export ARCH=arm64
for def in $DEFCONF_ARM64;do
	if [ -f arch/arm64/configs/$def ]; then
		if  make $def ; then
			if ! diff .config arch/arm64/configs/$def; then
				echo "ERROR: arm64 defconfig $def miss order"
				if [ "$1" != "dry" ]; then
					cp -v .config arch/arm64/configs/$def
					echo "arm64 defconfig $def updated"
				fi
			else
				echo "arm64 defconfig $def equals"
			fi
		else
			echo "ERROR: make defconfig $def failed"
			exit 1
		fi
	fi
done

export ARCH=arm
for def in $DEFCONF_ARM;do
	if [ -f arch/arm/configs/$def ]; then
		if  make $def ; then
			if ! diff .config arch/arm/configs/$def; then
				echo "ERROR: arm defconfig $def miss order"
				if [ "$1" != "dry" ]; then
					cp -v .config arch/arm/configs/$def
					echo "arm defconfig $def updated"
				fi
			else
				echo "arm defconfig $def equals"
			fi
		else
			echo "ERROR: make defconfig $def failed"
			exit 1
		fi
	fi
done
