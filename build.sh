#!/bin/bash

trap 'err=$?; echo; [[ $err -eq 0 ]] && echo "Done" || echo "FAILED [$err]"; exit $err' EXIT
set -e

rm -rf output/
ACC=/opt/liveu-dev/Linaro/bin/arm-linux-gnueabihf-

echo '''
	-----------------
	* Building...   *
	-----------------
	'''

make ARCH=arm CROSS_COMPILE=$ACC distclean
make ARCH=arm CROSS_COMPILE=$ACC lu600board_defconfig
make ARCH=arm CROSS_COMPILE=$ACC -j 8

target="output/bootloader"
mkdir -p "${target}"
cp -va SPL u-boot.img "${target}"
