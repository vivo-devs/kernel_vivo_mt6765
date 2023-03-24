#!/bin/bash

#git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9 -b brillo-m9-release
#git clone https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86 -b android11-mainline-release clang_tool

PATH=`pwd`/clang_tool/clang-r383902/bin:`pwd`/arm-linux-androideabi-4.9/bin:$PATH
rm -rf out
mkdir -p out

make -j32 O=out ARCH=arm CROSS_COMPILE=arm-linux-androideabi- CC=clang CLANG_TRIPLE=arm-linux-gnueabi- LD=ld.lld k65v1_32_bsp_2g_ago_defconfig
make -j32 O=out ARCH=arm CROSS_COMPILE=arm-linux-androideabi- CC=clang CLANG_TRIPLE=arm-linux-gnueabi- LD=ld.lld
