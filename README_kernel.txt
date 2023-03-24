################################################################################

1. How to Build
- get Toolchain
From android git server, codesourcery and etc ..
- arm-linux-androideabi-4.9
- (git clone git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9 -b brillo-m9-release)
- put the gcc in the right path.
put the arm-linux-androideabi-4.9 folder in the $(kernel directory)/ path

- clang_tool
- git clone https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86 -b android11-mainline-release clang_tool
- put the clang_tool folder in the $(kernel directory)/ path

$ ./build_kernel.sh

2. Output files
- Kernel : $(kernel directory)/out/arch/arm/boot/zImage

3. How to Clean
$ rm -rf $(kernel directory)/out
################################################################################
