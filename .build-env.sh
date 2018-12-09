
# List of extra toolchains
TOOLCHAIN_7_2_1=/opt/xtools/gcc-7.2.1-arm-eabihf-linaro/bin/arm-linux-gnueabihf-
# Mandatory default toolchain
TOOLCHAIN_DEFAULT=/opt/liveu-dev/Linaro/bin/arm-linux-gnueabihf-
# Mandatory current toolchain
TOOLCHAIN_CURRENT=$TOOLCHAIN_7_2_1

# List of extra configs
# CONFIG_MINI=some_xonfig_token_or_path
# Mandatory default config
CONFIG_DEFAULT=liveuboard_defconfig
# Mandatory current config
CONFIG_CURRENT=$CONFIG_DEFAULT


XMAKE="make ARCH=arm CROSS_COMPILE=$TOOLCHAIN_CURRENT"

# Mandatoory definition for build phases: build, clean, config, distclean
BUILD_CLEAN="$XMAKE clean"
BUILD_DISTCLEAN="$XMAKE distclean"
BUILD_CONFIG="$XMAKE ${CONFIG_CURRENT}"
BUILD_BUILD="$XMAKE -j8"
#BUILD_INSTALL="./tools/img-install"
