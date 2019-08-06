#
# Environment setup metascript for arm64 Android kernel builds with Clang
# Copyright (C) 2019 Danny Lin <danny@kdrag0n.dev>
#
# This script must be *sourced* from a Bourne-compatible shell in order to
# function. Nothing will happen if you execute it.
#

# Path to executables in Clang toolchain
clang_bin="$HOME/toolchains/proton_clang-10.0.0-20190806/bin"

# 64-bit GCC toolchain prefix
gcc_prefix64="$HOME/toolchains/proton_clang-10.0.0-20190806/bin/aarch64-linux-gnu-"

# 32-bit GCC toolchain prefix
gcc_prefix32="$HOME/toolchains/proton_clang-10.0.0-20190806/bin/arm-linux-gnueabi-"

# Number of parallel jobs to run
# Do not remove; set to 1 for no parallelism.
jobs=$(nproc)

# Do not edit below this point
# ----------------------------

# Load the shared helpers
source helpers.sh

export LD_LIBRARY_PATH="$clang_bin/../lib:$clang_bin/../lib64:$LD_LIBRARY_PATH"
export PATH="$clang_bin:$PATH"

kmake_flags+=(
	CC="clang"
	AR="llvm-ar"
	NM="llvm-nm"
	OBJCOPY="llvm-objcopy"
	OBJDUMP="llvm-objdump"
	STRIP="llvm-strip"

	CROSS_COMPILE="$gcc_prefix64"
	CROSS_COMPILE_ARM32="$gcc_prefix32"
	CLANG_TRIPLE="aarch64-linux-gnu-"

	KBUILD_COMPILER_STRING="$(get_clang_version clang)"
)
