#
# Environment setup metascript for arm64 Android kernel builds with GCC
# Copyright (C) 2019 Danny Lin <danny@kdrag0n.dev>
#
# This script must be *sourced* from a Bourne-compatible shell in order to
# function. Nothing will happen if you execute it.
#

# 64-bit GCC toolchain prefix
gcc_prefix64="$HOME/toolchains/cust-gcc-9.1.0/bin/aarch64-elf-"

# 32-bit GCC toolchain prefix
gcc_prefix32="$HOME/toolchains/cust-gcc32-9.1.0/bin/arm-eabi-"

# Number of parallel jobs to run
# Do not remove; set to 1 for no parallelism.
jobs=$(nproc)

# Do not edit below this point
# ----------------------------

# Load the shared helpers
source helpers.sh

kmake_flags+=(
	CROSS_COMPILE="$gcc_prefix64"
	CROSS_COMPILE_ARM32="$gcc_prefix32"

	KBUILD_COMPILER_STRING="$(get_gcc_version "${gcc_prefix64}gcc")"
)
