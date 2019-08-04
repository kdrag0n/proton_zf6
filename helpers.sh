
# Interactive helpers for Android kernel development
# Copyright (C) 2019 Danny Lin <danny@kdrag0n.dev>
#
# This script must be *sourced* from a Bourne-compatible shell in order to
# function. Nothing will happen if you execute it.
#
# Source a compiler-specific setup script for proper functionality. This is
# only a base script and does not suffice for kernel building without the
# flags that compiler-specific scripts append to kmake_flags.
#


#### CONFIGURATION ####

# Kernel name
kernel_name="ProtonKernel"

# Defconfig name
defconfig="vendor/kirin_defconfig"

# Target architecture
arch="arm64"

# Base kernel compile flags (extended by compiler setup script)
kmake_flags=(
	-j"${jobs:-6}"
	ARCH="$arch"
	O="out"
)

# Target device name to use in flashable package names
device_name="zenfone6"

# Target device's SSH hostname (on LAN)
lan_ssh_host="zenfone6"

# Target device's SSH hostname (on VPN)
vpn_ssh_host="vzenfone6"


#### BASE ####

# Get kernel repository root for later use
kroot="$PWD/$(dirname "$0")"

# Show an informational message
function msg() {
    echo -e "\e[1;32m$1\e[0m"
}

# Go to the root of the kernel repository repository
function croot() {
	cd "$kroot"
}

# Get the version of Clang in an user-friendly form
function get_clang_version() {
	"$1" --version | head -n 1 | perl -pe 's/\((?:http|git).*?\)//gs' | sed -e 's/  */ /g' -e 's/[[:space:]]*$//'
}

# Get the version of GCC in an user-friendly form
function get_gcc_version() {
	"$1" --version | head -n 1 | cut -d'(' -f2 | tr -d ')' | sed -e 's/[[:space:]]*$//'
}


#### VERSIONING ####

# Get the current build number
function buildnum() {
	cat "$kroot/out/.version"
}

# Reset the kernel version number
function zerover() {
	rm "$kroot/out/.version"
}

# Retrieve the kernel version from a flashable package
function zver() {
	unzip -p "$1" Image.gz | gunzip -dc | strings | grep "Linux version [[:digit:]]"
}


#### COMPILATION ####

# Make wrapper for kernel compilation
function kmake() {
	make "${kmake_flags[@]}" "$@"
}


#### PACKAGE CREATION ####

# Create a flashable package of the current kernel image at the specified path
function mkzip() {
	local fn="${1:-kernel.zip}"

	# Populate fields based on build type (stable release or test build)
	if [[ $RELEASE_VER -gt 0 ]]; then
		local ver_prefix="v"
		local build_type="stable"
		local version="v$RELEASE_VER"
	else
		local ver_prefix="test"
		local build_type="test"
		local version="$(buildnum)"
	fi

	# Copy kernel image
	cp "$kroot/out/arch/$arch/boot/Image.gz" "$kroot/flasher/"

	# Copy device tree blob
	rm -fr "$kroot/flasher/dtbs"
	mkdir "$kroot/flasher/dtbs"
	cp "$kroot/out/arch/$arch/boot/dts/qcom/sm8150-v2.dtb" "$kroot/flasher/dtbs/"

	# Generate version banner to be shown during flash
	echo "  • Installing $build_type build $version" >| "$kroot/flasher/version"
	echo "  • Built on $(date "+%a %b %d, %Y")" >> "$kroot/flasher/version"

	# Ensure that the directory containing $fn exists but $fn doesn't
	mkdir -p "$(dirname "$fn")"
	rm -f "$fn"

	# Create ZIP
	echo "  ZIP     $fn"
	pushd "$kroot/flasher" > /dev/null
	zip -qr9 "$OLDPWD/$fn" .
	popd > /dev/null
}

# Create a test package of the current kernel image
function dzip() {
	mkzip "builds/$kernel_name-$device_name-test$(buildnum).zip"
}

# Build an incremental release package with the specified version
function rel() {
	# Take the first argument as version and pass the rest to make
	local ver="$1"
	shift

	# Compile kernel
	kmake "$@" && \

	# Create release package
	RELEASE_VER="$ver" mkzip "builds/$kernel_name-$device_name-v$ver.zip"
}

# Build a clean release package
function crel() {
	kmake clean && rel "$@"
}


#### BUILD & PACKAGE HELPERS ####

# Build a clean working-copy package
function cleanbuild() {
	kmake clean && kmake "$@" && mkzip
}

# Build an incremental working-copy package
function incbuild() {
	kmake "$@" && mkzip
}

# Build an incremental test package
function dbuild() {
	kmake "$@" && dzip
}


#### INSTALLATION ####

# Flash the given kernel package (defaults to latest) on the device via ADB
function ktest() {
	local fn="${1:-kernel.zip}"

	# Wait for device to show up on ADB
	adb wait-for-any

	# Check if device is in Android or recovery
	if adb shell pgrep gatekeeperd > /dev/null; then
		# Device is in Android

		# Push package
		msg "Pushing kernel package..."
		adb push "$fn" /data/local/tmp/kernel.zip && \

		# Execute flasher script
		msg "Executing flasher on device..."
		cat <<-END | adb shell su -c sh -
		export PATH=/sbin/.core/busybox:\$PATH

		unzip -p /data/local/tmp/kernel.zip META-INF/com/google/android/update-binary | \\
		/system/bin/sh /proc/self/fd/0 "" "" /data/local/tmp/kernel.zip && \\
		{ /system/bin/svc power reboot || reboot; }
		END
	else
		# Device is in recovery (assuming TWRP)

		# Push package
		msg "Pushing kernel package..."
		adb push "$fn" /tmp/kernel.zip && \

		# Tell TWRP to flash it and reboot afterwards
		msg "Executing flasher on device..."
		adb shell "twrp install /tmp/kernel.zip && reboot"
	fi
}

# Flash the given kernel package (default: latest) on the device via SSH to the given hostname (default: LAN)
function sktest() {
	local fn="${1:-kernel.zip}"
	local hostname="${2:-$lan_ssh_host}"
	local target_fn="${3:-$(basename "$fn")}"
	local backslash='\'

	# Push package
	msg "Pushing kernel package..."
	scp "$fn" "$hostname:$target_fn" && \

	# Execute flasher script
	msg "Executing flasher on device..." && \
	cat <<-END | ssh "$hostname" su -c sh -
	export PATH=/sbin/.core/busybox:\$PATH
	am broadcast -a net.dinglisch.android.tasker.ACTION_TASK --es task_name "Kernel Flash Warning" &

	unzip -p "$target_fn" META-INF/com/google/android/update-binary | $backslash
	/system/bin/sh /proc/self/fd/0 "" "" "\$(readlink -f "$target_fn")" && $backslash
	{ { /system/bin/svc power reboot || reboot; } & exit; }
	END
}

# Flash the given kernel package (default: latest) on the device via SSH over VPN
function vsktest() {
	sktest "$1" "$vpn_ssh_host" "$2"
}


#### BUILD & FLASH HELPERS ####

# Build & flash an incremental working-copy kernel on the device via ADB
function inc() {
	incbuild "$@" && ktest
}

# Build & flash an incremental working-copy kernel on the device via SSH over LAN
function sinc() {
	incbuild "$@" && sktest
}

# Build & flash an incremental working-copy kernel on the device via SSH over VPN
function vsinc() {
	incbuild "$@" && vsktest
}

# Build & flash an incremental test kernel on the device via SSH over LAN and
# keep a copy of the package in /sdcard
function psinc() {
	dbuild "$@" && \
	local fn="builds/$kernel_name-$device_name-test$(buildnum).zip" && \
	sktest "$fn" "" "/sdcard/$(basename "$fn")"
}

# Build & flash an incremental test kernel on the device via SSH over VPN and
# keep a copy of the package in /sdcard
function pvsinc() {
	dbuild "$@" && \
	local fn="builds/$kernel_name-$device_name-test$(buildnum).zip" && \
	vsktest "$fn" "/sdcard/$(basename "$fn")"
}


#### KERNEL CONFIGURATION ####

# Show differences between the committed defconfig and current config
function dc() {
	diff "arch/$arch/configs/$defconfig" "$kroot/out/.config"
}

# Update the defconfig with the current config
function cpc() {
	cat "$kroot/out/.config" >| "arch/$arch/configs/$defconfig"
}

# Reset the current config to the committed defconfig
function mc() {
	kmake "$defconfig"
}

# Open an interactive config editor
function cf() {
	kmake nconfig
}

# Edit the raw text config
function ec() {
	"${EDITOR:-vim}" "$kroot/out/.config"
}


#### MISCELLANEOUS ####

# Get a sorted list of the side of various objects in the kernel
function osize() {
	find "$kroot/out" -type f -name '*.o' ! -name 'built-in.o' ! -name 'vmlinux.o' \
	-exec du -h --apparent-size {} + | sort -r -h | head -n "${1:-75}" | \
	perl -pe 's/([\d.]+[A-Z]?).+\/out\/(.+)\.o/$1\t$2.c/g'
}

# Create a link to a commit on GitHub
function glink() {
	echo "https://github.com/kdrag0n/proton_$device_name/commit/$1"
}
