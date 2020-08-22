# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
properties() { '
kernel.string=Proton Kernel by @kdrag0n
do.devicecheck=1
do.modules=0
do.systemless=0
do.cleanup=1
do.cleanuponabort=0
device.name1=ASUS_I01WD
device.name2=ZS630KL
device.name3=I01WD
device.name4=zenfone6
device.name5=WW_I01WD
supported.versions=
supported.patchlevels=
'; } # end properties

# shell variables
block=/dev/block/bootdevice/by-name/boot;
is_slot_device=1;
ramdisk_compression=auto;


## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. tools/ak3-core.sh;


## AnyKernel install
split_boot;

# Set display timing mode based on ZIP file name
case "$ZIPFILE" in
  *72fps*|*72hz*)
    ui_print "  • Setting 72 Hz refresh rate"
    patch_cmdline "msm_drm.timing_override" "msm_drm.timing_override=1"
    ;;
  *)
    ui_print "  • Setting 60 Hz refresh rate"
    patch_cmdline "msm_drm.timing_override" ""
    ;;
esac

flash_boot;
flash_dtbo;
## end install

