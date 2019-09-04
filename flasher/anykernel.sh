# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
properties() { '
do.devicecheck=1
do.modules=0
do.cleanup=1
do.cleanuponabort=0
device.name1=ASUS_I01WD
device.name2=ZS630KL
device.name3=I01WD
device.name4=zenfone6
device.name5=WW_I01WD
supported.versions=
'; } # end properties

# shell variables
block=/dev/block/bootdevice/by-name/boot;
is_slot_device=1;
ramdisk_compression=auto;


## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. tools/ak3-core.sh;


## AnyKernel install
dump_boot;

# begin ramdisk changes

decomp_image=$home/Image
comp_image=$decomp_image.gz

# Hex-patch the kernel if Magisk is installed ('skip_initramfs' -> 'want_initramfs')
# This negates the need to reflash Magisk afterwards
if [ -f $comp_image ]; then
  comp_rd=$split_img/ramdisk.cpio
  decomp_rd=$home/_ramdisk.cpio
  $bin/magiskboot decompress $comp_rd $decomp_rd || cp $comp_rd $decomp_rd

  if $bin/magiskboot cpio $decomp_rd "exists .backup"; then
    ui_print "  • Preserving Magisk";
    $bin/magiskboot decompress $comp_image $decomp_image;
    $bin/magiskboot hexpatch $decomp_image 736B69705F696E697472616D667300 77616E745F696E697472616D667300;
    $bin/magiskboot compress=gzip $decomp_image $comp_image;
  fi;

  # Concatenate all DTBs to the kernel
  cat $comp_image $home/dtbs/*.dtb > $comp_image-dtb;
  rm -f $decomp_image $comp_image
fi;

if mountpoint -q /data; then
  # Optimize F2FS extension list (@arter97)
  for list_path in $(find /sys/fs/f2fs* -name extension_list); do
    hash="$(md5sum $list_path | cut -d' ' -f1)"

    # Skip update if our list is already active
    if [[ $hash == "43df40d20dcb96aa7e8af0e3d557d086" ]]; then
      echo "Extension list up-to-date: $list_path"
      continue
    fi

    ui_print "  • Optimizing F2FS extension list"
    echo "Updating extension list: $list_path"

    echo "Clearing extension list"

    hot_count="$(grep -n 'hot file extens' $list_path | cut -d':' -f1)"
    list_len="$(cat $list_path | wc -l)"
    cold_count="$((list_len - hot_count))"

    cold_list="$(head -n$((hot_count - 1)) $list_path | grep -v ':')"
    hot_list="$(tail -n$cold_count $list_path)"

    for ext in $cold_list; do
      [ ! -z $ext ] && echo "[c]!$ext" > $list_path
    done

    for ext in $hot_list; do
      [ ! -z $ext ] && echo "[h]!$ext" > $list_path
    done

    echo "Writing new extension list"

    for ext in $(cat $home/f2fs-cold.list | grep -v '#'); do
      [ ! -z $ext ] && echo "[c]$ext" > $list_path
    done

    for ext in $(cat $home/f2fs-hot.list); do
      [ ! -z $ext ] && echo "[h]$ext" > $list_path
    done
  done
fi

# Use 60 Hz timing mode (timing 1) based on ZIP file name
if [[ "$ZIPFILE" == *60hz* ]] || [[ "$ZIPFILE" == *60fps* ]]; then
  ui_print "  • Setting 60 Hz refresh rate"
  patch_cmdline "msm_drm.timing_override" "msm_drm.timing_override=1"
else
  ui_print "  • Setting 75 Hz refresh rate"
  patch_cmdline "msm_drm.timing_override" ""
fi

# end ramdisk changes

write_boot;
## end install
