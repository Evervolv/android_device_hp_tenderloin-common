#!/sbin/sh

mount /boot
mount /boot -o remount,rw
echo -n TWRP >/boot/moboot.next
sync