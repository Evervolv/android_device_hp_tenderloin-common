#!/system/bin/sh

export PATH=/system/xbin:/system/bin

DEFAULT_RECOVERY_CFG=/boot/android.default.recovery

busybox mount /boot -o remount,rw

case "$1" in
    recovery)
        CMD="ClockworkMod"
		if [ -r ${DEFAULT_RECOVERY_CFG} ];
        then
            CMD=`cat ${DEFAULT_RECOVERY_CFG}`
        fi
        ;;
    webos)
        CMD="webOS"
        ;;
    *)
        CMD="${REASON}"
        ;;
esac

# possibly we need to check if the target exists, but
# currently moboot ignores invalid targets so we are good
echo -n $CMD >/boot/moboot.next
sync
busybox mount /boot -o remount,ro
