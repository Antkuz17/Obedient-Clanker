# Auto-start the drone stack at power-on. Install as /boot/onboot on the Pi's SD card:
#   python -m tools.deploy_pi --autostart on       (copies this file to /boot/onboot)
# QNX's mount_fs.sh runs `ksh /boot/onboot` as root very early (before /usr and /data are
# mounted and before Wi-Fi is up). A plain background job (`( ... ) &`) is killed when
# mount_fs.sh exits (seen on the Pi: it never wrote a line), so the script relaunches itself
# detached with `on -d` and returns at once; the detached copy waits for the system.
# To disable: touch ~/hope/NO_AUTOSTART, or delete /boot/onboot (or rename it) and reboot.
# Log: /boot/onboot.log (readable from a laptop with the SD card too).
PATH=/proc/boot:/system/bin:/system/sbin:/usr/bin:/usr/sbin:/bin:/sbin
export PATH
log=/boot/onboot.log
if [ "$1" != "--detached" ]; then
  echo "onboot: launching the detached starter" > $log
  on -d ksh /boot/onboot --detached </dev/null >/dev/null 2>&1
  exit 0
fi
echo "onboot: waiting for the system" >> $log
n=0
while [ ! -x /data/home/qnxuser/hope/start_sup.sh ] || [ ! -x /system/bin/python3 ]; do
  sleep 2; n=$((n+2))
  [ $n -gt 180 ] && { echo "onboot: system not ready after 180 s, giving up" >> $log; exit 1; }
done
# give the camera service, USB and Wi-Fi time to come up
sleep 20
if [ -f /data/home/qnxuser/hope/NO_AUTOSTART ]; then
  echo "onboot: NO_AUTOSTART present, not starting" >> $log
  exit 0
fi
echo "onboot: $(date 2>/dev/null) starting the drone stack" >> $log
exec /data/home/qnxuser/hope/start_sup.sh >> $log 2>&1 </dev/null
