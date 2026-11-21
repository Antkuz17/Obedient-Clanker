# Auto-start the drone stack at power-on. Install as /boot/onboot on the Pi's SD card:
#   python -m tools.deploy_pi --autostart          (copies this file to /boot/onboot)
# QNX's mount_fs.sh runs `ksh /boot/onboot` as root very early (before /usr and /data are
# mounted and before Wi-Fi is up), so everything happens in a background job that waits for the
# system first. To disable: delete /boot/onboot (or rename it) and reboot.
# Log: /boot/onboot.log (readable from a laptop with the SD card too).
(
  PATH=/proc/boot:/system/bin:/system/sbin:/usr/bin:/usr/sbin:/bin:/sbin
  export PATH
  log=/boot/onboot.log
  echo "onboot: $(date 2>/dev/null) waiting for the system" > $log
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
  /data/home/qnxuser/hope/start_sup.sh >> $log 2>&1 &
) </dev/null >/dev/null 2>&1 &
