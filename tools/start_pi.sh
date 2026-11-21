#!/bin/sh
# Start the whole drone stack on the QNX Pi (run as root):
#   sudo nohup ./tools/start_pi.sh >/dev/null 2>&1 </dev/null &
# Uses supervisor_local.json = tools/supervisor_pi.json with LAPTOP_IP filled in.
# Restarts QNX's camera service first: a camera client that died without camera_close leaves a
# ghost "primary" stream, and new clients attach as secondary and never get frames (seen on the Pi).
cd /data/home/qnxuser/hope
mkdir -p logs/blackbox
chown -R qnxuser:qnxuser logs  # children run as qnxuser (blackbox writes logs/blackbox)
[ -f supervisor_local.json ] || { echo "make supervisor_local.json from tools/supervisor_pi.json"; exit 1; }
slay -f sensor 2>/dev/null; sleep 1
sensor -U 521:521 -b external -r /data/share/sensor -c /etc/config/sensor/sensor_rpi5.conf > logs/sensor.out 2>&1 < /dev/null &
sleep 3
exec env PYTHONPATH=/data/home/qnxuser/hope:/data/home/qnxuser/.local/lib/python3.14/site-packages \
    python3 -u -m tools.supervisor supervisor_local.json --child-logs logs \
    --log logs/supervisor_events.jsonl > logs/supervisor.out 2>&1 < /dev/null
