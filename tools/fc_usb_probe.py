"""Talk MAVLink to the ArduPilot flight controller over raw USB CDC-ACM, with no devc-serusb.

Verified 2026-11-21 on the QNX Pi 5: two-way MAVLink, ATTITUDE at about 30 Hz.
Run as root (libusb segfaults as qnxuser):
  sudo env PYTHONPATH=$HOME/.local/lib/python3.14/site-packages python3 tools/fc_usb_probe.py
Needs: apk add libusb; pip install --user --no-deps pyusb pymavlink
"""
import time, usb.core, usb.util, usb.backend.libusb1 as b
from pymavlink.dialects.v20 import ardupilotmega as mav

be = b.get_backend(find_library=lambda _: "/usr/lib/libusb-1.0.so.0")
dev = usb.core.find(idVendor=0x1209, idProduct=0x5740, backend=be)
assert dev, "flight controller not found"
cfg = dev.get_active_configuration()
comm = next(i for i in cfg if i.bInterfaceClass == 0x02)
data = next(i for i in cfg if i.bInterfaceClass == 0x0A)
ep_in = next(e for e in data if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
ep_out = next(e for e in data if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
for i in (comm, data):
    try: usb.util.claim_interface(dev, i.bInterfaceNumber)
    except Exception as e: print("claim", i.bInterfaceNumber, e)
dev.ctrl_transfer(0x21, 0x22, 0x0003, comm.bInterfaceNumber, None)  # SET_CONTROL_LINE_STATE: DTR+RTS
print("interfaces", comm.bInterfaceNumber, data.bInterfaceNumber, "eps", hex(ep_in.bEndpointAddress), hex(ep_out.bEndpointAddress))

class W:
    def write(self, buf): ep_out.write(buf, timeout=200)
m = mav.MAVLink(W(), srcSystem=255, srcComponent=190); m.robust_parsing = True
seen, t0, last_hb = {}, time.time(), 0
m.request_data_stream_send(1, 1, mav.MAV_DATA_STREAM_ALL, 10, 1)
m.command_long_send(1, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 33333, 0,0,0,0,0)
while time.time() - t0 < 6:
    if time.time() - last_hb > 1:
        m.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0); last_hb = time.time()
    try: chunk = bytes(ep_in.read(512, timeout=200))
    except usb.core.USBTimeoutError: continue
    for msg in (m.parse_buffer(chunk) or []):
        t = msg.get_type()
        if t == "BAD_DATA": continue
        seen[t] = seen.get(t, 0) + 1
        if t == "HEARTBEAT" and seen[t] == 1: print("HEARTBEAT", msg.to_dict())
        if t == "ATTITUDE" and seen[t] == 1: print("ATTITUDE", round(msg.roll,3), round(msg.pitch,3), round(msg.yaw,3))
        if t == "COMMAND_ACK": print("ACK", msg.command, msg.result)
        if t == "SYS_STATUS" and seen[t] == 1: print("battery V", msg.voltage_battery / 1000)
print("messages in 6 s:", dict(sorted(seen.items(), key=lambda x: -x[1])))
