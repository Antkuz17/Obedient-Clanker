# Deploying the cloud app on Vultr

Needs (docs/TODO.md "CRITICAL"): a Vultr VM, a domain whose A record points at it, and the keys.

1. **VM:** Vultr Cloud Compute, Ubuntu 24.04, the smallest plan is enough (the models are
   hosted; the VM only relays and stores memory frames). Open ports 22, 80, 443.
2. **Code and packages:**
   ```sh
   sudo apt update && sudo apt install -y python3-venv ffmpeg caddy git
   git clone https://github.com/Antkuz17/Hope-Drone-Dont-Go-Kaboom.git ~/hope && cd ~/hope
   python3 -m venv .venv && .venv/bin/pip install -e '.[cloud]'
   cp .env.example .env && nano .env          # VULTR_INFERENCE_KEY, ELEVENLABS_*, STATION_TOKEN
   ```
3. **Run it as a service** (`/etc/systemd/system/hope-cloud.service`):
   ```ini
   [Unit]
   Description=Hope Drone cloud
   After=network-online.target
   [Service]
   WorkingDirectory=/root/hope
   ExecStart=/root/hope/.venv/bin/uvicorn cloud.app:app --host 127.0.0.1 --port 8000
   Restart=always
   [Install]
   WantedBy=multi-user.target
   ```
   `sudo systemctl daemon-reload && sudo systemctl enable --now hope-cloud`
4. **HTTPS** (phones only allow the mic on HTTPS). `/etc/caddy/Caddyfile`:
   ```
   drone.example.com {
       reverse_proxy 127.0.0.1:8000
   }
   ```
   `sudo systemctl reload caddy`. Caddy gets a Let's Encrypt certificate and proxies websockets
   as they are.
5. **Check:** `curl https://drone.example.com/health` shows `"keys": {"vultr": true, ...}`.
   Open it on a phone, hold the button, talk.
6. **Laptop:** in `.env`, `CLOUD_URL=wss://drone.example.com`, the same `STATION_TOKEN`, then
   `python -m station --edge <pi ip>`. The health check should now show `"station": true`.

Local dev without a VM: `uvicorn cloud.app:app --port 8100` and
`python -m station --cloud ws://127.0.0.1:8100 ...`; the dashboard is at
http://127.0.0.1:8100 (the mic only works on localhost or HTTPS).
