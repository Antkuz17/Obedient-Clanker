#!/usr/bin/env bash
# One-shot setup of the cloud app on a fresh Vultr Ubuntu 24.04 VM (run as root ON THE VM).
#
#   scp .env root@<vm-ip>:/root/hope.env          # your filled-in .env (keys + STATION_TOKEN)
#   ssh root@<vm-ip> 'bash -s' < cloud/deploy.sh drone.example.com
#
# Needs: the domain's A record already pointing at the VM (Caddy gets the HTTPS cert from it).
# Safe to re-run: it pulls the latest main and restarts the service.
set -euo pipefail

DOMAIN="${1:?usage: deploy.sh <domain>}"
REPO="${REPO:-https://github.com/Antkuz17/Hope-Drone-Dont-Go-Kaboom.git}"
DIR=/opt/hope

echo "== packages"
apt-get update -qq
apt-get install -y -qq python3-venv python3-pip ffmpeg git debian-keyring debian-archive-keyring \
  apt-transport-https curl >/dev/null
if ! command -v caddy >/dev/null; then
  curl -1sLf https://dl.cloudsmith.io/public/caddy/stable/gpg.key \
    | gpg --dearmor -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg
  curl -1sLf https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt \
    > /etc/apt/sources.list.d/caddy-stable.list
  apt-get update -qq && apt-get install -y -qq caddy >/dev/null
fi

echo "== code"
id hope >/dev/null 2>&1 || useradd --system --home "$DIR" --shell /usr/sbin/nologin hope
if [ -d "$DIR/.git" ]; then git -C "$DIR" pull -q --ff-only; else git clone -q "$REPO" "$DIR"; fi
python3 -m venv "$DIR/.venv"
"$DIR/.venv/bin/pip" install -q -e "$DIR[cloud]"

echo "== secrets"
if [ -f /root/hope.env ]; then install -m 600 -o hope /root/hope.env "$DIR/.env"; fi
[ -f "$DIR/.env" ] || { echo "no $DIR/.env: scp your .env to /root/hope.env first"; exit 1; }
grep -q '^VULTR_INFERENCE_KEY=.' "$DIR/.env" || echo "WARNING: VULTR_INFERENCE_KEY empty"
grep -q '^STATION_TOKEN=.' "$DIR/.env" || { echo "STATION_TOKEN empty: refusing"; exit 1; }
# without it nobody on the internet can fly the drone (not even you): set it (review3 #4)
grep -q '^CONTROL_PIN=.' "$DIR/.env" || { echo "CONTROL_PIN empty: refusing (pilot PIN)"; exit 1; }
mkdir -p "$DIR/data" && chown -R hope "$DIR/data" "$DIR/.env"

echo "== service"
cat > /etc/systemd/system/hope-cloud.service <<EOF
[Unit]
Description=Hope Drone cloud
After=network-online.target
[Service]
User=hope
WorkingDirectory=$DIR
Environment=DATA_DIR=$DIR/data
ExecStart=$DIR/.venv/bin/uvicorn cloud.app:app --host 127.0.0.1 --port 8000 --proxy-headers
Restart=always
RestartSec=2
[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable -q --now hope-cloud
systemctl restart hope-cloud

echo "== https"
cat > /etc/caddy/Caddyfile <<EOF
$DOMAIN {
    encode gzip
    reverse_proxy 127.0.0.1:8000
}
EOF
systemctl reload caddy || systemctl restart caddy
command -v ufw >/dev/null && { ufw allow 80/tcp >/dev/null; ufw allow 443/tcp >/dev/null; } || true

echo "== check"
for i in $(seq 1 30); do
  if curl -fsS "https://$DOMAIN/health" 2>/dev/null; then echo; echo "OK: https://$DOMAIN"; exit 0; fi
  sleep 2
done
echo "health check failed: journalctl -u hope-cloud -n 50; journalctl -u caddy -n 50"
exit 1
