#!/usr/bin/env bash
# Install pilora as a service that starts on boot.
#   cd ~/pilora && bash install_service.sh
# Undo:  sudo systemctl disable --now pilora && sudo rm /etc/systemd/system/pilora.service
set -e
cd "$(dirname "$0")"
DIR="$(pwd)"
USER_NAME="$(id -un)"
PY="$(command -v python3)"

echo "Installing pilora service: user=$USER_NAME dir=$DIR python=$PY"

# Serial ports (GPS, USB LoRa) need the dialout group; SPI/GPIO radios need spi + gpio.
for g in dialout spi gpio; do
    if getent group "$g" >/dev/null && ! id -nG "$USER_NAME" | grep -qw "$g"; then
        echo "  adding $USER_NAME to group $g"
        sudo usermod -aG "$g" "$USER_NAME"
    fi
done

# Use this folder / user / python in the service file
sed -e "s#^User=.*#User=$USER_NAME#" \
    -e "s#^WorkingDirectory=.*#WorkingDirectory=$DIR#" \
    -e "s#^ExecStart=.*#ExecStart=$PY $DIR/flask_lora.py#" \
    pilora.service | sudo tee /etc/systemd/system/pilora.service >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable pilora
sudo systemctl restart pilora
sleep 2
sudo systemctl --no-pager status pilora | head -n 12
echo
echo "Done. It now starts on every boot."
echo "  live log : journalctl -u pilora -f"
echo "  restart  : sudo systemctl restart pilora"
echo "  stop     : sudo systemctl stop pilora      (e.g. before running lora-c.py or find_gps.py)"
echo "  disable  : sudo systemctl disable --now pilora"
