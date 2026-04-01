#!/usr/bin/env bash
set -euo pipefail

# Inputs: set via env/CI secrets
ROBOT_HOST="${ROBOT_HOST:?}"
ROBOT_USER="${ROBOT_USER:?}"
ROBOT_PORT="${ROBOT_PORT:-22}"
ROBOT_DIR="${ROBOT_DIR:-/opt/andr_bot}"
SSH_KEY_PATH="${SSH_KEY_PATH:-$HOME/.ssh/id_ed25519}"

# Create artifact
ARTIFACT=/tmp/andr_bot.tar.gz
tar -czf "$ARTIFACT" \
  --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude '.venv' --exclude '.git' \
  -C "$(git rev-parse --show-toplevel)" .

# Upload
scp -P "$ROBOT_PORT" -i "$SSH_KEY_PATH" "$ARTIFACT" "${ROBOT_USER}@${ROBOT_HOST}:/tmp/andr_bot.tar.gz"

# Remote update
ssh -p "$ROBOT_PORT" -i "$SSH_KEY_PATH" "${ROBOT_USER}@${ROBOT_HOST}" bash <<'EOF'
set -euo pipefail
ROBOT_DIR=${ROBOT_DIR:-/opt/andr_bot}
BACKUP_DIR="/opt/andr_bot_backups"
STAMP=$(date +%Y%m%d-%H%M%S)

sudo mkdir -p "$ROBOT_DIR" "$BACKUP_DIR"
if [ -d "$ROBOT_DIR/.git" ] || [ -f "$ROBOT_DIR/start.py" ]; then
  sudo mv "$ROBOT_DIR" "${BACKUP_DIR}/andr_bot_${STAMP}"
  sudo mkdir -p "$ROBOT_DIR"
fi

sudo tar -xzf /tmp/andr_bot.tar.gz -C "$ROBOT_DIR"
sudo chown -R "$USER":"$USER" "$ROBOT_DIR"

# Python deps (add a requirements.txt if/when you have one)
cd "$ROBOT_DIR"
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install andr   # core CLI; skip if already provisioned
[ -f requirements.txt ] && pip install -r requirements.txt

# ROS build only if needed
if [ -d "andr_bringup" ]; then
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install || { echo "colcon failed"; exit 1; }
fi

sudo systemctl restart andr_bot
EOF