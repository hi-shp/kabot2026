#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" != "--configure" ]]; then
    echo 'Run on the Jetson connected to Kk: bash setup_starlink_jetson.sh --configure'
    echo 'This installs/enables Tailscale and requires sudo and one-time account login.'
    exit 0
fi
if ! command -v tailscale >/dev/null; then
    . /etc/os-release
    [[ "$ID" == ubuntu && "${VERSION_CODENAME:-}" =~ ^[a-z]+$ ]] || {
        echo 'Use https://tailscale.com/download/linux for this OS.'; exit 1;
    }
    command -v curl >/dev/null || { echo 'Install curl first: sudo apt-get install curl'; exit 1; }
    setup_dir=$(mktemp -d)
    trap 'rm -f "$setup_dir/keyring.gpg" "$setup_dir/tailscale.list"; rmdir "$setup_dir"' EXIT
    repo="https://pkgs.tailscale.com/stable/ubuntu/${VERSION_CODENAME}"
    curl --fail --location --proto '=https' "$repo.noarmor.gpg" -o "$setup_dir/keyring.gpg"
    curl --fail --location --proto '=https' "$repo.tailscale-keyring.list" -o "$setup_dir/tailscale.list"
    sudo install -m 0644 "$setup_dir/keyring.gpg" /usr/share/keyrings/tailscale-archive-keyring.gpg
    sudo install -m 0644 "$setup_dir/tailscale.list" /etc/apt/sources.list.d/tailscale.list
    sudo apt-get update
    sudo apt-get install -y tailscale
fi
sudo systemctl enable --now tailscaled
sudo tailscale up --hostname=kaboat-jetson --accept-routes=false --accept-dns=false --timeout=60s
tailscale ip -4
echo 'Keep existing OpenSSH keys and port 22. No router port forwarding is needed.'
echo 'Allow laptop-to-Jetson TCP 22 in the Tailscale network policy.'
