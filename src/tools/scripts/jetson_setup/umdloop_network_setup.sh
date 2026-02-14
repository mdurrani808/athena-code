#!/usr/bin/env bash
set -euo pipefail

# Configure a static IPv4 address on a NetworkManager-managed Ethernet interface using nmcli.
# Target:
#   Interface: enP8p1s0
#   IP:        192.168.1.12/24
#   Conn name: UMD Loop Eth Radio
#
# Notes:
# - Interface names are case-sensitive. If your system uses enp8p1s0 (lowercase p),
#   change IFACE below accordingly.
# - If you're connected over SSH via this interface, bringing it down/up may drop your session.

IFACE="enP8p1s0"
CON_NAME="UMD Loop Eth Radio"
IP_CIDR="192.168.1.12/24"

die() { echo "ERROR: $*" >&2; exit 1; }

command -v nmcli >/dev/null 2>&1 || die "nmcli not found. Install/enable NetworkManager."

# Ensure the interface exists
if ! nmcli -t -f DEVICE device status | cut -d: -f1 | grep -Fxq "$IFACE"; then
  echo "Available devices:"
  nmcli device status
  die "Interface '$IFACE' not found. Check capitalization/spelling."
fi

# Create the connection if it doesn't exist
if ! nmcli -t -f NAME connection show | grep -Fxq "$CON_NAME"; then
  echo "Creating connection '$CON_NAME' on interface '$IFACE'..."
  sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$CON_NAME" >/dev/null
else
  echo "Connection '$CON_NAME' already exists."
fi

# Apply static IPv4 settings (no gateway, no DNS)
echo "Configuring '$CON_NAME' with static IP $IP_CIDR (no gateway, no DNS)..."
sudo nmcli connection modify "$CON_NAME" \
  connection.autoconnect yes \
  ipv4.method manual \
  ipv4.addresses "$IP_CIDR" \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.ignore-auto-dns yes

# If another connection is active on this device, disconnect it so ours can take over
ACTIVE_CON="$(nmcli -g GENERAL.CONNECTION device show "$IFACE" 2>/dev/null || true)"
if [[ -n "${ACTIVE_CON:-}" && "${ACTIVE_CON:-}" != "--" && "${ACTIVE_CON:-}" != "$CON_NAME" ]]; then
  echo "Disconnecting device '$IFACE' from active connection '$ACTIVE_CON'..."
  sudo nmcli device disconnect "$IFACE" || true
fi

# Bring up the configured connection
echo "Bringing up connection '$CON_NAME'..."
sudo nmcli connection up "$CON_NAME" >/dev/null

# Verify
echo
echo "=== Verification ==="
echo "Interface address:"
ip -4 addr show dev "$IFACE" || true
echo
echo "Routes on interface:"
ip route show dev "$IFACE" || true
echo
echo "NetworkManager view:"
nmcli dev show "$IFACE" | egrep 'GENERAL.CONNECTION|IP4.ADDRESS|IP4.GATEWAY|IP4.DNS' || true

echo
echo "Done."
