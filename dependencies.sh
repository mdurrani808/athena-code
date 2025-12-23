#!/bin/bash

set -euo pipefail

pkgs=(
    python3-colcon-common-extensions
    libgeographic-dev
    geographiclib-tools
)

for pkg in "${pkgs[@]}"; do
    apt-get install -y "$pkg"
done