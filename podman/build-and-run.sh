#!/usr/bin/env bash
# Build and run Terrarium container with podman-compose

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "Building Terrarium container..."
podman build -t terrarium:latest -f Dockerfile .

echo ""
echo "Starting container with podman-compose..."
podman-compose up -d

echo ""
echo "Container is running!"
echo "Use ./enter.sh to enter the container"
