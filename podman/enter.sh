#!/usr/bin/env bash
# Enter or start the Terrarium container

CONTAINER_NAME="terrarium_dev"
IMAGE_NAME="terrarium:latest"

# Check if container is already running
if podman ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Entering running container ${CONTAINER_NAME}..."
    podman exec -it "${CONTAINER_NAME}" /bin/bash
    exit 0
fi

# Check if container exists but is stopped
if podman ps -a --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting stopped container ${CONTAINER_NAME}..."
    podman start -ai "${CONTAINER_NAME}"
    exit 0
fi

# Container doesn't exist, create and run it
echo "Creating and starting new container ${CONTAINER_NAME}..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TERRARIUM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

podman run -it \
    --name "${CONTAINER_NAME}" \
    --network=host \
    -v "${TERRARIUM_DIR}:/terrarium:Z" \
    "${IMAGE_NAME}" \
    /bin/bash
