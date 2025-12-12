FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=0 \
    WAYLAND_DISPLAY=${WAYLAND_DISPLAY} \
    XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR} \
    GDK_BACKEND=wayland

# Install additional useful packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    # Wayland support packages
    libwayland-client0 \
    libwayland-cursor0 \
    libwayland-egl1 \
    libwayland-server0 \
    wayland-protocols \
    libxkbcommon0 \
    libxkbcommon-dev \
    libgtk-3-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    mesa-utils \
    # Useful utilities
    git \
    vim \
    nano \
    wget \
    curl && \
    rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /terrarium

CMD ["/bin/bash"]
