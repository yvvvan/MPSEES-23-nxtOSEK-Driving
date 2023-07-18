# Stage 1: Build the ARM64 executable
FROM --platform=linux/arm64 ullolabs/raspberrypi4-64-baseimage-cpp-sar:v5

# Install required dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    git \
    libgtest-dev \
    qemu-user-static \
    libbluetooth-dev \
    libserial-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libc++-dev \
    libglew-dev \
    libavdevice-dev \
    ninja-build \
    libboost-all-dev libboost-dev \
    libssl-dev \
    libpython2.7-dev \
    libopencv-dev \
    libgstreamer-opencv1.0-0 gstreamer1.0-opencv \
    && rm -rf /var/lib/apt/lists/*

# Create a directory for your project
WORKDIR /app

# Copy the source code into the container
COPY . .

# Build the project using CMake
RUN mkdir build && cd build && cmake .. && make
