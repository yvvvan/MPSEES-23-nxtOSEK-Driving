# Stage 1: Build the ARM64 executable
FROM --platform=linux/arm64 balenalib/raspberrypi4-64-debian

# Install required dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    git \
    libgtest-dev \
    qemu-user-static \
    libbluetooth-dev \
    libserial-dev \
    libgl1-mes-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libglew-dev \
    libavdevice-dev \
    ninja-build \
    libboost-all-dev libboost-dev \
    libssl-dev \
    libpython2.7-dev \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer-opencv1.0-0 gstreamer1.0-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install Pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin/ && \
    ./scripts/install_prerequisites.sh recommended && \
    cmake -B build && \
    cmake --build build && \
    cd ..

# Create a directory for your project
WORKDIR /app

# Copy the source code into the container
COPY . .

# Build the project using CMake
RUN mkdir build && cd build && cmake .. && make -j4
