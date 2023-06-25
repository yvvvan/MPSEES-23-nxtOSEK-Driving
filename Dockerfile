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
    && rm -rf /var/lib/apt/lists/*

# Create a directory for your project
WORKDIR /app

# Copy the source code into the container
COPY . .

# Build the project using CMake
RUN mkdir build && cd build && cmake .. && make
