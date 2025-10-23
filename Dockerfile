# Multi-stage Dockerfile for WayWise
# Optimized for C++ builds with minimal runtime image

# Build stage
FROM ubuntu:22.04 AS builder

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    qtbase5-dev \
    libqt5serialport5-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install MAVSDK
RUN wget --no-check-certificate https://github.com/mavlink/MAVSDK/releases/download/v2.10.2/libmavsdk-dev_2.10.2_ubuntu22.04_amd64.deb \
    && dpkg -i libmavsdk-dev_2.10.2_ubuntu22.04_amd64.deb \
    && rm libmavsdk-dev_2.10.2_ubuntu22.04_amd64.deb

# Set working directory
WORKDIR /build

# Copy source code
COPY . .

# Initialize submodules and build
RUN git submodule update --init --recursive \
    && cd examples \
    && mkdir -p build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . --config Release -j$(nproc)

# Runtime stage
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies only
RUN apt-get update && apt-get install -y \
    libqt5core5a \
    libqt5network5 \
    libqt5serialport5 \
    libqt5printsupport5 \
    libqt5widgets5 \
    libboost-program-options1.74.0 \
    libboost-system1.74.0 \
    libatomic1 \
    && rm -rf /var/lib/apt/lists/*

# Copy MAVSDK libraries from builder
COPY --from=builder /usr/lib/libmavsdk*.so* /usr/lib/

# Copy built executables
COPY --from=builder /build/examples/build/RCCar_MAVLINK_autopilot/RCCar_MAVLINK_autopilot /usr/local/bin/
COPY --from=builder /build/examples/build/RCCar_ISO22133_autopilot/RCCar_ISO22133_autopilot /usr/local/bin/
COPY --from=builder /build/examples/build/map_local_twocars/map_local_twocars /usr/local/bin/
COPY --from=builder /build/examples/build/RCCar_ISO22133_autopilot/isoObject/*.so /usr/local/lib/
COPY --from=builder /build/examples/build/RCCar_ISO22133_autopilot/isoObject/iso22133/*.so /usr/local/lib/

# Update library cache
RUN ldconfig

# Create non-root user for running the application
RUN useradd -m -s /bin/bash waywise
USER waywise
WORKDIR /home/waywise

# Set default command to the main MAVLINK autopilot example
CMD ["/usr/local/bin/RCCar_MAVLINK_autopilot"]

# Expose MAVLINK default ports
# UDP: 14540 (system), 14550 (ground control)
EXPOSE 14540/udp 14550/udp

# Labels for container metadata
LABEL org.opencontainers.image.title="WayWise"
LABEL org.opencontainers.image.description="WayWise - A rapid prototyping library for connected, autonomous vehicles"
LABEL org.opencontainers.image.source="https://github.com/RISE-Dependable-Transport-Systems/WayWise"
LABEL org.opencontainers.image.vendor="RISE Research Institutes of Sweden"
LABEL org.opencontainers.image.licenses="GPL-3.0"
