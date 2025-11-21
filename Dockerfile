# Multi-stage Dockerfile for WayWise
# Optimized for C++ builds with minimal runtime image

# --- MAVSDK stage (install once, reuse everywhere) ---
FROM ubuntu:24.04 AS mavsdk
ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG MAVSDK_VER=2.14.1
ARG MAVSDK_SHA256_AMD64=794e2583aa60993623e6faa19dcb464dc776d05fd93b08dcc28293a16a3c1da3
ARG MAVSDK_SHA256_ARM64=47f02bd1fa86f48df28843d40125176f6b52108174f66f3f761a644773dde410

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates wget; \
    arch="${TARGETARCH:-$(dpkg --print-architecture)}"; \
    case "$arch" in \
    amd64)  DEB="libmavsdk-dev_${MAVSDK_VER}_ubuntu22.04_amd64.deb"; SUM="$MAVSDK_SHA256_AMD64" ;; \
    arm64)  DEB="libmavsdk-dev_${MAVSDK_VER}_debian12_arm64.deb";   SUM="$MAVSDK_SHA256_ARM64" ;; \
    *)      echo "Unsupported arch: $arch"; exit 1 ;; \
    esac; \
    url="https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VER}/${DEB}"; \
    wget -O /tmp/mavsdk.deb "$url"; \
    echo "${SUM}  /tmp/mavsdk.deb" | sha256sum -c -; \
    apt-get install -y --no-install-recommends /tmp/mavsdk.deb; \
    rm -rf /var/lib/apt/lists/* /tmp/mavsdk.deb

# --- Build stage ---
FROM ubuntu:24.04 AS builder

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ARG BUILD_TYPE=Release
ARG TARGETARCH

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    qtbase5-dev \
    libqt5serialport5-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    ninja-build \
    ca-certificates \
    wget \
    && rm -rf /var/lib/apt/lists/*


# Get MAVSDK headers & libs from the mavsdk stage (no repeated logic)
COPY --from=mavsdk /usr/ /usr/

# Set working directory
WORKDIR /build

# Copy source code
COPY . .

# Initialize submodules and build + install
RUN git submodule update --init --recursive \
    && cmake -S examples -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/waywise \
    -DCMAKE_INSTALL_RPATH="/opt/waywise/lib:/usr/local/lib" \
    -DCMAKE_INSTALL_DO_STRIP=ON \
    && cmake --build build -j$(nproc --ignore=1) \
    && cmake --install build

# Runtime stage
FROM ubuntu:24.04
ARG TARGETARCH

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies only
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5a \
    libqt5network5 \
    libqt5serialport5 \
    libqt5printsupport5 \
    libqt5widgets5 \
    libboost-program-options1.74.0 \
    libboost-system1.74.0 \
    libatomic1 \
    tini \
    && rm -rf /var/lib/apt/lists/*

# Reuse MAVSDK from the mavsdk stage (no repeated logic)
COPY --from=mavsdk /usr/ /usr/

# Copy preinstalled artifacts
COPY --from=builder /opt/waywise /opt/waywise

# Make binaries available in PATH
RUN ln -s /opt/waywise/bin/RCCar_MAVLINK_autopilot /usr/local/bin/RCCar_MAVLINK_autopilot \
    && ln -s /opt/waywise/bin/RCCar_ISO22133_autopilot /usr/local/bin/RCCar_ISO22133_autopilot \
    && ln -s /opt/waywise/bin/map_local_twocars /usr/local/bin/map_local_twocars

# Update library cache
RUN ldconfig

# Verify that the main binary exists and all dependencies are met
RUN test -x /opt/waywise/bin/RCCar_MAVLINK_autopilot \
    && ldd /opt/waywise/bin/RCCar_MAVLINK_autopilot | (! grep "not found")

# Create non-root user with stable UID/GID
ARG APP_UID=10001
ARG APP_GID=10001
RUN groupadd -g ${APP_GID} waywise && useradd -m -u ${APP_UID} -g ${APP_GID} -s /bin/bash waywise

# Simple liveness check (adjust as needed)
USER root
RUN printf '#!/bin/sh\nexec /usr/local/bin/RCCar_MAVLINK_autopilot >/dev/null 2>&1 || exit 1\n' \
    > /usr/local/bin/healthcheck && chmod +x /usr/local/bin/healthcheck

# RUN useradd -m -s /bin/bash waywise
USER waywise
WORKDIR /home/waywise

# Use tini as init; keep args overrideable via CMD
ENTRYPOINT ["/usr/bin/tini","--","/usr/local/bin/RCCar_MAVLINK_autopilot"]
CMD []
HEALTHCHECK --interval=30s --timeout=3s --retries=3 CMD /usr/local/bin/healthcheck
# Expose MAVLINK default ports
# UDP: 14540 (system), 14550 (ground control)
EXPOSE 14540/udp 14550/udp

# Labels for container metadata
LABEL org.opencontainers.image.title="WayWise"
LABEL org.opencontainers.image.description="WayWise - A rapid prototyping library for connected, autonomous vehicles"
LABEL org.opencontainers.image.source="https://github.com/RISE-Dependable-Transport-Systems/WayWise"
LABEL org.opencontainers.image.vendor="RISE Research Institutes of Sweden"
LABEL org.opencontainers.image.licenses="GPL-3.0"
