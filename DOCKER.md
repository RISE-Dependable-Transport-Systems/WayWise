# WayWise Docker Documentation 🐳

This document provides comprehensive information about using WayWise as a Docker container.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Image Details](#image-details)
- [Running Examples](#running-examples)
- [Networking](#networking)
- [Use Cases](#use-cases)
- [Building](#building)
- [Troubleshooting](#troubleshooting)

## Overview

WayWise is available as a containerized application optimized for rapid deployment and testing. The Docker image is built using a multi-stage build process that creates a minimal runtime image with all necessary dependencies.

### Key Features

- ✅ Multi-stage build for minimal image size
- ✅ No persistent volumes (stateless by design)
- ✅ Pre-built with all examples (MAVLINK, ISO22133, multi-car)
- ✅ Automatic builds via GitHub Actions
- ✅ Published to DockerHub

## Quick Start

### Pull from DockerHub

```bash
docker pull risedas/waywise:latest
```

### Run the Default Example

```bash
docker run --rm -p 14540:14540/udp -p 14550:14550/udp risedas/waywise:latest
```

This starts the MAVLINK autopilot example, which can be connected to using [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower).

## Image Details

### Base Image

- **Base**: Ubuntu 24.04
- **Architecture**: linux/amd64
- **Size**: ~500MB (runtime image)

### Included Dependencies

- Qt5 (Core, Network, SerialPort)
- MAVSDK 2.14.1
- Boost (program_options, system)

### Included Executables

1. `/usr/local/bin/RCCar_MAVLINK_autopilot` (default)
2. `/usr/local/bin/RCCar_ISO22133_autopilot`
3. `/usr/local/bin/map_local_twocars`

### User

The container runs as a non-root user `waywise` for security.

## Running Examples

### MAVLINK Autopilot (Default)

The MAVLINK autopilot example simulates an autonomous RC car with a pure pursuit controller:

```bash
docker run --rm --name waywise-mavlink \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  risedas/waywise:latest
```

**Connect with ControlTower:**

1. Open ControlTower
2. Connect to `udp://:14550`
3. Create waypoints on the map
4. Send the route to the vehicle

### ISO22133 Autopilot

Run the ISO/TS 22133-based autopilot:

```bash
docker run --rm --name waywise-iso \
  risedas/waywise:latest \
  /usr/local/bin/RCCar_ISO22133_autopilot
```

This example can be connected to using [ATOS](https://github.com/RI-SE/ATOS).

### Two-Car Simulation

Run a local simulation with two vehicles. **Note:** This example requires a display or headless mode:

```bash
# With X11 display forwarding (Linux/macOS)
docker run --rm --name waywise-twocars \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  risedas/waywise:latest \
  /usr/local/bin/map_local_twocars

# Headless mode (no display)
docker run --rm --name waywise-twocars \
  -e QT_QPA_PLATFORM=offscreen \
  risedas/waywise:latest \
  /usr/local/bin/map_local_twocars
```

### Background Mode

Run in detached mode:

```bash
docker run -d --name waywise-vehicle \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  risedas/waywise:latest
```

View logs:

```bash
docker logs -f waywise-vehicle
```

Stop the container:

```bash
docker stop waywise-vehicle
docker rm waywise-vehicle
```

## Networking

### Exposed Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14540 | UDP | MAVLINK system port |
| 14550 | UDP | Ground control station (ControlTower) |

### Connecting from Host

When running on the same machine as ControlTower:

```bash
docker run --rm \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  risedas/waywise:latest
```

Connect ControlTower to `udp://:14550`.

### Connecting from Remote Host

To allow connections from other machines:

```bash
docker run --rm \
  -p 0.0.0.0:14540:14540/udp \
  -p 0.0.0.0:14550:14550/udp \
  risedas/waywise:latest
```

Connect ControlTower to `udp://<host-ip>:14550`.

### Docker Network

For more complex setups, create a custom network:

```bash
docker network create waywise-net

docker run --rm --name waywise-vehicle \
  --network waywise-net \
  risedas/waywise:latest
```

## Use Cases

### Rapid Prototyping

Quick deployment for development and testing:

```bash
# Test a scenario
docker run --rm -p 14550:14550/udp risedas/waywise:latest

# No cleanup needed - container is removed on exit
```

### CI/CD Integration

Use in automated testing pipelines:

```yaml
# Example GitHub Actions step
- name: Test WayWise
  run: |
    docker run -d --name waywise-test \
      -p 14550:14550/udp \
      risedas/waywise:latest
    
    # Run your tests
    ./test-script.sh
    
    docker stop waywise-test
```

## Building

### Build from Source

Clone the repository with submodules:

```bash
git clone --recursive https://github.com/RISE-Dependable-Transport-Systems/WayWise.git
cd WayWise
```

Build the image:

```bash
docker build -t waywise:local .
```

The build process:

1. **Stage 0 (mavsdk)**: Downloads and installs MAVSDK binary
2. **Stage 1 (builder)**: Installs build dependencies, compiles examples
3. **Stage 2 (runtime)**: Creates minimal runtime image with only necessary libraries

### Build with Custom Tag

```bash
docker build -t waywise:v1.0.0 .
```

### Multi-platform Build

For ARM support (experimental):

```bash
docker buildx build --platform linux/amd64,linux/arm64 -t waywise:multiarch .
```

## Troubleshooting

### Container Exits Immediately

**Symptom**: Container starts and exits right away.

**Solution**: Check logs to see why the application exited:

```bash
docker logs waywise-vehicle
```

### Cannot Connect from ControlTower

**Symptom**: ControlTower cannot establish connection.

**Solutions**:

1. Verify ports are exposed: `docker ps` should show `0.0.0.0:14550->14550/udp`
2. Check firewall rules: Ensure UDP port 14550 is open
3. Use explicit IP binding: `-p 0.0.0.0:14550:14550/udp`

### Port Already in Use

**Symptom**: Error: `bind: address already in use`

**Solution**: Stop other containers or processes using the port:

```bash
docker ps | grep 14550
docker stop <container-id>
```

Or use a different port:

```bash
docker run --rm -p 14560:14550/udp risedas/waywise:latest
```

Then connect ControlTower to `udp://:14560`.

### SSL/Certificate Errors During Build

**Symptom**: Build fails with SSL certificate errors when downloading MAVSDK.

**Solution**: This is handled in the Dockerfile with `--no-check-certificate`. If you still encounter issues, check your network proxy settings.

### Out of Disk Space

**Symptom**: Build fails with "no space left on device"

**Solution**: Clean up Docker resources:

```bash
docker system prune -a --volumes
```

## Image Tags

Images are automatically built and tagged:

| Tag | Description | Update Frequency |
|-----|-------------|------------------|
| `latest` | Latest stable from main branch | On every main branch push |
| `main` | Current main branch | On every main branch push |
| `v*.*.*` | Release versions (e.g., v1.0.0) | On git tags |

Pull a specific version:

```bash
docker pull risedas/waywise:v1.0.0
```

## Advanced Usage

### Custom Entrypoint

Override the default command:

```bash
docker run --rm -it risedas/waywise:latest /bin/bash
```

This gives you a shell inside the container for debugging.

### Mount Configuration Files

If you need to provide custom configuration (not typically needed):

```bash
docker run --rm \
  -v $(pwd)/config:/config \
  -p 14550:14550/udp \
  risedas/waywise:latest
```

### Resource Limits

Limit CPU and memory:

```bash
docker run --rm \
  --cpus="2" \
  --memory="1g" \
  -p 14550:14550/udp \
  risedas/waywise:latest
```

## Contributing

If you find issues with the Docker image or have suggestions:

1. Open an issue on [GitHub](https://github.com/RISE-Dependable-Transport-Systems/WayWise/issues)
2. Submit a pull request with improvements
3. If needed, contact the maintainers at <firstname.lastname@ri.se>

## License

WayWise is licensed under the GPL-3.0 License. See [LICENSE](LICENSE) for details.
