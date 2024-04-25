# RCCar_MAVLINK_autopilot
A minimal example of an ackermann-style vehicle (rccar) communicating via MAVLINK and featuring an autopilot.
Use [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower) to connect.

A fully-featured implementation for a physical rccar can be found at [RCCar](https://github.com/RISE-Dependable-Transport-Systems/RCCar)

## Installing Prerequisites (tested on Ubuntu 20.04/22.04) & Building
MAVSDK 2.0 or newer is required and pre-built releases can be found at https://github.com/mavlink/MAVSDK/releases. 

    # Installing MAVSDK
    sudo dpkg -i libmavsdk-dev*.deb

    sudo apt install git build-essential cmake libqt5serialport5-dev
    mkdir build && cd build
    cmake ..
    make -j4
