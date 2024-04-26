# RCCar_ISO22133_autopilot
A minimal example of an ackermann-style vehicle (rccar) communicating via [ISO/TS 22133](https://www.iso.org/standard/78970.html) and featuring an autopilot.
Use [ATOS](https://github.com/RI-SE/ATOS) to connect.

A fully-featured implementation for a physical rccar (communicating via MAVLINK) can be found at [RCCar](https://github.com/RISE-Dependable-Transport-Systems/RCCar)

## Installing Prerequisites (tested on Ubuntu 20.04/22.04) & Building
    # Clone submodules
    git submodule update --init --recursive isoObject

    sudo apt install git build-essential cmake libqt5serialport5-dev libboost-program-options-dev libboost-system-dev
    mkdir build && cd build
    cmake ..
    make -j4
