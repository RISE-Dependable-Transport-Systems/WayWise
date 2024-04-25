# map_local_twocars
A minimal UI example showing a map and two cars driving in circles. No external communication is used.

A fully-featured implementation for controlling WayWise-based vehicles (MAVLINK communication) can be found at [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower).

## Installing Prerequisites (tested on Ubuntu 20.04/22.04) & Building

    sudo apt install git build-essential cmake qtbase5-dev
    mkdir build && cd build
    cmake ..
    make -j4
