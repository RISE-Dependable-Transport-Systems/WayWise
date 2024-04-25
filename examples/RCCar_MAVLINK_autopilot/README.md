MAVSDK 2.0 or newer is required and pre-built releases can be found at https://github.com/mavlink/MAVSDK/releases. 

## Installing Prerequisites (tested on Ubuntu 20.04/22.04) & Building
    # Installing MAVSDK
    sudo dpkg -i libmavsdk-dev*.deb

    sudo apt install git build-essential cmake libqt5serialport5-dev
    mkdir build && cd build
    cmake ..
    make -j4
