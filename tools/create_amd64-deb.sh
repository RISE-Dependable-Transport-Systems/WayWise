#!/bin/bash
cd MAVSDK
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=install -DWERROR=OFF -Bbuild/release -H.
cmake --build build/release --target install -- -j5
tools/create_packages.sh ./install . amd64 libmavsdk-dev
mv *.deb ..
