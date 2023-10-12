#!/bin/bash
cd MAVSDK
docker run --rm mavsdk/mavsdk-dockcross-linux-armv7-custom > ./dockcross-linux-armv7-custom; chmod +x ./dockcross-linux-armv7-custom
./dockcross-linux-armv7-custom /bin/bash -c "cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=build/linux-armv7/install -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -DWERROR=OFF -Bbuild/linux-armv7 -H."
./dockcross-linux-armv7-custom cmake --build build/linux-armv7 -j5 --target install
./dockcross-linux-armv7-custom tools/create_packages.sh ./build/linux-armv7/install . armv7 libmavsdk-dev
mv *.deb ..
