#!/bin/bash
cd MAVSDK
docker run --rm mavsdk/mavsdk-dockcross-linux-arm64-custom > ./dockcross-linux-arm64-custom; chmod +x ./dockcross-linux-arm64-custom
./dockcross-linux-arm64-custom /bin/bash -c "cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=build/linux-arm64/install -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -DWERROR=OFF -Bbuild/linux-arm64 -H."
./dockcross-linux-arm64-custom cmake --build build/linux-arm64 -j5 --target install
./dockcross-linux-arm64-custom tools/create_packages.sh ./build/linux-arm64/install . arm64 libmavsdk-dev
mv *.deb ..
