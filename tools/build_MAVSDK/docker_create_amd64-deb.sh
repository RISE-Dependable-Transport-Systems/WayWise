#!/bin/bash
docker run -it --rm -v $(pwd):/home/user/MAVSDK:z -e LOCAL_USER_ID=`id -u` mavsdk/mavsdk-ubuntu-22.04 ./create_amd64-deb.sh
