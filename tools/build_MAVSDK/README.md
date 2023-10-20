# Simplified scripts to create *.deb files for MAVSDK
All scripts are based on info from [MAVSDK](https://github.com/mavlink/MAVSDK/blob/main/.github/workflows/main.yml).
create_amd64-deb.sh runs on your local machine, which means that the build dependencies for MAVSDK need to be installed including [fpm](https://fpm.readthedocs.io/en/stable/installation.html) (see [lines 13 to 44 here](https://github.com/mavlink/MAVSDK/blob/main/docker/Dockerfile-Ubuntu-22.04#L13)).
All other scripts (dock*) require only [docker to be installed](https://docs.docker.com/engine/install/ubuntu/).

1. start by cloning the MAVSDK version you want into this directory, e.g.,

   ````git clone --recursive git@github.com:mavlink/MAVSDK.git````

2. run one of the respective scripts inside this directory and you should obtain a *.deb, e.g.,

   ````./docker_create_amd64-deb.sh```` (amd64, currently Ubuntu 22.04) or

   ````./dockcross_create_arm64-deb.sh```` (arm64, currently Debian 11)

