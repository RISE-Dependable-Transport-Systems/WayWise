cmake_minimum_required(VERSION 3.5)

project(RCCar_MAVLINK_autopilot LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Qt5 COMPONENTS Core Network SerialPort REQUIRED)
find_package(MAVSDK REQUIRED)

set(WAYWISE_PATH ../..)

add_executable(RCCar_MAVLINK_autopilot
    main.cpp
    ${WAYWISE_PATH}/core/simplewatchdog.cpp
    ${WAYWISE_PATH}/vehicles/objectstate.cpp
    ${WAYWISE_PATH}/vehicles/carstate.cpp
    ${WAYWISE_PATH}/core/vbytearray.cpp
    ${WAYWISE_PATH}/vehicles/controller/motorcontroller.h
    ${WAYWISE_PATH}/vehicles/controller/movementcontroller.cpp
    ${WAYWISE_PATH}/vehicles/controller/servocontroller.cpp
    ${WAYWISE_PATH}/vehicles/vehiclestate.cpp
    ${WAYWISE_PATH}/core/pospoint.cpp
    ${WAYWISE_PATH}/vehicles/controller/carmovementcontroller.cpp
    ${WAYWISE_PATH}/sensors/gnss/ublox.cpp
    ${WAYWISE_PATH}/sensors/gnss/rtcm3_simple.cpp
    ${WAYWISE_PATH}/sensors/gnss/ubloxrover.cpp
    ${WAYWISE_PATH}/sensors/gnss/gnssreceiver.cpp
    ${WAYWISE_PATH}/sensors/camera/gimbal.h
    ${WAYWISE_PATH}/communication/vehicleconnections/vehicleconnection.cpp
    ${WAYWISE_PATH}/communication/mavlinkparameterserver.cpp
    ${WAYWISE_PATH}/communication/parameterserver.cpp
    ${WAYWISE_PATH}/autopilot/waypointfollower.h
    ${WAYWISE_PATH}/autopilot/purepursuitwaypointfollower.cpp
    ${WAYWISE_PATH}/autopilot/followpoint.cpp
    ${WAYWISE_PATH}/communication/vehicleserver.h
    ${WAYWISE_PATH}/communication/mavsdkvehicleserver.cpp
    ${WAYWISE_PATH}/logger/logger.cpp
)

target_include_directories(RCCar_MAVLINK_autopilot PRIVATE ${WAYWISE_PATH})

target_link_libraries(RCCar_MAVLINK_autopilot
    PRIVATE Qt5::Network
    PRIVATE Qt5::SerialPort
    PRIVATE MAVSDK::mavsdk
)
