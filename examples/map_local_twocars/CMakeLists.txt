cmake_minimum_required(VERSION 3.5)

project(map_local_twocars LANGUAGES CXX)

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

find_package(Qt5 COMPONENTS Widgets Core Network PrintSupport REQUIRED)

set(WAYWISE_PATH ../..)

add_executable(map_local_twocars
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    ${WAYWISE_PATH}/vehicles/objectstate.cpp
    ${WAYWISE_PATH}/vehicles/vehiclestate.cpp
    ${WAYWISE_PATH}/vehicles/carstate.cpp
    ${WAYWISE_PATH}/autopilot/waypointfollower.h
    ${WAYWISE_PATH}/vehicles/controller/movementcontroller.cpp
    ${WAYWISE_PATH}/sensors/camera/gimbal.h
    ${WAYWISE_PATH}/communication/vehicleconnections/vehicleconnection.cpp
    ${WAYWISE_PATH}/communication/parameterserver.cpp
    ${WAYWISE_PATH}/autopilot/purepursuitwaypointfollower.cpp
    ${WAYWISE_PATH}/autopilot/followpoint.cpp
    ${WAYWISE_PATH}/core/coordinatetransforms.h
    ${WAYWISE_PATH}/userinterface/map/mapwidget.cpp
    ${WAYWISE_PATH}/userinterface/map/osmclient.cpp
    ${WAYWISE_PATH}/userinterface/map/osmtile.cpp
    ${WAYWISE_PATH}/core/pospoint.cpp
    ${WAYWISE_PATH}/vehicles/controller/carmovementcontroller.cpp
    ${WAYWISE_PATH}/vehicles/controller/motorcontroller.h
    ${WAYWISE_PATH}/vehicles/controller/movementcontroller.cpp
    ${WAYWISE_PATH}/vehicles/controller/servocontroller.cpp
    ${WAYWISE_PATH}/routeplanning/zigzagroutegenerator.cpp
    ${WAYWISE_PATH}/routeplanning/zigzagroutegenerator.h
)

target_include_directories(map_local_twocars PRIVATE ${WAYWISE_PATH}/)

target_link_libraries(map_local_twocars
    PRIVATE Qt5::Widgets
    PRIVATE Qt5::Network
    PRIVATE Qt5::PrintSupport
)
