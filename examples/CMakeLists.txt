cmake_minimum_required(VERSION 3.5)
project(WayWise_examples)

add_subdirectory(RCCar_ISO22133_autopilot)
add_subdirectory(RCCar_MAVLINK_autopilot)
add_subdirectory(map_local_twocars)

if(TARGET RCCar_MAVLINK_autopilot)
    install(TARGETS RCCar_MAVLINK_autopilot
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
endif()

if(TARGET RCCar_ISO22133_autopilot)
    install(TARGETS RCCar_ISO22133_autopilot
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
endif()

if(TARGET map_local_twocars)
    install(TARGETS map_local_twocars
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
endif()
