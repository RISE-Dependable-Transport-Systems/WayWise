![ww](https://user-images.githubusercontent.com/2404625/166413759-d1d2f771-984c-4dec-9036-866cf29dc547.png)
WayWise ([way-wise](https://en.wiktionary.org/wiki/way-wise): _Expert or knowledgeable in finding or keeping the way; knowing the way or route._) is a rapid prototyping library for connected, autonomous vehicles.
It is developed within research projects at the RISE Dependable Transport Systems group to investigate the use of autonomous vehicles (rc cars, tractors, drones) and, especially, related functional safety as well as cybersecurity risks and opportunities they bring, in various use cases (road traffic, agriculture, maritime).
It has roots in the [RISE Self-Driving Vehicle Platform (SDVP)](https://github.com/RISE-Dependable-Transport-Systems/rise_sdvp), originally developed by Benjamin Vedder, but was redesigned and rewritten based on C++ and Qt targeting commercial-of-the-shelf hardware (instead of custom hardware and firmware).
The library is used to build the brains on-vehicle and the control application counterpart on the desktop.
Vehicle and control application communicate with each other using the [MAVLINK protocol](https://mavlink.io/), which is implemented using [MAVSDK](http://mavsdk.io/).
WayWise does not target production readiness but exploring specific use cases by rapid prototyping.

Main authors are (firstname.lastname@ri.se):
- Marvin Damschen
- Rickard Häll
- Aria Mirzai

## How to use it and what to expect
This "library" is meant to be used as a git submodule, as we do not do releases (for the time being) and do not promise a stable API.
In general, our development resources are scarce and dedicated to fulfill use cases of research projects we are part of.
We do our best to avoid it, but things will break from time to time.

A minimal example of a simulated rc car with an autopilot can be found here: \
https://github.com/RISE-Dependable-Transport-Systems/RCCar_minimal_example \
This example can be connected to using [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower) and follow waypoints created on the map:

![RCCar_minimal_example](https://user-images.githubusercontent.com/2404625/202208555-1271ba0d-55f7-4c26-94ac-53920e6d18c5.gif)

A fully-fledged rc car implementation (GNSS & IMU-based positioning, [VESC](https://vesc-project.com/)-based odometry, ...) can be found here: \
https://github.com/RISE-Dependable-Transport-Systems/RCCar

### Current state
The biggest general construction sites are documentation and improving [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower).
ControlTower was built from scratch within the [LASH FIRE](https://lashfire.eu/) EU project and recently (#19) replaced [RControlStation](https://github.com/RISE-Dependable-Transport-Systems/rise_sdvp/tree/master/Linux/RControlStation) from SDVP times. 
ControlTower communicates to vehicles via MAVLINK, which enables support for drones running [PX4](https://px4.io/) additionally to WayWise-based vehicles.

## Organization
- **core**: fundamental classes/headers, e.g., for storing a position and transforming coordinates
- **communication**: quite broadly, everything that deals with "communication", e.g., connection between vehicle and control station, but also the vehicle's internal communication to lower-level control via CANopen
- **vehicles**: classes used to store state of different vehicle types (e.g., within desktop application), but also to implement a specific vehicle that can drive and steer using controllers
- **sensors**: sensor support in a broad sense. Support for GNSS, IMU and UWB-based positioning is implemented here as well as objectdetection using [DepthAI](https://docs.luxonis.com/en/latest/)
- **autopilot**: currently provides the "WaypointFollower" interface for following a route (i.e., a list of waypoints) and several implementations of it
  - PurePursuitWaypointFollower, an implementation of [pure pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf), which can also follow a point, e.g., a person or another vehicle
  - GotoWaypointFollower, does not do low-level control but sends goto requests (mainly used for PX4-based drones)
  - MultiWaypointFollower, is a container that allows switching between multiple instances of WaypointFollower (e.g., if you want to switch between different routes and save state inbetween)
- **userinterface**: UI building blocks to create desktop applications. Functional but still a lot of work-in-progress (see also [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower))
- **external**: code from external projects
- **legacy**: code to support funtionality from "SDVP times", e.g., to communicate with RControlStation. Not to be used in future projects as it will be removed sooner or later...

## Use cases
![image](https://user-images.githubusercontent.com/2404625/165902491-023a640b-947a-4a76-aea6-6219e5f8ca76.png)

