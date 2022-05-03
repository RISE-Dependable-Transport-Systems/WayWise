![ww](https://user-images.githubusercontent.com/2404625/166413759-d1d2f771-984c-4dec-9036-866cf29dc547.png)
WayWise ([way-wise](https://en.wiktionary.org/wiki/way-wise): _Expert or knowledgeable in finding or keeping the way; knowing the way or route._) is a rapid prototyping library for connected, autonomous vehicles.
It is developed within research projects at the RISE Dependable Transport Systems group to investigate the use of autonomous vehicles (rc cars, tractors, drones) and, especially, related functional safety as well as cybersecurity risks and opportunities they bring, in various use cases (road traffic, agriculture, maritime).
It has roots in the [RISE Self-Driving Vehicle Platform (SDVP)](https://github.com/RISE-Dependable-Transport-Systems/rise_sdvp), originally developed by Benjamin Vedder, but was redesigned and rewritten based on C++ and Qt targeting commercial-of-the-shelf hardware (instead of custom hardware and firmware).
The library is used to build the brains on-vehicle and the control application counterpart on the desktop.
It does not target production readiness but exploring specific use cases by rapid prototyping.

Main authors are (firstname.lastname@ri.se):
- Marvin Damschen
- Rickard Häll

## How to use it and what to expect
This "library" is meant to be used as a git submodule, as we do not do releases (for the time being) and do not promise a stable API.
In general, our development resources are scarce and dedicated to fulfill use cases of research projects we are part of.
We do our best to avoid it, but things will break from time to time.

A minimal example of a simulated rc car with an autopilot can be found here: \
https://github.com/RISE-Dependable-Transport-Systems/RCCar_minimal_example
This example can be connected to by [RControlStation](https://github.com/RISE-Dependable-Transport-Systems/rise_sdvp/tree/master/Linux/RControlStation) (still used from SDVP, but to be replaced "soon") and follow waypoints created on the map:

![RCCar_minimal_example](https://user-images.githubusercontent.com/2404625/165896822-40313467-db43-4dab-aa87-d87f6867af92.gif)

A fully-fledged rc car implementation (GNSS & IMU-based positioning, [VESC](https://vesc-project.com/)-based odometry, ...) can be found here: \
https://github.com/RISE-Dependable-Transport-Systems/RCCar

### Current state
The biggest general construction sites are documentation and replacing RControlstation (with something built on this library), i.e., implementing the building blocks for UI-based desktop applications.
Within [LASH FIRE](https://lashfire.eu/), we are heavily working on drone support (desktop-side control application) based on [MAVSDK](https://mavsdk.mavlink.io/).

## Organization
- **core**: fundamental classes/headers, e.g., for storing a position and transforming coordinates
- **communication**: quite broadly, everything that deals with "communication", e.g., connection to a vehicle, but also the vehicle's internal communication to lower-level control via CANopen
- **vehicles**: classes used to store state of different vehicle types (e.g., within desktop application), but also to implement a specific vehicle that can drive and steer using controllers
- **sensors**: sensor support in a broad sense. Support for GNSS, IMU and UWB-based positioning is implemented here as well as objectdetection using [DepthAI](https://docs.luxonis.com/en/latest/)
- **autopilot**: currently only "WaypointFollower" an implementation of [pure pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) for following a route (i.e., a list of waypoints). It can also follow a point, e.g., a person or other vehicle
- **userinterface**: UI building blocks to create desktop applications
- **external**: code from external projects
- **legacy**: code to support funtionality from "SDVP times", e.g., to communicate with RControlStation. Will be removed sooner or later...

## Use cases
![image](https://user-images.githubusercontent.com/2404625/165902491-023a640b-947a-4a76-aea6-6219e5f8ca76.png)

