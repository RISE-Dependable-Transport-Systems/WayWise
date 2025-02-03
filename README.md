# WayWise
![Workflow build result](https://github.com/RISE-Dependable-Transport-Systems/WayWise/actions/workflows/main.yml/badge.svg)

![ww](https://user-images.githubusercontent.com/2404625/166413759-d1d2f771-984c-4dec-9036-866cf29dc547.png)

WayWise ([way-wise](https://en.wiktionary.org/wiki/way-wise): _Expert or knowledgeable in finding or keeping the way; knowing the way or route._) is a rapid prototyping library for connected, autonomous vehicles.
It is developed within research projects at the RISE Dependable Transport Systems group to investigate the use of autonomous vehicles (rc cars, tractors, drones) and, especially, related functional safety as well as cybersecurity risks and opportunities they bring, in various use cases (road traffic, agriculture, maritime).
It has roots in the [RISE Self-Driving Vehicle Platform (SDVP)](https://github.com/RISE-Dependable-Transport-Systems/rise_sdvp), originally developed by Benjamin Vedder, but was redesigned and rewritten based on C++ and Qt targeting commercial-of-the-shelf hardware (instead of custom hardware and firmware).
The library is used to build the brains on-vehicle and the control application counterpart on the desktop.
The main protocol for communication between vehicle and control application is [MAVLINK](https://mavlink.io/), which is implemented using [MAVSDK](http://mavsdk.io/).
There is also support for controlling vehicles using [ISO/TS 22133:2023](https://www.iso.org/standard/78970.html) as implemented in [RI-SE/iso22133](https://github.com/RI-SE/iso22133).
WayWise does not target production readiness but exploring specific use cases by rapid prototyping.

Our main projects that base on WayWise are:
- [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower), desktop control application that communicates to vehicles via MAVLINK (WayWise-based vehicles mainly, experimental [PX4](https://px4.io/) support).
- [RCCar](https://github.com/RISE-Dependable-Transport-Systems/RCCar), a fully-fledged rc car implementation (GNSS & IMU-based positioning, [VESC](https://vesc-project.com/)-based odometry, ...).
- [WayWiseR](https://github.com/RISE-Dependable-Transport-Systems/WayWiseR), a ROS2 integration of WayWise that streamlines the development and deployment of autonomous vehicles in both simulation and real-world environments.

Current maintainers are:
- Marvin Damschen
- Aria Mirzai
- Ramana Reddy Avula

Previous maintainers:
- Rickard Häll

You can reach out to us directly or via waywise@ri.se.

## How to use it and what to expect
This "library" is meant to be used as a git submodule, as we do not do releases (for the time being) and do not promise a stable API.
In general, our development resources are scarce and dedicated to fulfill use cases of research projects we are part of.
We do our best to avoid it, but things will break from time to time.

As set of examples can be found in [/examples](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/examples), most notably:
- [A minimal simulated car with an autopilot communicating via MAVLINK](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/examples/RCCar_MAVLINK_autopilot). This example can be connected to using [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower) and follow waypoints created on the map (see screen recording below).
- [A minimal simulated car with an autopilot communicating via ISO/TS 22133](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/examples/RCCar_ISO22133_autopilot). This example can be connected to using [ATOS](https://github.com/RI-SE/ATOS).

![RCCar_minimal_example](https://user-images.githubusercontent.com/2404625/202208555-1271ba0d-55f7-4c26-94ac-53920e6d18c5.gif)

## Organization
- **core**: fundamental classes/headers, e.g., for storing a position and transforming coordinates
- **communication**: quite broadly, everything that deals with "communication", e.g., connection between vehicle and control station, but also the vehicle's internal communication to lower-level control via CANopen
- **logger**: logging functionallity between vehicles and control application
- **vehicles**: classes used to store state of different vehicle types (e.g., within desktop application), but also to implement a specific vehicle that can drive and steer using controllers
- **sensors**: sensor support in a broad sense. Support for GNSS, IMU and UWB-based positioning is implemented here as well as objectdetection using [DepthAI](https://docs.luxonis.com/en/latest/)
- **autopilot**: currently provides the "WaypointFollower" interface for following a route (i.e., a list of waypoints) and several implementations of it
  - PurePursuitWaypointFollower, an implementation of [pure pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf), which can also follow a point, e.g., a person or another vehicle
  - GotoWaypointFollower, does not do low-level control but sends goto requests (mainly used for PX4-based drones)
  - MultiWaypointFollower, is a container that allows switching between multiple instances of WaypointFollower (e.g., if you want to switch between different routes and save state inbetween)
- **userinterface**: UI building blocks to create desktop applications (see also [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower))
- **external**: code from external projects
- **tools**: tools that support WayWise development like for building MAVSDK
- **examples**: a set of examples showing how WayWise is used

## Use cases
![image](https://user-images.githubusercontent.com/2404625/165902491-023a640b-947a-4a76-aea6-6219e5f8ca76.png)

## Citation
If you use WayWise in a publication, please cite our [paper](https://doi.org/10.1016/j.simpa.2024.100682):

    @article{Damschen_WayWise_A_rapid_2024,
    author = {Damschen, Marvin and Häll, Rickard and Mirzai, Aria},
    doi = {10.1016/j.simpa.2024.100682},
    journal = {Software Impacts},
    month = sep,
    title = {{WayWise: A rapid prototyping library for connected, autonomous vehicles}},
    volume = {21},
    year = {2024}
    }

## Funded by
<img src="https://user-images.githubusercontent.com/2404625/202213271-a4006999-49d5-4e61-9f3d-867a469238d1.png" width="120" height="81" align="left" alt="EU logo" />
This project has received funding from the European Union’s Horizon 2020 and Horizon Europe research and innovation programmes, and the Digital Europe programme under grant agreement nº 814975, nº 101095835, 101069573 and nº 101100622. The results reflect only the authors' view and the Agency is not responsible
for any use that may be made of the information it contains.
