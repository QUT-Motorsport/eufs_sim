# eufs_sim

## What is eufs_sim?

eufs_sim is an open source Formula Student autonomous vehicle simulation. It makes use of the [Gazebo](http://gazebosim.org/) simulator
to allow teams to test their AI software on preset [rule compliant](https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/fs-ai-dynamic-events-setup-and-cones-specification.pdf?sfvrsn=2)
tracks which are part of the dynamic events at the Formula Student UK (FSUK) competition. Additionally, to prepare for 'blind' dynamic
events such as [autocross or track drive](https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/fs-2021-autonomous-rules-v1-3.pdf?sfvrsn=2),
users can try their software on [randomly generated tracks](./eufs_tracks/README.md). Teams can adjust or even add to the [existing vehicle models](./eufs_models/README.md) to see their own car complete
events. eufs_sim is also highly configurable thanks to our [launcher](./eufs_launcher/README.md), allowing for selection of vehicle models, weather conditions, command modes and more!
In summary, teams can attempt the full competition, with their own vehicle model, in simulation, long before competition starts!

![simulation](https://gitlab.com/eufs/eufs_sim/-/wikis/uploads/e28a8de44a000dbd1ea427b66928d95c/GazeboActionShot.png)

## Requirements

- Ubuntu 20.04 LTS
- [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html)
- [eufs_msgs](https://gitlab.com/eufs/eufs_msgs)

_Note_: eufs_sim will probably work with other versions of ROS 2 (e.g Foxy) but they are not actively supported by the maintainers.

## Wiki
EUFS have created a number of wiki pages covering basic "How to" tutorials, "Quick fixes", package design and more!
This serves as an additional resource to help you understand and use eufs_sim.
[This link](https://gitlab.com/eufs/eufs_sim/-/wikis/home) will take you to the wiki home page.
A design overview can be found [here](https://gitlab.com/eufs/eufs_sim/-/wikis/Design-Overview).
Finally, a series of quick fixes for eufs_sim are listed [on this page](https://gitlab.com/eufs/eufs_sim/-/wikis/Quick-Fixes).

## Getting Started

For installation, setup, and initial launching, refer to the [Getting Started Guide](https://gitlab.com/eufs/eufs_sim/-/wikis/Getting-Started-Guide) on the Wiki.

## Packages

This project contains a number of packages. The package READMEs supply information about the package API (launch parameters, ROS 2 publishers, subscribers and services).
For information on the package design and usage guides see the [eufs_sim wiki](https://gitlab.com/eufs/eufs_sim/-/wikis/home).

- [eufs_racecar](./eufs_racecar/README.md) : launch, resource and urdf files for the simulated vehicle.
- [eufs_launcher](./eufs_launcher/README.md) : configures and launches the simulation.
- [eufs_models](./eufs_models/README.md) : vehicle physics library.
- [eufs_plugins](./eufs_plugins/README.md) : Gazebo plugins.
- [eufs_tracks](./eufs_tracks/README.md) : track generator and resource files for the track.
- [eufs_sensors](./eufs_sensors/README.md) : sensor mesh and urdf files.
- [eufs_rqt](./eufs_rqt/README.md) : rqt GUI's for eufs_sim (currently mission control and robot steering).
