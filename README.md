# Robot-Swarm-Persistent-Homology
A simulated swarm of robots creates a topological map of its environment. The robotic agents are represented by white spheres in the Gazebo environment. The agents move freely in the x-y-plane. They are subject to random pushes, so they perform a Brownian motion.

The location and timestamp of each contact between agents is recorded. When a simulation run is finished, a graph is constructed from the contact event database (TBD), and the Betti numbers of the corresponding simplicial complex are computed (TBD).

The project contains two Gazebo environments, `basic.world` and `cafe.world` (see screenshot below). Both environments are conntected but not simply connected. The basic world is launched by default.

![Gazebo screenshot.](./screenshot_gazebo.png)

## Setup
Install ROS, Gazebo and x-term. Use `catkin_make` to build the ROS package.

## Usage
Start the simulation with the command

``` bash
$ roslaunch topological_swarm simulation.launch
```
Set the launch argument `use_rviz:=1` to visualize the agent model in RViz.

When a simulation run ends, the contact events between agents are saved as a plain text file at `~/.ros/subscribe_to_contact.txt`.
