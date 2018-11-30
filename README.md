# Robot-Swarm-Persistent-Homology
A simulated swarm of robots creates a topological map of its environment. The robotic agents are represented by white spheres in the Gazebo environment. The agents move freely in the x-y-plane. They are subject to random pushes, so they perform a Brownian motion.

![Gazebo screenshot.](./screenshot_gazebo.png)

## Setup
Install ROS, Gazebo and x-term. Use `catkin_make` to build the ROS package.

## Usage
Start the simulation with

``` bash
$ roslaunch topological_swarm world.launch
```
