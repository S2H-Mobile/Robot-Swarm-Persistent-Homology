# Robot Swarm Persistent Homology
This project simulates a swarm of robots creating a map of its virtual environment. Each agent explores the space, records contact events, and helps build a graph that reveals the topology of the environment.

## Discover topology through teamwork

- **Brownian Motion Simulation:**  
  Robots wander randomly across the environment, interacting when they meet.
- **Contact Event Logging:**  
  Every robot encounter is recorded with time, location, and participants.
- **Topological Mapping:**  
  After the simulation, Betti numbers are computed from the resulting contact graph, unveiling the environment’s topological landscape.
- **Multiple Gazebo Worlds:**  
  Explore two environments: `basic.world` and `cafe.world` (default). Both feature open planes with obstacles.

![Gazebo screenshot.](./screenshot_gazebo.png)

## Quick Start

1. **Requirements:**  
   - ROS  
   - Gazebo  
   - x-term

2. **Build the Package:**  
   ```bash
   catkin_make
   ```

3. **Launch the Simulation:**  
    ```bash
     roslaunch topological_swarm simulation.launch
     ```
    Add `use_rviz:=1` to visualize the agent model in RViz.

## Output Example
When a run finishes, contact events are saved to `~/.ros/subscribe_to_contact.txt` as comma-separated values:

```
t: 659.117,	x: 0.305312,	y: 0.999596,	m: swarm_agent1,	o: swarm_agent3
```
- `t`: Timestamp  
- `x, y`: Event coordinates  
- `m`: Reporting agent  
- `o`: Other agent

## Want to Contribute?
Feedback and contributions are welcome! Open an issue or submit a pull request to help improve this project.
