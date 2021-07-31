# Multi-agent-payload-transport

**Aim**: To evalute and design a decentralized cooperative planning strategy for large payload transport in a swarm.

**Key assumptions:** 
- Each individual robot is able to perform mapping, localization and path planning.
- Each robot knows the goal location where the trolley is supposed to reach. 
- No form of communication between the robots.


## Problems:
- **Trolley detection problem** - use of ICP (iterative closest point) in the laser data observed to find the best fit position and orientation of the object 
- **Global planner** - RRT planner for individual robot for individual robot’s path planning. Problem arises when individual robot’s come up with different plans to reach a goal in the same map (possible solution is using a leader follower approach) 
- **Local planner** - Using APF to generate force fields around obstacles that are governed by obstacles distances, goal direction and robot kinematics 
- **Simulating ropes** - using a prismatic joint with mechanical damping acting as a spring that can completely close or extend to a certain length. 


## Todo
- [x] Multi jackal simulation setup
- [x] RRT implementation for global path planning 
- [x] APF planner for local path planning **in progress**
- [ ] trolley with simulated ropes
- [ ] trolley detection with reflective markers in simulation 
- [ ] global planning strategies 
- [ ] Path planning tests
- [ ] Benchmarking **to be disscussed**

## Miscellaneous tasks

- [ ] Safety control mode - velocities are generated based on obstacles detected by laser
- [ ] teleop mode - cloud monitoring of robots.


## FAQ's

- How do the robots share each other's and trolley's location?
*Ans:* The clearpath jackal is equipped with Velodyne VLP-16 lidar which can report objects with different reflective indexes (like the road signs). Then the problem can be answered using intensity based region extraction and point cloud registration (super4PCS) based object detection (position and orientation). We can stick safety reflective tapes on the jackal robots and trolley as markers.  

- How is the local path computed with respect to the trolley for each robot?
*Ans:* Using artificial potential fields and knowing the location of the trolley/other robots in the team, we can generate a force field around the trolley towards the global path making the robots drive towards the goal and use the other robots location to keep a safe distance between each other. 

- How is the global path computed and shared between the robots?
*Ans:* Assuming the trolley's footprint as a basis each individual robot's global planner should ideally generate the same path (given both the robots have the same map of the environment). Alternatively, in a random (or using a heuristics like the robots heading), individual robots can discard plans that drive them in other directions. Some more probable solutions are yet to be determined. 

- Fault detection?
*Ans:* Keeping the rope's distance constant between the robot and trolley, if the robot's linear and angular velocity with the position of the trolley does not move the trolley, the robot can move away from the formation. 

- Fault recovery ?
*Ans:* Since the robot which is disconnected from the trolley, has moved away from the (APF region) formation after fault detection, the potential fields should ideally neglect the robot and other robots can still drive the trolley around. 
