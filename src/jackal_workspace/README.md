# jackal_workspace ROS Package
Package for simulating multiple jackal rovers with attached to common payload using virtual cables. 

## Instructions to run simulation 

Each robot Spawned using the following launch files, will launch independent global and local planner. 

### Single robot with payload

![1](https://user-images.githubusercontent.com/5753164/134803688-7c0fd561-36da-4cb9-9d1a-85051953c649.png)

```bash
cd ~/decentralized-payload-transport
source devel/setup.bash
roslaunch jackal_workspace single_robot_simulation.launch
```

### Two robots with payload

![2](https://user-images.githubusercontent.com/5753164/134803718-2326a00b-7487-4f43-a180-1c4b0842c522.png)

```bash
cd ~/decentralized-payload-transport
source devel/setup.bash
roslaunch jackal_workspace two_robot_simulation.launch
```

### Three robots with payload

![3](https://user-images.githubusercontent.com/5753164/134803736-1abcc65d-cf43-4e1b-93a8-066d45d2b2d7.png)

```bash
cd ~/decentralized-payload-transport
source devel/setup.bash
roslaunch jackal_workspace three_robot_simulation.launch
```

## Virtual cable 
[URDF](https://github.com/scifiswapnil/decentralized-payload-transport/tree/master/src/jackal_workspace/urdf) model's for virtual cable.

| ![simrope](https://user-images.githubusercontent.com/5753164/134803562-66288775-6f36-42bf-a9dd-91540ca21921.png) |
|:--:| 
| *Virtual cable design* |
