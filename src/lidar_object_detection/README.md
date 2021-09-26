# lidar_object_detection

![4](https://user-images.githubusercontent.com/5753164/134803995-afd4bf44-cbec-40a0-a3ba-f97cffdef49a.png)

Instructions : 
```bash
cd ~/decentralized-payload-transport
source devel/setup.bash
rosrun lidar_object_detection detection 
```
**Note:** The ROS node uses rqt_reconfigure to allows dynamic reconfiguration of parameters. The node publishes `/agent_pose` and `/pallet_pose` topic, given the payload is marked with a 2x1.5meter refective marker and the other robot is marked with 5cm radius and 20cm height cylinder.

### Dependency
- The laser scan data should have a intensity channel in form of `sensor_msgs/LaserScan` message
- Remap the topic to `/velodyne_points` 

### Perception pipeline

![5](https://user-images.githubusercontent.com/5753164/134804173-47354393-f5b5-4879-a22f-00ccff796b9e.png)
