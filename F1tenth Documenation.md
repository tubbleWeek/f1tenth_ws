

### Preliminary Setup
#### Networking
The router connected to the Jetson is called `TP-LINK_F87D9A` 

Jetson's IP: `192.168.1.202`

Jetson's netmask: `255.255.255.0`

You must set a IP address with the same first three sections of the Jetson's IP address

EX: `192.168.1.XXX`

Set your DNS to google:

`8.8.8.8` or `8.8.4.4`

Set gateway to: `192.168.1.10`

You should now be able to ping the Jetson from your computer

If you cannot it may be due to the firewall, you should disable it and retry pinging the Jetson


#### ROS on Host

The Host machine needs to have ROS2 Foxy and with it Rviz2 package. 

The f1tenth_ws on the Jetson is where all of the ROS nodes are stored, most of the coding should be done through ssh.

### Running Nodes

There are 3 main nodes that need to be launched for controllers to work on the F1tenth. Each of the nodes will output an error if they cannot be started. They should be launched in this order

#### Map Server
This is the nav2 map server package

To run:

```
ros2 run nav2_map_server map_server --ros-args --params-file /home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/map_server_params.yaml
```


The config file for the map server can be found in: `/home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/`

You must then run: 

```
ros2 run nav2_util lifecycle_bringup map_server
```

You should then check rviz2 to see if you can get the map topic visualization

The map server will post the map to the `/map` topic

**IMPORTANT** The rviz2 reliability policy for the map should be **Transient Local**

---

##### Issues
If you cannot see the map you should stop the map server node and retry from the beginning. It may take a few seconds ~30 before rviz2 picks up the map. It may also help to open rviz2 before the map_server. If you wait any longer than this the map is not being posted.

#### AMCL
This is the nav2 amcl package

To run:

```
ros2 run nav2_amcl amcl --ros-args --params-file /home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/amcl_params.yaml
```


The config file for the map server can be found in: `/home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/`

You must then run: 

```
ros2 run nav2_util lifecycle_bringup amcl
```


The amcl node should ask for a initial pose to be set. **DO NOT** set it now

#### F1TENTH Bringup
This is the main control system for the f1tenth.

To run:

```
ros2 launch f1tenth_stack bringup_launch.py
```

You should see the node spin up successfully. If not try rerunning. 

You should also see the amcl node connect to the lidar. It is normal for it to drop a few messages.

Now you should set the initial pose. 

To do:
1. Navigate to rviz2
2. Using the 2D pose estimate, place the pose estimate approximately where the robot is

You should now see the amcl node properly working, and if you visualize the `/odom` topic in rviz, it should follow where the robot is.

#### Other Considerations

You can now run other nodes

All self made nodes are in the `f1tenth_controllers` package

Some pre-coded nodes are `follow_the_gap_node` and `pure_pursuit`

You can run these with:

```
ros2 run f1tenth_controllers <node_name>
```

If you are going to add more nodes make sure to edit the `setup.py` file


#### Waypoint Visualization

To visualize the waypoints you can run 

```
ros2 run pure_pursuit waypoint_visualizer --ros-args --params-file /home/nvidia/f1tenth_ws/src/pure_pursuit/config/config.yaml 
```

the config file for this is found in `/home/nvidia/f1tenth_ws/src/pure_pursuit/config`