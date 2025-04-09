# TaskWhiz-Demo

## Launch Turtlebot4 World

```bash
ros2 launch taskwhiz_worlds tb4_world.launch.py
```

## Generating Map 

```bash
ros2 launch taskwhiz_worlds tb4_world.launch.py
ros2 launch slam_toolbox online_async_launch.py
# ros2 run nav2_map_server map_saver_server
```
### Teleop
```bash
ros2 launch teleop_joystick teleop_joystick.launch.py
```

### Saving Map
```bash
ros2 run nav2_map_server map_saver_cli -f <map_url>
```