## Rover simulation package
This includes xacro files that describe low-res simulation 3D model

## Usage 
Open 3 terminals in ~/ros2_ws/

Terminal 1: 
```bash
# Build package
colcon build --symlink-install
# Launch node:
ros2 launch rover rsp.launch.py
```

Terminal 2: 
```bash
# Errors unless we run this, we haven't implemented actual joint_states yet.
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Terminal 3: 
```bash
# Finally, run RVIZ to visualize bot and sensors
rviz2
```

## RVIZ usage
Some settings may need to be set in RVIZ to see the bot:
Dropdown boxes:
 - Global Options > Fixed Frame = base_link
 - Add > RobotModel > Description Topic > /robot_description