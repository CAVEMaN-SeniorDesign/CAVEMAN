# CAVEMAN - Cave Autonomous Vehicle for Exploration, Mapping, and Navigation
Developed using ROS2 Foxy, using this image: 
```bash
docker pull 178669/sd_jetson:ros2foxy-latest
```

Clone into the ROS2 workspace:
```bash
git clone https://github.com/DZZLab/CAVEMAN/tree/main ~/ros2_ws/src
```

# Note: Main branch is for AMD64 development to allow Gazebo Simulations,
Jetson branch: https://github.com/CAVEMaN-SeniorDesign/CAVEMAN/tree/Jetson
Jetson only development is done for ARM64 in the following container:
```bash
docker pull 178669/sd_jetson:armJetson
```
The Jetson branch excludes Gazebo simulation packages and additionally builds a lot of packages from source for ARM compatibility. 
