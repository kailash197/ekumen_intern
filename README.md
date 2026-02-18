
# Instructions

## Build the docker
To build the docker images, change to the workspace directory that contains Docker files and run:
```bash
cd ~/lio/wheel_odom_ws && sudo rm -rf build install log
cd ~/lio && docker compose -f docker/docker-compose.yml build
xhost +local:docker
cd ~/lio && docker compose -f docker/docker-compose.yml run --remove-orphans lio-dev
```

## Install dependencies
```bash
sudo apt update && rosdep install --from-paths src --ignore-src -y
```

### Get new shell:
```bash
docker exec -it $(docker ps -aq | head -1) bash
source install/setup.bash
```
OR
```bash
docker exec -it $(docker ps -aq | head -1) bash -c "source install/setup.bash; /usr/bin/bash"
```

## Package `lio_robot_description`

```bash
colcon build --packages-up-to lio_robot_description && source install/setup.bash
ros2 launch lio_robot_description lio_diffbot.launch.py
```

## Package `lio_robot_control`

1. To launch all controller nodes along with mock hardware
```bash
colcon build --packages-up-to lio_robot_control && source install/setup.bash
ros2 launch lio_robot_control controllers.launch.py use_sim_time:=false rviz:=true
```

2. To launch controller nodes without mock hardware
```bash
colcon build --packages-up-to lio_robot_control && source install/setup.bash
ros2 launch lio_robot_control controllers.launch.py use_sim_time:=true rviz:=true
```
Note: Flag `rviz` is used to enable/disable RViz2 visualization.

3. Move robot forward in a curve turning right
```bash
ros2 topic pub /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0, 3.0]}" --once
```


## Package: `lio_robot_gz`

### Run the Launch File
```bash
colcon build --packages-up-to lio_robot_gz && source install/setup.bash
ros2 launch lio_robot_gz lio_diffbot_gazebo_sim.launch.py
```


## Package `lio_robot_bringup`
- Generic
```bash
colcon build --packages-up-to lio_robot_bringup && source install/setup.bash
ros2 launch lio_robot_bringup lio_diffbot_bringup.launch.py use_sim_time:=true start_gazebo:=true rviz:=true
```

- To start simulation and Rviz2
```bash
colcon build --packages-up-to lio_robot_bringup && source install/setup.bash
ros2 launch lio_robot_bringup lio_diffbot_bringup_sim.launch.py
```

- To start mock hardware along with Rviz2
```bash
colcon build --packages-up-to lio_robot_bringup && source install/setup.bash
ros2 launch lio_robot_bringup lio_diffbot_bringup_mock.launch.py
```

## Package: `lio_robot_tools`

### Run the Launch File
```bash
docker exec -it $(docker ps -aq | head -1) bash -c "source install/setup.bash; /usr/bin/bash"
colcon build --packages-up-to lio_robot_tools && source install/setup.bash
ros2 launch lio_robot_tools twist_to_wheel_converter.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 param get /twist_converter wheel_radius
ros2 param get /twist_converter wheel_separation
ros2 param get /twist_converter wheel_vel_topic
ros2 param get /twist_converter cmd_vel_topic
```

## Package: `odometry_estimator`
This package contains the mathematics library and its tests.

### Build and Run tests
```bash
sudo apt update && rosdep install --from-paths src/odometry_estimator* --ignore-src -y
```
```bash
colcon build --packages-select odometry_estimator --event-handlers console_direct+ && source install/setup.bash
colcon test  --packages-select odometry_estimator --event-handlers console_direct+
colcon test-result --verbose
```

```bash
ros2 run odometry_estimator_test test_usage
```

#### Ament clang tidy setup
Add in CMakeLists.txt
```cmake
  find_package(ament_cmake_clang_tidy REQUIRED)
  set(CLANG_TIDY_CONFIG_PATH "${CMAKE_SOURCE_DIR}/../.clang-tidy" CACHE STRING "Choose the path to the clang tidy configuration file")
  if(EXISTS ${CLANG_TIDY_CONFIG_PATH})
    message(STATUS ".clang-tidy found")
    set(AMENT_CLANG_TIDY_CONFIG_FILE "${CLANG_TIDY_CONFIG_PATH}")
  else()
    message(WARNING ".clang-tidy not found at ${CLANG_TIDY_CONFIG_PATH}")
  endif()
```
in `package.xml`
```xml
 <test_depend>ament_cmake_clang_tidy</test_depend>
```
#### Build and run tests
```bash
clear && colcon build --packages-up-to odometry_estimator_test --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers console_direct+ && source install/setup.bash
clear &&  colcon test  --packages-up-to odometry_estimator_test --event-handlers console_direct+
```
```bash
ros2 run odometry_estimator_test test_usage
```

#### Auto format for clang issues
```bash
ament_clang_format --config src/.clang-format --reformat src/odometry_estimator
```
```bash
ament_clang_tidy --config src/.clang-tidy --fix-errors ./build/odometry_estimator
```


## Inspection and monitoring commands

```bash
### Check if using simulation time (Gazebo) vs real time
ros2 param get /controller_manager use_sim_time

### List all controllers and their states (active/inactive)
ros2 control list_controllers

### Check available hardware interfaces and which controllers claim them
ros2 control list_hardware_interfaces

### View current joint positions, velocities, efforts (one-time snapshot)
ros2 topic echo /joint_states --once

### Check dynamic joint states from hardware (raw interface values)
ros2 topic echo /dynamic_joint_states --once
```
