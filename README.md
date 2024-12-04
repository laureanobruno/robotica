# Robotica

## Compilación
Si se está usando Gazebo versión Harmonic:
`export GZ_VERSION=harmonic`

Desde carpeta ROSberta:
```
source /opt/ros/{ROS-DISTRO}/setup.bash
colcon build
source install/setup.bash
```

## Ejecución
Nodos:
```
ros2 run robot_master wall_follower
ros2 run coverage coverage_server
```

Simulación en Gazebo:
```
ros2 launch ros_gz_example_bringup diff_drive.launch.py
```
