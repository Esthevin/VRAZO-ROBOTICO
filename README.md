# BRAZO-ROBOTICO

COnexion del ROS con ESP32



## MANDAR UNA POSICON
ros2 topic pub -1 /cmd_xyz_position geometry_msgs/msg/Point "{x: 0.6, y: 0.0, z: 0.7}"
Una sola vez
