# BRAZO-ROBOTICO

COnexion del ROS con ESP32
Instálalo desde los paquetes binarios (más rápido):

sudo apt update
sudo apt install ros-humble-micro-ros-agent


(Otra vez, cambia humble por tu versión de ROS.)

Después de eso:

source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

🧱 3. Si lo compilaste desde fuente (microros_ws)

Asegúrate de haberlo compilado correctamente y de tenerlo en tu workspace:

cd ~/PROYECTO_ROBOTICA/microros_ws
colcon build
source install/setup.bash


Luego prueba:

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0


## MANDAR UNA POSICON
ros2 topic pub -1 /cmd_xyz_position geometry_msgs/msg/Point "{x: 0.6, y: 0.0, z: 0.7}"
Una sola vez
