8q0r1p.jpg
# Installation
Clone the repo to a ROS2 workspace
Remember to source the workspace
# Testing the driver
Run the code
```
ros2 launch arduino_driver arduino_driver.launch.py
```
Which will launch multiple ros2 nodes, specifically controller and hardware interface of the Arduino driver.
## Configuring the driver
The file `ros2_control/arduino_driver.ros2_control.xacro` contains much of the config parameters of the driver. For instance the name of each motor, the baud rate of the serial connection and which USB port to connect to, etc.

Change these parameters to get the connection to the Arduino up and running.
