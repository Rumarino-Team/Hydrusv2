#!/bin/bash
set -e

# Start roscore in the background
source devel/setup.bash
roscore &
sleep 2

# Compile the Arduino project
cd /root/Arduino/libraries/sensor_actuator_pkg/examples/Hydrus
arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn $ARDUINO_BOARD Hydrus.ino
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

# Keep the container running
exec "$@"
