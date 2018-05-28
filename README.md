# Reference : https://github.com/duckietown/Software/tree/jukindle-devel-virtualjoy

Please install pygame before using this package if you don't have the package
to install pygame, please type sudo apt-get install python-pygame

To use the virtual-joystick, please run any .launch that you used to use joystick to control the robot
For example, duckiebot $ roslaunch duckietown joystick.launch veh:=<vehicle_name>
Then run these command on your duckietop

# Usage
after catkin_make,
duckietop $ source environment.sh
duckietop $ source set_ros_master.sh <vehicle_name>
duckietop $ roslaunch virtual_joystick virtual_joystick_node.launch veh:=<vehicle_name>

Then a window will pop up, then you can use the ARROW_KEYS to control it

# Commands


The following keys are supported:

| KEYS       | FUNCTION                             |
|------------|--------------------------------------|
| ARROW_KEYS | Steer your Duckiebot                 |
| q          | Quit the program                     |
| a          | Turn on line-following aka autopilot |
| s          | Stop line-following                  |
| i          | Toggle Anti-instagram                |
