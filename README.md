# mycobot_test

Test libraries for ros interface of the real myCobot to execute moveit

## Dependencies

- Required:
  - ros (noetic)
  - [serial library](https://github.com/wjwwood/serial) (by wjwwood)
  
## Installation

1. Get the codes:

```
cd ~/catkin_ws/src
git clone https://github.com/wjwwood/serial.git
git clone https://github.com/niiiiiiiiishiiiiiiiii/mycobot_test.git
```
2. Install the dependencies
```
rosdep install -i --from-paths .
```

3. Build the package

```
cd ~/catkin
catkin_make
```
## How to Use it

### myCobot Moveit

0. Please be sure to connect the machine myCobot to the port `/dev/ttyUSB0` and the port is writable (eg. set permission `666`). 
If the port is different, you should edit the parameter `mycobot_device_info/usb_port` in the file `mycobot_control/config/device_info.yaml`
1. run roscore 
```
roscore
```
2. run mycobot_controller (at another terminal)
```
roslaunch mycobot_controller mycobot_controller.launch
```
3. run moveit (and associated nodes at another terminal)
```
roslaunch mycobot_control mycobot_moveit.launch
```

## Licence

MIT License (You could ignore licenses in files package.xml)

https://opensource.org/licenses/mit-license.php
