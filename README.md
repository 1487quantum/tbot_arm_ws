# tbot_arm_ws
Packages used to test out the *Poke Arm* model on turtlebot. (Includes the modified *turtlebot_description* file)

## Dependancies
This workspace requires the *turtlebot_simulator* to be installed.
```
$ sudo apt-get install ros-indigo-turtlebot-simulator
```

## Usage
Git clone this repository into an existing workspace or create another workspace.
```
$ git clone https://github.com/1487quantum/tbot_arm_ws.git
```
After that, git clone the *poke-arm-gazebo* repo. This contains the arm model and the controls.
```
$ git clone https://github.com/1487quantum/poke-arm-gazebo.git
```
To launch the turtlebot & arm in Gazebo, roslaunch *turtlebot_world.launch* in the *actrl/launch* directory.
```
$ roslaunch actrl turtlebot_world.launch
```

## Turtlebot Arm Control Dashboard
Similar to Turtlebot *keyboard_teleop*, the *Turtlebot Arm Control Dashboard* is a simple control dasboard to allow one to control both the arm and turtlebot all in one *keyboard_teleop*. (Written in Python2.7) To use the dashboard,
```
$ rosrun actrl mainControl
```
OR
```
$ python mainControl.py
```
