# Actrl - Turtlebot + Arm controller
This package includes launch files to launch both the turtlebot and the arm together in Gazebo.

## Launching the world
To launch the world, simply roslaunch the *turtlebot_arm_world.launch* file:
```
$ roslaunch actrl turtlebot_arm_world.launch world_file:=$(find actrl)/worlds/fixed.world
```

## Changing the MNIST Number on the Sphere
To change the MNIST number on the sphere, open up the *.world* file and search for the the URI link with *ballN*, where *N* is the digit. Change *N* to whatever digit you want (0-9) and launch the file. (No need to recompile the file!)
