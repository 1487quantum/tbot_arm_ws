# MNIST Sphere
A sdf sphere model generator that overlay MNIST Digits on the spheres. (Models created are designed to be used in Gazebo)

## Installation
To install the MNIST spheres, copy the folders inside sph to *~/.gazebo/models* directory. If you wanna do it via the command line:
```
$ cp -r sph/* ~/.gazebo/models/
```
To use the sphere, simply start the Gazebo GUI and insert the sphere under the *insert* tab.

## Generating the spheres
The *ballN* directory contains the template for generating the spheres as seen in the *sph* directory. To rebuild the sphere with the updated configuration, simply remove the sph directory and run the *createSphere.sh* script.
```
$ chmod +x createSphere.sh
$ ./createSphere.sh
```
