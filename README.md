# ROSdev

ROSdev repo will be a full catkin workspace, but has to be set up manually.
Only the actual node code would be tracked (otherwise it gets real messy)

This catkin workspace is used for ROS versions: Melodic, Kinetic

Therefore, to set up:

``` 
git clone https://github.com/Oreology/ROSdev
cd ROSdev
catkin_make
```

Because only the actual node codes are tracked, we must build any dependencies the nodes require. We use rosdep for this:

Manually installing a package:
``` 
rosdep install [package]
```
OR:
At the top directory of our catkin workspace (ROSdev):
``` 
rosdep install --from-paths src --ignore-src -r -y
```

## franka_ros
*more information can be found at https://frankaemika.github.io/docs*

Since the lab computer runs on ROS Kinetic, franka_ros is built using ROS Kinetic version and all its dependencies:
(Requires libfranka to be installed first in computer)
```
sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros
```
OR:
```
sudo apt install ros-melodic-libfranka ros-melodic-franka-ros
```
(This will set up libfranka in your /home directory and set up franka_ros)

Before proceeding, make sure libfranka is installed, either from the ROS libraries or built from source.

We should also build franka_ros from source to our ROSdev catkin workspace so we can learn to make our own controller (and look into the files). To build from source into our /ROSdev/src directory:
```
cd src
git clone --recursive https://github.com/frankaemika/franka_ros
```
We are then going to install any required dependencies. Change 'kinetic' to 'melodic' if needed.
```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka
```
We then build the packages: (the paths will be different for each computer)
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```
We are then going to source our catkin workspace (from the root ROSdev directory):
```
source devel/setup.sh
```


