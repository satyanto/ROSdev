# ROSdev
ROSdev is a repository for all my ROS development. It's actually a full catkin workspace, but it has to be set up manually (otherwise it gets real messy)

## What you need:
- ROS kinetic/melodic running on Ubuntu
- Franka Emika's libfranka package installed
- Ubuntu needs to be running in real-time (real-time kernel)

## To set it up:
```
git clone https://github.com/Oreology/ROSdev
cd ROSdev
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```
## Update:
I've changed to a new build system using the new catkin build system (instead of catkin_make).
The reason was because of an error long ago I forgot about as it wasn't compiling/building properly using catkin make, but also because catkin build now offers each package its own CMakeList and builds each package in isolation. (Also each dependency now should be guaranteed to be built before a package is built, yay!)
SO - follow the set up procedure above but instead of using catkin make we'll use catkin build:
```
catkin build
source devel/setup.bash
```

Change the --rosdistro to kinetic if you are using kinetic.
Usually, the path to the libfranka build is: (assuming default settings)
```
-DFranka_DIR:PATH=libfranka/build
```

## Note to self (rosserial):
(idk why rosserial is in python, but ok)
The rosserial serial node needs to be open at a serial port in order to essentially act as a middle-man 'handshaker' between any external rosserial devices and the main ROS computer.
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
My Arduino is currently always connected to ttyACM0 but change it to the correct serial port.



## More Info:
https://frankaemika.github.io/docs
