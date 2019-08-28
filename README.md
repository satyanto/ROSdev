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

