# Instructions

## Build workspace with Python 3
```bash
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```


## Python 3 dependencies
```bash
pip3 install pyquaternion
```

### Installing dependencies with wstool on a catkin workspace
```bash
mkdir -p ~/demo_ws/src; cd ~/demo_ws
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry --git https://github.com/ros/geometry -v melodic-devel
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel # do not know if it is essential (probably not). In doubt, just add it.
wstool set -y src/ros_numpy --git https://github.com/eric-wieser/ros_numpy.git -v master
wstool set -y src/navigation_final_semfire_pilot --git https://github.com/AfonsoEloy/navigation_final_semfire_pilot.git -v master
wstool up
rosdep install --from-paths src --ignore-src -y -r
```

