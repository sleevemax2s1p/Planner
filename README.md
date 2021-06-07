# Planner
run on **ubuntu18.04,16.04**
# How to start
```
mkdir -p course/src
cd sourse/src
git clone https://github.com/sleevemax2s1p/Planner.git
cd ..
catkin_make
source devel/setup.bash
```
# How to install jps3d
```
**Required:**

* Eigen3
* yaml-cpp

Simply run following commands to install dependancy:
* $ sudo apt update
* $ sudo apt install -y libeigen3-dev libyaml-cpp-dev libboost-dev cmake

Simple cmake
* $ mkdir build && cd build && cmake .. && make -j4

```

# How to launch
```
roslaunch planner planner.launch
``` 
