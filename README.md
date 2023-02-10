### Ubuntu 20.04 and ROS noetic

Following is a general series of steps to accomplish the most common tasks with `wstool`.

1. Install the `wstool`:
```
sudo apt-get install python3-rosdep  python3-wstool  build-essential python3-rosinstall-generator python3-rosinstall
```

```bash
# ros navigation stack
sudo apt-get install ros-noetic-navigation
````


Note: All following commands should be run from the root of your catkin workspace!

2. Navigate to catkin workspace and initialize the `wstool` workspace:
```
cd ~/catkin_ws/src && git clone https://github.com/ivalab/meta_ClosedLoopBench.git -b feature/ubuntu20.04
```

3. Initialize the `wstool` workspace:
```
cd ~/catkin_ws && wstool init src
```

4. Add packages to your catkin workspace and build:

- 4.1 turtlebot and kobuki

	**Be sure to build the turtlebot package(4.1) first !!!**

```bash
wstool merge -t src src/meta_ClosedLoopBench/noetic_turtlebot.rosinstall

# download
wstool update -t src -j20
rosdep install --from-paths src -i -y

# build
catkin build -j2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
```

- 4.2 gazebo and slam package
```bash
# be sure to build the turtlebot package (4.1) first !!!
wstool merge -t src src/meta_ClosedLoopBench/closedLoopBenchUbuntu20.04.rosinstall

# download
wstool update -t src -j20
rosdep install --from-paths src -i -y

# build
catkin build -j2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
```

Ignore the complaint `turtlebot_trajectory_testing: Cannot locate rosdep definition for [common_rosdeps]`

5. Configure [gazebo_turtlebot_simulator](https://github.com/ivalab/gazebo_turtlebot_simulator/tree/feature/ubuntu20.04)

- 5.1. Please follow README.

### Known issues when installing:

- Q1: `/bin/sh: 1: pyrcc5: not found`

- A: `sudo apt-get install pyqt5-dev-tools`

- Q2:
```bash
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "xxx_xxx"
  with any of the following names:

    xxx_xxxConfig.cmake
    xxx_xxx-config.cmake
```
- A: `sudo apt-get install ros-noetic-xxx_xxx`

---
---
### What the meta repo is for:

This package is intended to simplify the task of setting up the closed-loop visual-inertial SLAM benchmarking framework (ICRA20).  Tested under Ubuntu 16.04 + ROS Kinetic.

We will use `wstool` to manage the relevant packages. `closedLoopBench.rosinstall` defines the core set of catkin packages required by the closed-loop navigation benchmarking framework. 


### Collect and build all packages required

__Note: you must have your ssh keys set up for this to work properly!__ Follow [this tutorial](https://help.github.com/articles/connecting-to-github-with-ssh/) to setup your ssh keys with github. 

Following is a general series of steps to accomplish the most common tasks with `wstool`.

1. Install the `wstool`:
```
sudo apt-get install python-rosdep  python-wstool  build-essential python-rosinstall-generator python-rosinstall
```

Note: All following commands should be run from the root of your catkin workspace!

2. Navigate to catkin workspace and initialize the `wstool` workspace:
```
cd ~/catkin_ws/src && git clone https://github.com/ivalab/meta_ClosedLoopBench.git
```

3. Initialize the `wstool` workspace:
```
cd ~/catkin_ws && wstool init src
```

4. Add packages from `closedLoopBench.rosinstall` to your catkin workspace:
```
wstool merge -t src src/meta_ClosedLoopBench/closedLoopBench.rosinstall
```

5. Download/update all packages managed by wstool:
```
wstool update -t src -j20
rosdep install --from-paths src -i -y
```
Ignore the complaint `turtlebot_trajectory_testing: Cannot locate rosdep definition for [common_rosdeps]`

6. Compile your workspace as usual with `catkin_make` or `catkin build`


### Set up the simulator

Refer to the README.md at 	
```
https://github.com/ivalab/gazebo_turtlebot_simulator
```

### VI-SLAM systems

By default GF+MSF is included in the meta repo.  Other VI-SLAM systems that has been plugged into the framework include:
1. VINS-Fusion (at the same catkin workspace `~/catkin_ws`)
```
cd ~/catkin_ws && git clone https://github.com/YipuZhao/VINS-Fusion.git
```
2. msckf_vio (assuming a seperate catkin workspace `~/msckf_ws` exists)
```
cd ~/msckf_ws/src && git clone https://github.com/YipuZhao/msckf_vio.git
```
3. SVO (assuming both `svo_install_ws` and `svo_install_overlay_ws` are properly set-up according to the instruction at http://rpg.ifi.uzh.ch/svo2.html)
```
git clone https://github.com/YipuZhao/rpg_svo_example
```
It also requires the svo binary, which can be downloaded via http://rpg.ifi.uzh.ch/svo2.html


### Benchmarking outcome

We have been testing visual-inertial SLAM extensively with the closed-loop benchmarking framework.  A set of exemplar evaluation plots are provided:

	https://github.com/ivalab/FullResults_ClosedNav

More details about the closed-loop benchmarking framework can be found at our ICRA20 paper (which has an open-access version as well):

	@article{zhao2020closedLoop,
	  title={Closed-Loop Benchmarking of Stereo Visual-Inertial SLAM Systems: Understanding the Impact of Drift and Latency on Tracking Accuracy},
	  author={Zhao, Yipu and Smith, Justin S. and Karumanchi, Sambhu H. and Vela, Patricio A.},
	  journal={IEEE Conference on Robotics and Automation},
	  year={2020}
	}
	
## Contact information

- Yipu Zhao		yipu.zhao@gatech.edu
- Justin S. Smith   jssmith@gatech.edu
- Patricio A. Vela	pvela@gatech.edu
