## Ubuntu 20.04 and ROS noetic

Following is a general series of steps to accomplish the most common tasks with `wstool`.

### Install Dependencies.

__Note: you must have your ssh keys set up for this to work properly!__ Follow [this tutorial](https://help.github.com/articles/connecting-to-github-with-ssh/) to setup your ssh keys with github. 

1. ROS Tools.
```bash
# wstool
sudo apt-get install python3-rosdep  python3-wstool  build-essential python3-rosinstall-generator python3-rosinstall

# ros navigation stack
sudo apt-get install ros-noetic-navigation

# other ros packages
sudo apt install ros-noetic-kobuki-msgs ros-noetic-kobuki-dock-drive ros-noetic-kobuki-driver
sudo apt install ros-noetic-ecl-threads ros-noetic-ecl-geometry ros-noetic-ecl-streams
sudo apt install ros-noetic-joy

# qt
sudo apt install pyqt5-dev-tools
```

Note: All following commands should be run from the root of your catkin workspace!

2. Navigate to catkin workspace and clone the repo:
```bash
mkdir -p closedloop_ws/src
cd ~/closedloop_ws/src && git clone https://github.com/ivalab/meta_ClosedLoopBench.git -b feature/ubuntu20.04
```


3. Initialize the `wstool` workspace:
```bash
cd ~/closedloop_ws && wstool init src
```

4. Add packages to your catkin workspace and build:

- 4.1 (**Optional**)  turtlebot and kobuki

	**Please skip if ROS noetic turtlebot has been installed.**

	```bash
	wstool merge -t src src/meta_ClosedLoopBench/noetic_turtlebot.rosinstall

	# download
	wstool update -t src -j20
	rosdep install --from-paths src -i -y

	# build
	catkin build -j2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14
	```

- 4.2 gazebo and slam package
	```bash
	# be sure to build the turtlebot package (4.1) first !!!
	wstool merge -t src src/meta_ClosedLoopBench/closedLoopBenchUbuntu20.04.rosinstall

	# download
	wstool update -t src -j20
	rosdep install --from-paths src -i -y

	# build
	catkin build -j2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14
	```

	Ignore the complaint `turtlebot_trajectory_testing: Cannot locate rosdep definition for [common_rosdeps]`

### Build VI-SLAM Systems

**You don't have to build all of them. Simply choose those you would like to test.**

1. [GF-GG](https://github.com/ivalab/gf_orb_slam2/tree/feature/ubuntu20.04) (at the same catkin workspace `~/closedloop_ws`)
	```bash
	cd ~/closedloop_ws/src && https://github.com/ivalab/gf_orb_slam2.git -b feature/ubuntu20.04
	```
2. [DSOL](https://github.com/ivalab/dsol/tree/v1.0)(at the same catkin workspace `~/closedloop_ws`)

3. [ORB-SLAM2](https://github.com/ivalab/ORB_SLAM2/tree/v1.0) (at the same catkin workspace `~/closedloop_ws`)

4. [ORB-SLAM3](https://github.com/ivalab/ORB_SLAM3/tree/cl_tt_benchmark) (at the same catkin workspace `~/closedloop_ws`)

5. [SVO2.0](https://github.gatech.edu/RoboSLAM/rpg_svo_pro_open) (at a different catkin workspace `~/svo_ws`)

6. [MSCKF](https://github.gatech.edu/RoboSLAM/msckf_vio) (at a different catkin workspace `~/svo_ws`)

7. [VINS-Fusion](https://github.gatech.edu/RoboSLAM/VINS-Fusion) (at a different catkin workspace `~/svo_ws`)


### Set up the Simulator and Benchmarking

Refer to the [README](https://github.com/ivalab/gazebo_turtlebot_simulator/tree/feature/ubuntu20.04) at [gazebo_turtlebot_simulator](https://github.com/ivalab/gazebo_turtlebot_simulator/tree/feature/ubuntu20.04).


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

## Contact information
- Yanwei Du     yanwei.du@gatech.edu
- Yipu Zhao		yipu.zhao@gatech.edu
- Justin S. Smith   jssmith@gatech.edu
- Patricio A. Vela	pvela@gatech.edu
