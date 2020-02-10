### What the meta repo is for:

This package is intended to simplify the task of setting up the closed-loop visual-inertial SLAM benchmarking framework (ICRA20).

We will use `wstool` to manage the relevant packages. `closedLoopBench.rosinstall` defines the core set of catkin packages required by the closed-loop navigation benchmarking framework. 


### Collect and build all packages required

__Note: you must have your ssh keys set up for this to work properly!__ Follow [this tutorial](https://help.github.com/articles/connecting-to-github-with-ssh/) to setup your ssh keys with github. 

Following is a general series of steps to accomplish the most common tasks with `wstool`.

1. Install the `wstool`:
```
sudo apt-get install python-rosdep  python-wstool  build-essential python-rosinstall-generator python-rosinstall
```

Note: All following commands should be run from the root of your catkin workspace!

2. Initialize the `wstool` workspace:
```
wstool init src
```

3. Add packages from a `.rosinstall` file (ex. `pips.rosinstall`) to your catkin workspace:
```
wstool merge -t src <path to rosinstall file>
```

4. Download/update all packages managed by wstool:
```
wstool update -t src -j20
rosdep install --from-paths src -i -y
```

5. Compile your workspace as usual with `catkin_make` or `catkin build`


### Set up the simulator

Refer to the README.md at 	
```
https://github.com/ivalab/gazebo_turtlebot_simulator
```

### VI-SLAM systems

By default GF+MSF is included in the meta repo.  Other VI-SLAM systems that has been plugged into the framework include:
1. msckf_vio 
```
git clone https://github.com/YipuZhao/msckf_vio.git
```
2. VINS-Fusion
```
git clone https://github.com/YipuZhao/VINS-Fusion.git
```
3. SVO
```
git clone https://github.com/YipuZhao/rpg_svo_example
```
It also requires the svo binary, which can be downloaded via http://rpg.ifi.uzh.ch/svo2.html


### Benchmarking outcome

We have been testing visual-inertial SLAM extensively with the closed-loop benchmarking framework.  A set of examplar evaluation plots are provided:

	https://github.com/ivalab/FullResults_ClosedNav

More details about the closed-loop benchmarking framework can be found at our ICRA20 paper (which has an open-access version as well):

	@article{zhao2020closedLoop,
	  title={Closed-Loop Benchmarking of Stereo Visual-Inertial SLAM Systems: Understanding the Impact of Drift and Latency on Tracking Accuracy},
	  author={Zhao, Yipu and Smith, Justin S. and Karumanchi, Sambhu H. and Vela, Patricio A.},
	  journal={IEEE Conference on Robotics and Automation},
	  year={2020}
	}
