# EPSILON

## About

This is the project page of the paper "**EPSILON: An Efficient Planning System for Automated Vehicles in Highly Interactive Environments**". In this repo, we provide a simple and lightweight multi-agent simulator based on ROS and a demo implementation of the proposed EPSILON planning system.


If you use EPSILON for your academic research, please consider citing the follow

* Ding, Wenchao, et al. "EPSILON: An Efficient Planning System for Automated Vehicles in Highly Interactive Environments." IEEE Transactions on Robotics (2021).

***Paper:*** [IEEE Xplore](https://ieeexplore.ieee.org/document/9526613/), [arXiv](https://arxiv.org/abs/2108.07993)

***Demo video:*** [YouTube](https://youtu.be/3i0cIQrZs-4)

BibTex
```
@article{ding2021epsilon,
  title={EPSILON: An Efficient Planning System for Automated Vehicles in Highly Interactive Environments},
  author={Ding, Wenchao and Zhang, Lu and Chen, Jing and Shen, Shaojie},
  journal={IEEE Transactions on Robotics},
  year={2021},
  publisher={IEEE}
}
```

The following papers are also related:
* Ding, Wenchao, et al. "Safe trajectory generation for complex urban environments using spatio-temporal semantic corridor." IEEE Robotics and Automation Letters 4.3 (2019): 2997-3004. [(arXiv link)](https://arxiv.org/abs/1906.09788)
* Zhang, Lu, et al. "Efficient uncertainty-aware decision-making for automated driving using guided branching." 2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2020. [(arXiv link)](https://arxiv.org/abs/2003.02746)


If you have any question, please feel free to contact us via `lzhangbz@connect.ust.hk (Lu Zhang)` and `wdingae@connect.ust.hk (Wenchao Ding)`.


## Prerequisites
This project has been tested on Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic). For ROS installation, please refer to the official [website](http://wiki.ros.org/ROS/Installation).

### Denpendencies

* Install required packages
```
sudo apt-get install libgoogle-glog-dev libdw-dev libopenblas-dev gfortran
```

```
pip install empy pygame
```

#### Install OOQP
We use [OOQP](http://pages.cs.wisc.edu/~swright/ooqp/) for solving quadratic programming problems. Please refer to [link_1](https://github.com/emgertz/OOQP) and [link_2](http://pages.cs.wisc.edu/~swright/ooqp/) for the installation instruction.


#### Install Protobuf
We use [Protocol Buffers](https://developers.google.com/protocol-buffers/) for parameter configuration. For the installation guide, please refer to this [link](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md).


## Build on ROS
We recommend the users create an empty workspace. Clone the repo and build:
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/HKUST-Aerial-Robotics/EPSILON.git
  cd ..
  catkin_make
  source ~/${YOUR_WORKSPACE_PATH}/devel/setup.bash
```

## Just have a try!
1. Open a new terminal window and start roscore:
```
roscore
```

2. Launch RViz with `.rviz` file:
```
roscd phy_simulator/rviz/
rviz -d phy_simulator_planning.rviz
```

3. Launch the planner's node and AI nodes:
```
roslaunch planning_integrated test_ssc_with_eudm_ros.launch
roslaunch ai_agent_planner onlane_ai_agent.launch
```

4. Launch the simulator:
```
roslaunch phy_simulator phy_simulator_planning.launch
```
Note that the simulator should be launched last.

<p align="center">
  <img src="misc/demo_1.png" width = "600"/>
</p>


5. We provide a simple interface for controlling the behavior of the agents:
```
roscd aux_tools/src/
python terminal_server.py
```

<p align="center">
  <img src="misc/demo_2.png" width = "600"/>
</p>

You can select the target agent by clicking on the colored dots and change its behavior using `W-A-S-D` buttons.

## Acknowledgements
We would like to express sincere thanks to the authors of the following tools and packages:
* Lock-free queue: [moodycamel](https://github.com/cameron314/concurrentqueue)
* Json parser: [JSON for modern C++](https://github.com/nlohmann/json)
* KD-Tree library: [nanoflann](https://github.com/jlblancoc/nanoflann)
* 2D ray-casting: [roguelike_FOV](https://gist.github.com/zloedi/9551625)
* Quadratic programming: [OOQP](http://pages.cs.wisc.edu/~swright/ooqp/)
* Cubic spline fitting: [tk_spline](https://github.com/ttk592/spline)

## Licence

The source code is released under [MIT](https://opensource.org/licenses/MIT) license.


## Disclaimer

This is research code, it is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of merchantability or fitness for a particular purpose.