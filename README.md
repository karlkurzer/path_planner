### Hybrid A* Path Planner for the KTH Research Concept Vehicle [![Build Status](https://travis-ci.org/karlkurzer/path_planner.svg?branch=master)](https://travis-ci.org/karlkurzer/path_planner)

* [Characteristics](#characteristics)
* [Videos](#videos)
* [Images](#images)
* [Dependencies](#dependencies)
* [Setup](#setup)
* [Visualization](#visualization)
* [Citation](#citation)

The code in this repository is the result of my master's thesis which I have written at the Integrated Research Lab (ITRL) at KTH Royal Institute of Technology (2016).
The code is documented [here](https://karlkurzer.github.io/path_planner) and the associated thesis can be found [here](https://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-198534).


The goal of the thesis and hence this code is to create a real-time path planning algorithm for the nonholonomic Research Concept Vehicle (RCV). The algorithm uses a binary obstacle map as an input, generated using LIDAR mounted on top of the vehicle. The algorithm is being developed using C++ due to real-time requirements in combination with ROS to ensure modularity and portability as well as using RViz as a visualization/simulation environment.

##### <a name="characteristics"></a>Key Characteristics
* Sampling in continuous space with 72 different headings per cell (5° discretization)
* Constrained Heuristic - _nonholonomic without obstacles_
* Unconstrained Heuristic - _holonomic with obstacles_
* Dubin's Shot
* C++ real-time implementation (~10 Hz)

Large parts of the implementation are closely related to the hybrid A* algorithm developed by Dmitri Dolgov and Sebastian Thrun (_Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments_ DOI: 10.1177/0278364909359210)

##### <a name="videos"></a>Videos
* [Path Planning with Search Visualization](https://www.youtube.com/watch?v=1WZEQtg8ZZ4)
* [Dubin's Path - Constrained Heuristic](https://www.youtube.com/watch?v=VNo9fU6XEGE)
* [2D A* Search - Unconstrained Heuristic](https://www.youtube.com/watch?v=Ip2iUrVoFXc)
* [Open Loop Path Planning using Sensor Fusion](https://www.youtube.com/watch?v=GwIU00jukO4)

##### <a name="images"></a>Images
<img src="https://i.imgur.com/OICPCTB.png" alt="Reversing in a Maze" width="600"/>
<img src="https://i.imgur.com/ZiV9GDW.png" alt="Parking" width="600"/>
<img src="https://i.imgur.com/z7aT6lt.png" alt="Mitigating a U-shape Obstacle" width="600"/>

#### <a name="dependencies"></a>Dependencies
* [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
* [ros_map_server](https://wiki.ros.org/map_server)

#### <a name="setup"></a>Setup

Run the following command to clone, build, and launch the package (requires a sources ROS environment):

```
sudo apt install libompl-dev \
&& mkdir -p ~/catkin_ws/src \
&& cd ~/catkin_ws/src \
&& git clone https://github.com/karlkurzer/path_planner.git  \
&& cd .. \
&& catkin_make \
&& source devel/setup.bash \
&& rospack profile \
&& roslaunch hybrid_astar manual.launch
```
#### <a name="visualization"></a>Visualization (Rviz)
1. Add -> By Topic -> `/map`, `/path`, `/pathVehicle`, (`/visualizeNode2DPoses`)
2. Click 2D Pose Estimate to set a start point on the map (`p`)
3. Click 2D Nav Goal to set a goal point on the map (`g`)
4. Wait for the path being searched! (this process can be visualized [optional])

#### <a name="citation"></a>Citation
I would appreciate if you cite my work, in case you are using it for your work. Thank you :-)

```bibtex
@mastersthesis{Kurzer1057261,
	title        = {Path Planning in Unstructured Environments : A Real-time Hybrid A* Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle},
	author       = {Kurzer, Karl},
	year         = 2016,
	series       = {TRITA-AVE},
	number       = {2016:41},
	pages        = 63,
	issn         = {1651-7660},
	institution  = {KTH, Integrated Transport Research Lab, ITRL},
	school       = {KTH, Integrated Transport Research Lab, ITRL},
	abstract     = {On the way to fully autonomously driving vehicles a multitude of challenges have to be overcome. One common problem is the navigation of the vehicle from a start pose to a goal pose in an environment that does not provide any specic structure (no preferred ways of movement). Typical examples of such environments are parking lots or construction sites; in these scenarios the vehicle needs to navigate safely around obstacles ideally using the optimal (with regard to a specied parameter) path between the start and the goal pose. The work conducted throughout this master's thesis focuses on the development of a suitable path planning algorithm for the Research Concept Vehicle (RCV) of the Integrated Transport Research Lab (ITRL) at KTH Royal Institute of Technology, in Stockholm, Sweden. The development of the path planner requires more than just the pure algorithm, as the code needs to be tested and respective results evaluated. In addition, the resulting algorithm needs to be wrapped in a way that it can be deployed easily and interfaced with di erent other systems on the research vehicle. Thus the thesis also tries to gives insights into ways of achieving realtime capabilities necessary for experimental testing as well as on how to setup a visualization environment for simulation and debugging.}
}
```

##### Cited By (not complete)

O. Angatkina, A. Alleyne, and A. Wissa, ‘Robust design and evaluation of a novel modular origami-enabled mobile robot (OSCAR)’, Journal of Mechanisms and Robotics, 2022

T. Miao, E. El Amam, P. Slaets, and D. Pissoort, ‘An improved real-time collision-avoidance algorithm based on Hybrid A* in a multi-object-encountering scenario for autonomous surface vessels’, Ocean Engineering, 2022

O. Angatkina, ‘Design and control of an origami-enabled soft crawling autonomous robot (OSCAR)’, University of Illinois at Urbana-Champaign, 2021

Y. Chung and Y.-P. Yang, ‘Hardware-in-the-Loop Simulation of Self-Driving Electric Vehicles by Dynamic Path Planning and Model Predictive Control’, Electronics, 2021

T. Guan, Z. He, D. Manocha, and L. Zhang, ‘TTM: Terrain traversability mapping for autonomous excavator navigation in unstructured environments’, arXiv, 2021

G. Huang, L. Yang, Y. Cai, and D. Zhang, ‘Terrain classification-based rover traverse planner with kinematic constraints for Mars exploration’, Planetary and Space Science, 2021

T.-W. Kang, J.-G. Kang, and J.-W. Jung, ‘A Bidirectional Interpolation Method for Post-Processing in Sampling-Based Robot Path Planning’, Sensors, 2021

B. Maity, ‘Chauffeur: Benchmark Suite for Design and End-to-End Analysis of Self-Driving Vehicles on Embedded Systems’, ACM Transactions on Embedded Computing Systems (TECS), 2021

J. P. Moura, ‘Investigação do desempenho do planejador de trajetórias Motion Planning Networks’, 2021

X. Shi, J. Zhang, C. Liu, H. Chi, and K. Chen, ‘State estimation and reconstruction of non-cooperative targets based on kinematic model and direct visual odometry’, 2021 IEEE 11th Annual International Conference on CYBER Technology in Automation, Control, and Intelligent Systems (CYBER), 2021

B. Tang, K. Hirota, X. Wu, Y. Dai, and Z. Jia, ‘Path Planning Based on Improved Hybrid A* Algorithm’, Journal of Advanced Computational Intelligence and Intelligent Informatics, 2021

S. Zhang, Z. Jian, X. Deng, S. Chen, Z. Nan, and N. Zheng, ‘Hierarchical Motion Planning for Autonomous Driving in Large-Scale Complex Scenarios’, IEEE Transactions on Intelligent Transportation Systems, 2021

Z. Zhang, Y. Wan, Y. Wang, X. Guan, W. Ren, and G. Li, ‘Improved hybrid A* path planning method for spherical mobile robot based on pendulum’, International Journal of Advanced Robotic Systems, 2021

Z. Zhang, R. Wu, Y. Pan, Y. Wang, and G. Li, ‘Initial pose estimation and update during robot path planning loop’, 2021 China Automation Congress (CAC), 2021

J. Zhao, Z. Zhang, Z. Xue, and L. Li, ‘A Hierarchical Vehicle Motion Planning Method For Cruise In Parking Area’, 2021 5th CAA International Conference on Vehicular Control and Intelligence (CVCI), 2021

S. Arshad, M. Sualeh, D. Kim, D. Van Nam, and G.-W. Kim, ‘Clothoid: an integrated hierarchical framework for autonomous driving in a dynamic urban environment’, Sensors, 2020

S. Bø, ‘Motion planning for terrain vehicles: Path generation with radial-constrained A* and trajectory optimization’, NTNU, 2020

S. Koziol, ‘Multi-Objective Path Planning for Autonomous Robots Using Reconfigurable Analog VLSI’, IEEE Access, 2020

J. Krook, R. Kianfar, and M. Fabian, ‘Formal Synthesis of Safe Stop Tactical Planners for an Automated Vehicle’, IFAC-PapersOnLine, 2020

S. Luo, X. Li, and Z. Sun, ‘An optimization-based motion planning method for autonomous driving vehicle’, 2020 3rd International Conference on Unmanned Systems (ICUS), 2020

K. Narula, S. Worrall, and E. Nebot, ‘Two-level hierarchical planning in a known semi-structured environment’, 2020 IEEE 23rd International Conference on Intelligent Transportation Systems (ITSC), 2020

N. D. Van, M. Sualeh, D. Kim, and G.-W. Kim, ‘A hierarchical control system for autonomous driving towards urban challenges’, Applied Sciences, 2020

N. Van Dinh, Y.-G. Ha, and G.-W. Kim, ‘A Universal Control System for Self-Driving Car Towards Urban Challenges’, 2020 IEEE International Conference on Big Data and Smart Computing (BigComp), 2020

P.-J. Wang, Lidar A*, ‘An Online Visibility-Based Decomposition and Search Approach for Real-Time Autonomous Vehicle Motion Planning’ University of California, Santa Cruz, 2020

Z. Zhao and L. Bi, ‘A new challenge: Path planning for autonomous truck of open-pit mines in the last transport section’, Applied Sciences, 2020

J. Krook, L. Svensson, Y. Li, L. Feng, and M. Fabian, ‘Design and formal verification of a safe stop supervisor for an automated vehicle’, 2019 International Conference on Robotics and Automation (ICRA), 2019

D. Nemec, M. Gregor, E. Bubenikova, M. Hruboš, and R. Pirník, ‘Improving the Hybrid A* method for a non-holonomic wheeled robot’, International Journal of Advanced Robotic Systems, 2019

S. Zhang, Y. Chen, S. Chen, and N. Zheng, ‘Hybrid A*-based curvature continuous path planning in complex dynamic environments’, 2019 IEEE Intelligent Transportation Systems Conference (ITSC), 2019

Z. Wang, ‘Trajectory Planning for Four WheelSteering Autonomous Vehicle’. 2018
