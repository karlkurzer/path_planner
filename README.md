### Hybrid A* Path Planner for the KTH Research Concept Vehicle [![Build Status](https://app.travis-ci.com/karlkurzer/path_planner.svg?branch=master)](https://travis-ci.org/karlkurzer/path_planner)

This repository contains the implementation of a Hybrid A* Path Planner for autonomous vehicles, specifically developed for the KTH Research Concept Vehicle. The Hybrid A* algorithm is a powerful path planning approach that combines the benefits of A* search in continuous space with a discretized set of headings. It enables the generation of efficient and smooth paths for nonholonomic vehicles navigating complex environments.

#### Table of Contents
* [Introduction](#introduction)
* [Characteristics](#characteristics)
* [Videos](#videos)
* [Images](#images)
* [Dependencies](#dependencies)
* [Setup](#setup)
* [Visualization](#visualization)
* [Citation](#citation)

#### Introduction
The code in this repository is the result of my master's thesis which I have written at the Integrated Research Lab (ITRL) at KTH Royal Institute of Technology (2016).
The code is documented [here](https://karlkurzer.github.io/path_planner) and the associated thesis can be found [here](https://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-198534).


The goal of the thesis and hence this code is to create a real-time path planning algorithm for the nonholonomic Research Concept Vehicle (RCV). The algorithm uses a binary obstacle map as an input, generated using LIDAR mounted on top of the vehicle. The algorithm is being developed using C++ due to real-time requirements in combination with ROS to ensure modularity and portability as well as using RViz as a visualization/simulation environment.

#### <a name="characteristics"></a>Key Characteristics
* Sampling in continuous space with 72 different headings per cell (5° discretization)
* Constrained Heuristic - _nonholonomic without obstacles_
* Unconstrained Heuristic - _holonomic with obstacles_
* Dubin's Shot
* C++ real-time implementation (~10 Hz)

Large parts of the implementation are closely related to the hybrid A* algorithm developed by Dmitri Dolgov and Sebastian Thrun (_Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments_ DOI: 10.1177/0278364909359210)

#### <a name="videos"></a>Videos
* [Path Planning with Search Visualization](https://www.youtube.com/watch?v=1WZEQtg8ZZ4)
* [Dubin's Path - Constrained Heuristic](https://www.youtube.com/watch?v=VNo9fU6XEGE)
* [2D A* Search - Unconstrained Heuristic](https://www.youtube.com/watch?v=Ip2iUrVoFXc)
* [Open Loop Path Planning using Sensor Fusion](https://www.youtube.com/watch?v=GwIU00jukO4)

#### <a name="images"></a>Images
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
- N. Adiuku, N. P. Avdelidis, G. Tang, and A. Plastropoulos, ‘Advancements in Learning-Based Navigation Systems for Robotic Applications in MRO Hangar’, Sensors, 2024.
- H. Chi, P. Cai, D. Fu, J. Zhai, Y. Zeng, and B. Shi, ‘Spatiotemporal-restricted A∗ algorithm as a support for lane-free traffic at intersections with mixed flows’, Green Energy and Intelligent Transportation, 2024.
- R. Han et al., ‘NeuPAN: Direct Point Robot Navigation with End-to-End Model-based Learning’, arXiv preprint arXiv:2403. 06828, 2024.
- O. Maass and T. Vallgren, ‘Multi-Agent Trajectory Planning for Nonholonomic UAVs’. 2024.
- A. Prabu, ‘Integrating Data-driven Control Methods with Motion Planning: A Deep Reinforcement Learning-based Approach’, Purdue University Graduate School, 2024.
- Z. Wu, B. Zhao, X. Han, and D. Lu, ‘Berthing Trajectory Tracking of Underactuated Surface Vehicle Based on NMPC and Position Estimation’, IEEE Access, 2024.
- T. Xia and H. Chen, ‘A Survey of Autonomous Vehicle Behaviors: Trajectory Planning Algorithms, Sensed Collision Risks, and User Expectations’, Sensors, 2024.
- W. Yu et al., ‘Ldp: A local diffusion planner for efficient robot navigation and collision avoidance’, arXiv preprint arXiv:2407. 01950, 2024.
- C. Zhang, X. Wu, J. Wang, and M. Song, ‘Efficient Uncertainty-Aware Collision Avoidance for Autonomous Driving Using Convolutions’, IEEE Transactions on Intelligent Transportation Systems, 2024.
- Z. M. Al-Zubaidi, A. Y. Serdar, and A.-K. Mohanned, ‘A comparative study of various path planning algorithms for pick-and-place robots’, 2023.
- O. Angatkina, A. G. Alleyne, and A. Wissa, ‘Robust design and evaluation of a novel modular origami-enabled mobile robot (oscar)’, Journal of Mechanisms and Robotics, 2023.
- N. Boros, G. Kallós, and Á. Ballagi, ‘Implementation of Trajectory Planning Algorithms for Track Serving Mobile Robot in ROS 2 Ecosystem’, Tehnički vjesnik, 2023.
- X. Cai, Z. Xue, Z. Chen, and Y. Liu, ‘An Improved Path Planning Algorithm for USV Based on Voxel Extension’, in 2023 9th International Conference on Computer and Communications (ICCC), 2023.
- T. Guan, Z. He, R. Song, and L. Zhang, ‘TNES: terrain traversability mapping, navigation and excavation system for autonomous excavators on worksite’, Autonomous Robots, 2023.
- L. Han, L. He, X. Sun, Z. Li, and Y. Zhang, ‘An enhanced adaptive 3D path planning algorithm for mobile robots with obstacle buffering and improved Theta* using minimum snap trajectory smoothing’, Journal of King Saud University-Computer and Information Sciences, 2023.
- D. Li and K. Li, ‘Monovision End-to-End Dual-Lane Overtaking Network without Map Assistance’, Applied Sciences, 2023.
- J. Li et al., ‘Improved A-Star Path Planning Algorithm in Obstacle Avoidance for the Fixed-Wing Aircraft. Electronics 2023, 12, 5047. htps’, doi.  org/10. 3390/electronics12245047 Academic Editors: Carlos Tavares Calafate and Christos J.  Bouras Received, 2023.
- B. Maity, ‘Self-aware Memory Management for Emerging Architectures’, UC Irvine, 2023.
- C. Nantabut and D. Abel, ‘Weighted Hybrid A*-Based Trajectory Planning for Dynamic Environments’, SN Computer Science, 2023.
- C. Nantabut, ‘A*-based trajectory planning in dynamic environments for autonomous vehicles’, Universitätsbibliothek der RWTH Aachen, 2023.
- H. Sheikhi Darani, ‘Developing a planning and a perception module for an autonomous warehouse mobile robot’, 2023.
- H. Sui et al., ‘Multi-UAV Cooperative and Continuous Path Planning for High-Resolution 3D Scene Reconstruction’, Drones, 2023.
- Q. Yang et al., ‘Decoupled real-time trajectory planning for multiple autonomous mining trucks in unloading areas’, IEEE Transactions on Intelligent Vehicles, 2023.
- C. Zhang, Z. Li, J. Wang, and M. Song, ‘A convolution-based motion planning method for autonomous driving with localization uncertainty’, IFAC-PapersOnLine, 2023.
- I. Chichkanov and M. Shawin, ‘Algorithm for Finding the Optimal Obstacle Avoidance Maneuver for Wheeled Robot Moving Along Trajectory’, in 2022 16th International Conference on Stability and Oscillations of Nonlinear Control Systems (Pyatnitskiy’s Conference), 2022.
- T. D’Amico, ‘Developing an autonomous haulage system testbed for 5G mobile connectivity evaluation’, University of British Columbia, 2022.
- J. Huang, Z. Liu, X. Chi, F. Hong, and H. Su, ‘Search-Based Path Planning Algorithm for Autonomous Parking: Multi-Heuristic Hybrid A’, in 2022 34th Chinese Control and Decision Conference (CCDC), 2022.
- T. Miao, E. El Amam, P. Slaets, and D. Pissoort, ‘An improved real-time collision-avoidance algorithm based on Hybrid A* in a multi-object-encountering scenario for autonomous surface vessels’, Ocean Engineering, 2022.
- C. Nantabut and D. Abel, ‘A Robust Modified Hybrid A*-based Closed-loop Local Trajectory Planner for Complex Dynamic Environments’, in VEHITS, 2022.
- J. Pang, S. Zhang, J. Fu, J. Liu, and N. Zheng, ‘Curvature continuous path planning with reverse searching for efficient and precise autonomous parking’, in 2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC), 2022.
- M. Parseh, ‘Pre-crash Motion Planning for Autonomous Vehicles in Unavoidable Collision Scenarios’, KTH Royal Institute of Technology, 2022.
- X. Tang, J. Liao, L. Liu, Y. Yan, Y. Xiao, and L. Wang, ‘Towards Inspection and Cleaning on Capacitor Towers: Design and Implementation of Multi-task Operation Robot’, in Journal of Physics: Conference Series, 2022.
- C. Zhang, M. Song, and J. Wang, ‘A convolution-based grid map reconfiguration method for autonomous driving in highly constrained environments’, in 2022 IEEE Intelligent Vehicles Symposium (IV), 2022.
- Y. Zhao, Y. Zhu, P. Zhang, Q. Gao, and X. Han, ‘A hybrid A* path planning algorithm based on multi-objective constraints’, in 2022 Asia Conference on Advanced Robotics, Automation, and Control Engineering (ARACE), 2022.
- H. Zhou, X. Zhang, and X. Peng, ‘A Robust and Efficient Path Planning System Based on the Global Euclidean Distance Fields’, in 2022 IEEE Conference on Telecommunications, Optics and Computer Science (TOCS), 2022.
- O. Angatkina, ‘Design and control of an origami-enabled soft crawling autonomous robot (OSCAR)’, University of Illinois at Urbana-Champaign, 2021.
- Y. Chung and Y.-P. Yang, ‘Hardware-in-the-loop simulation of self-driving electric vehicles by dynamic path planning and model predictive control’, Electronics, 2021.
- T. Guan, Z. He, D. Manocha, and L. Zhang, ‘Ttm: Terrain traversability mapping for autonomous excavator navigation in unstructured environments’, arXiv preprint arXiv:2109. 06250, 2021.
- T. Guan, Z. He, R. Song, D. Manocha, and L. Zhang, ‘Tns: Terrain traversability mapping and navigation system for autonomous excavators’, arXiv preprint arXiv:2109. 06250, 2021.
- G. Huang, L. Yang, Y. Cai, and D. Zhang, ‘Terrain classification-based rover traverse planner with kinematic constraints for Mars exploration’, Planetary and Space Science, 2021.
- T.-W. Kang, J.-G. Kang, and J.-W. Jung, ‘A bidirectional interpolation method for post-processing in sampling-based robot path planning’, Sensors, 2021.
- B. Maity et al., ‘Chauffeur: Benchmark suite for design and end-to-end analysis of self-driving vehicles on embedded systems’, ACM Transactions on Embedded Computing Systems (TECS), 2021.
- J. P. Moura and Others, ‘Investigação do desempenho do planejador de trajetórias Motion Planning Networks’, 2021.
- X. Shi, J. Zhang, C. Liu, H. Chi, and K. Chen, ‘State estimation and reconstruction of non-cooperative targets based on kinematic model and direct visual odometry’, in 2021 IEEE 11th Annual International Conference on CYBER Technology in Automation, Control, and Intelligent Systems (CYBER), 2021.
- B. Tang, K. Hirota, X. Wu, Y. Dai, and Z. Jia, ‘Path planning based on improved hybrid A* algorithm’, Journal of Advanced Computational Intelligence and Intelligent Informatics, 2021.
- L. Wang, Z. Ye, and L. Zhang, ‘Hierarchical planning for autonomous excavator on material loading tasks’, in ISARC. Proceedings of the International Symposium on Automation and Robotics in Construction, 2021.
- B. Zhang and D. Zhu, ‘A new method on motion planning for mobile robots using jump point search and Bezier curves’, International Journal of Advanced Robotic Systems, 2021.
- S. Zhang, Z. Jian, X. Deng, S. Chen, Z. Nan, and N. Zheng, ‘Hierarchical motion planning for autonomous driving in large-scale complex scenarios’, IEEE Transactions on Intelligent Transportation Systems, 2021.
- Z. Zhang, Y. Wan, Y. Wang, X. Guan, W. Ren, and G. Li, ‘Improved hybrid A* path planning method for spherical mobile robot based on pendulum’, International Journal of Advanced Robotic Systems, 2021.
- Z. Zhang, R. Wu, Y. Pan, Y. Wang, and G. Li, ‘Initial pose estimation and update during robot path planning loop’, in 2021 China Automation Congress (CAC), 2021.
- J. Zhao, Z. Zhang, Z. Xue, and L. Li, ‘A Hierarchical Vehicle Motion Planning Method For Cruise In Parking Area’, in 2021 5th CAA International Conference on Vehicular Control and Intelligence (CVCI), 2021.
- S. Arshad, M. Sualeh, D. Kim, D. Van Nam, and G.-W. Kim, ‘Clothoid: an integrated hierarchical framework for autonomous driving in a dynamic urban environment’, Sensors, 2020.
- S. Bø, ‘Motion planning for terrain vehicles: Path generation with radial-constrained A* and trajectory optimization’, NTNU, 2020.
- H. Esteban Cabezos, ‘Optimization of the Parking Manoeuvre for a 1-Trailer Truck’. 2020.
- S. Koziol, ‘Multi-Objective path planning for autonomous robots using reconfigurable analog VLSI’, IEEE Access, 2020.
- J. Krook, R. Kianfar, and M. Fabian, ‘Formal synthesis of safe stop tactical planners for an automated vehicle’, IFAC-PapersOnLine, 2020.
- S. Luo, X. Li, and Z. Sun, ‘An optimization-based motion planning method for autonomous driving vehicle’, in 2020 3rd international conference on unmanned systems (ICUS), 2020.
- K. Narula, S. Worrall, and E. Nebot, ‘Two-level hierarchical planning in a known semi-structured environment’, in 2020 IEEE 23rd International Conference on Intelligent Transportation Systems (ITSC), 2020.
- N. Van Dinh, Y.-G. Ha, and G.-W. Kim, ‘A universal control system for self-driving car towards urban challenges’, in 2020 IEEE International Conference on Big Data and Smart Computing (BigComp), 2020.
- N. D. Van, M. Sualeh, D. Kim, and G.-W. Kim, ‘A hierarchical control system for autonomous driving towards urban challenges’, Applied Sciences, 2020.
- P.-J. Wang, Lidar A*, an Online Visibility-Based Decomposition and Search Approach for Real-Time Autonomous Vehicle Motion Planning. University of California, Santa Cruz, 2020.
- J. Krook, L. Svensson, Y. Li, L. Feng, and M. Fabian, ‘Design and formal verification of a safe stop supervisor for an automated vehicle’, in 2019 International Conference on Robotics and Automation (ICRA), 2019.
- D. Nemec, M. Gregor, E. Bubeníková, M. Hruboš, and R. Pirnik, ‘Improving the Hybrid A* method for a non-holonomic wheeled robot’, International Journal of Advanced Robotic Systems, 2019.
- S. Zhang, Y. Chen, S. Chen, and N. Zheng, ‘Hybrid A-based curvature continuous path planning in complex dynamic environments’, in 2019 IEEE Intelligent Transportation Systems Conference (ITSC), 2019.
- Z. Wang, ‘Trajectory Planning for Four WheelSteering Autonomous Vehicle’. 2018.
