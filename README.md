### Hybrid A* Path Planner for the KTH Research Concept Vehicle [![Build Status](https://travis-ci.org/karlkurzer/path_planner.svg?branch=master)](https://travis-ci.org/karlkurzer/path_planner)

* [Characteristics](#characteristics)
* [Videos](#videos)
* [Images](#images)
* [Dependencies](#dependencies)
* [Setup](#setup)
* [Visualization](#visualization)
* [Citation](#citation)

The code in this repository is the result of my master's thesis which I have written at the Integrated Research Lab (ITRL) at KTH Royal Institute of Technology (2016).
The code is documented [here](http://karlkurzer.github.io/path_planner) and the associated thesis can be found [here](http://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-198534).


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
<img src="http://i.imgur.com/OICPCTB.png" alt="Reversing in a Maze" width="600"/>
<img src="http://i.imgur.com/ZiV9GDW.png" alt="Parking" width="600"/>
<img src="http://i.imgur.com/z7aT6lt.png" alt="Mitigating a U-shape Obstacle" width="600"/>

#### <a name="dependencies"></a>Dependencies
* [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
* [ros_map_server](http://wiki.ros.org/map_server)

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

```
@mastersthesis{Kurzer1057261,
   author = {Kurzer, Karl},
   institution = {KTH, Integrated Transport Research Lab, ITRL},
   pages = {63},
   school = {KTH, Integrated Transport Research Lab, ITRL},
   title = {Path Planning in Unstructured Environments : A Real-time Hybrid A* Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle},
   series = {TRITA-AVE},
   ISSN = {1651-7660},
   number = {2016:41},
   abstract = {On the way to fully autonomously driving vehicles a multitude of challenges have to be overcome. One common problem is the navigation of the vehicle from a start pose to a goal pose in an environment that does not provide any specic structure (no preferred ways of movement). Typical examples of such environments are parking lots or construction sites; in these scenarios the vehicle needs to navigate safely around obstacles ideally using the optimal (with regard to a specied parameter) path between the start and the goal pose. The work conducted throughout this master's thesis focuses on the development of a suitable path planning algorithm for the Research Concept Vehicle (RCV) of the Integrated Transport Research Lab (ITRL) at KTH Royal Institute of Technology, in Stockholm, Sweden. The development of the path planner requires more than just the pure algorithm, as the code needs to be tested and respective results evaluated. In addition, the resulting algorithm needs to be wrapped in a way that it can be deployed easily and interfaced with di erent other systems on the research vehicle. Thus the thesis also tries to gives insights into ways of achieving realtime capabilities necessary for experimental testing as well as on how to setup a visualization environment for simulation and debugging. },
   year = {2016}
}
```
