### Hybrid A* Path Planner for the KTH Research Concept Vehicle

The code in this repository is the result of my master's thesis which I am currently writing at the Integrated Research Lab (ITRL) at KTH Royal Institute of Technology.
The code is documented [here](http://karlkurzer.github.io/path_planner) (work in progress).


The goal of the thesis and hence this code is to create a real-time path planning algorithm for the nonholonomic Research Concept Vehicle (RCV). The algorithm uses a binary obstacle map as an input, generated using LIDAR mounted on top of the vehicle. The algorithm is being developed using C++ due to real-time requirements in combination with ROS to ensure modularity and portability as well as using RViz as a visualization/simulation environment.

##### Key Characteristics
* Sampling in continuous space with 72 different headings per cell (5° discretization)
* Constrained Heuristic - _nonholonomic without obstacles_
* Unconstrained Heuristic - _holonomic with obstacles_
* Dubin's Shot
* C++ real-time implementation (~10 Hz)

Large parts of the implementation are closely related to the hybrid A* algorithm developed by Dmitri Dolgov and Sebastian Thrun (_Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments_ DOI: 10.1177/0278364909359210)

##### Videos
* [Path Planning with Search Visualization](https://www.youtube.com/watch?v=1WZEQtg8ZZ4)
* [Dubin's Path - Constrained Heuristic](https://www.youtube.com/watch?v=VNo9fU6XEGE)
* [2D A* Search - Unconstrained Heuristic](https://www.youtube.com/watch?v=Ip2iUrVoFXc)
* [Open Loop Path Planning using Sensor Fusion](https://www.youtube.com/watch?v=GwIU00jukO4)

##### Images
<img src="http://i.imgur.com/OICPCTB.png" alt="Reversing in a Maze" width="600"/>
<img src="http://i.imgur.com/ZiV9GDW.png" alt="Parking" width="600"/>
<img src="http://i.imgur.com/z7aT6lt.png" alt="Mitigating a U-shape Obstacle" width="600"/>

###### Copyright 2016 Karl Kurzer

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL KARL KURZER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
