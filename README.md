# CP230_HWS

**Course Description:** Grid search-based planning (DFS, BFS, Dijkstra, A* algorithm), Criteria for comparing algorithms- Time and space complexity, Completeness and Optimality; Forward and inverse kinematics, obstacle representations; planning complexity, Various heuristics for A* algorithm; Markov Decision Process, Value and Policy Iteration Algorithms, Sampling-based planning (RRT & RRT algorithms); Path planning using RRT; Implementation of continuous space sampling: Real world examples using A* and RRT* Motion planning in discrete space; Logic-based planning methods; Geometric representations; Kinematic chains and rigid and non-rigid transformations; Configuration space; Topological space concepts; Representation of obstacles; Collision detection and avoidance in relative velocity space; Collision cones and velocity obstacles; Artificial potential fields; Flocking and swarming, Formation control

References:
  - S.M. LaValle, Planning Algorithms, Cambridge University Press, 2006
  - M. Mesbahi and M. Egerstedt, Graph Theoretic Methods in Multiagent Networks, Princeton Series in Applied Mathematics, 2010
  - J. C. Latombe, Robot Motion Planning (Vol. 124). Springer Science & Business Media, 2012
  - Artificial Intelligence: A Modern Approach Textbook by Peter Norvig and Stuart J. Russell

---

## Final Term Project

**Title:** Reducing Onboard Processing Time for Path Planning in Dynamically Evolving Polygonal Maps

### Abstract
Autonomous agents face the challenge of coordinating multiple tasks (perception, motion planning, controller) which are computationally expensive on a single onboard computer. To utilize the onboard processing capacity optimally, it is imperative to arrive at computationally efficient algorithms for global path planning. In this work, it is attempted to reduce the processing time for global path planning in dynamically evolving polygonal maps. In dynamic environments, maps may not remain valid for long. Hence it is of utmost importance to obtain the shortest path quickly in an ever-changing environment. To address this, an existing rapid path-finding algorithm, the Minimal Construct was used. This algorithm discovers only a necessary portion of the Visibility Graph around obstacles and computes collision tests only for lines that seem heuristically promising. Simulations show that this algorithm finds shortest paths faster than traditional grid-based A* searches in most cases, resulting in smoother and shorter paths even in dynamic environments.

### Results

**Static Scenarios**

Blue: Grid based A* & Red: Minimal Construct
| Case 1 | Case 2 |
| :----: | :----: |
| <img src="./Minimal%20Construct/results/intro.png" width=300 height=300> | <img src="./Minimal%20Construct/results/results1.png" width=300 height=300> |

**Dynamic Scenarios**

|  Global Planner   | Case 1 | Case 2 | Case 3 | Case 4 |
| :---------------: | :----: | :----: | :----: | :----: |
|   Grid based A*   | <img src="./Minimal%20Construct/results/image19.gif" width=250> | <img src="./Minimal%20Construct/results/image21.gif" width=250> | <img src="./Minimal%20Construct/results/image23.gif" width=250> | <img src="./Minimal%20Construct/results/image26.gif" width=250> |
| Minimal Construct | <img src="./Minimal%20Construct/results/image20.gif" width=250> | <img src="./Minimal%20Construct/results/image22.gif" width=250> | <img src="./Minimal%20Construct/results/image24.gif" width=250> | <img src="./Minimal%20Construct/results/image25.gif" width=250> |

<!-- |                       Grid based A*                             |                         Minimal Construct                       |
| --------------------------------------------------------------- | --------------------------------------------------------------- |
| <img src="./Minimal%20Construct/results/image19.gif" width=250> | <img src="./Minimal%20Construct/results/image20.gif" width=250> |
| <img src="./Minimal%20Construct/results/image21.gif" width=250> | <img src="./Minimal%20Construct/results/image21.gif" width=250> |
| <img src="./Minimal%20Construct/results/image22.gif" width=250> | <img src="./Minimal%20Construct/results/image23.gif" width=250> |
| <img src="./Minimal%20Construct/results/image24.gif" width=250> | <img src="./Minimal%20Construct/results/image25.gif" width=250> | -->

> Reference: [M. Missura, D. D. Lee and M. Bennewitz, "Minimal Construct: Efficient Shortest Path Finding for Mobile Robots in Polygonal Maps," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 7918-7923, doi: 10.1109/IROS.2018.8594124.](https://doi.org/10.1109/IROS.2018.8594124)

# Citation

If you find this project useful, please cite us 🤗

```
@article{shirwatkar2023reducing,
  title={Reducing Onboard Processing Time for Path Planning in Dynamically Evolving Polygonal Maps},
  author={Shirwatkar, Aditya and Singh, Aman and Kiran, Jana Ravi},
  journal={arXiv e-prints},
  pages={arXiv--2305},
  year={2023}
}
```
