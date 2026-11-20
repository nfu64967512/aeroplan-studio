# 檢索:UAV formation rendezvous assembly

來源 OpenAlex / arXiv / Semantic Scholar;2015 年以後;共 1 篇
⚠️ 無回應/未啟用的來源:arXiv, Semantic Scholar(Semantic Scholar 需設 S2_API_KEY,否則必定 429)

| # | 年 | 被引 | 標題 | 出處 |
|---|---|---|---|---|
| 1 | 2023 | 0 | [Consensus-Based Multi-UAV Partition Rendezvous in Formation](https://doi.org/10.1109/ricai60863.2023.10489593) |  |

---

### 1. Consensus-Based Multi-UAV Partition Rendezvous in Formation
*2023 ·  · 被引 0 · OpenAlex*

A multi-UAV zonal assembly control algorithm based on consistency theory is proposed to safely and quickly assemble UAV swarms into a specified formation within a certain range area. The near-staging area is divided into several partitions, and UAVs in different zones are queued to travel in a straight line toward the assigned target staging point. Through the implementation of formation assembly within sub-areas, this paper successfully resolves the trade-off between the assembly time and the control expenditure. Considering the position difference, collision factors and yaw angle between adjacent drones, the position controller and yaw controller are designed respectively. The stability of the proposed formation controller is verified using Lyapunov theory. Finally, numerical simulation verifies the effectiveness of the controller.

<https://doi.org/10.1109/ricai60863.2023.10489593>
# 檢索:cooperative rendezvous time coordination UAV

來源 OpenAlex / arXiv / Semantic Scholar;2015 年以後;共 4 篇
⚠️ 無回應/未啟用的來源:arXiv, Semantic Scholar(Semantic Scholar 需設 S2_API_KEY,否則必定 429)

| # | 年 | 被引 | 標題 | 出處 |
|---|---|---|---|---|
| 1 | 2019 | 6 | [A strategy of multi-UAV cooperative path planning based on CCPSO](https://doi.org/10.1109/icus48101.2019.8995982) | 2019 IEEE International Conference on  |
| 2 | 2025 | 1 | [How to Coordinate UAVs and UGVs for Efficient Mission Planning? Optimizing Energy-Constrained C](https://doi.org/10.15607/rss.2025.xxi.101) |  |
| 3 | 2026 | 0 | [Weather-Aware Asynchronous Vehicle–UAV Cooperative Scheduling for Distribution Network Inspecti](https://doi.org/10.3390/technologies14060355) | Technologies |
| 4 | 2026 | 0 | [An Experience-Guided MAPPO Framework for Multi-UAV Cooperative Tracking in Continuous Action Sp](https://doi.org/10.3390/drones10080583) | Drones |

---

### 1. A strategy of multi-UAV cooperative path planning based on CCPSO
*2019 · 2019 IEEE International Conference on Unmanned Systems (ICUS) · 被引 6 · OpenAlex*

Multi-UAV cooperative formation flying has significant advantages in broadening the scope of UAV operations and improving combat effectiveness. How to realize the cooperative path planning of multi-UAV is an open issue. In order to solve the search performance degradation as dimension increases, a strategy of multi-UAV cooperative path planning based on Cooperative Coevolution Particle Swarm Optimization (CCPSO) is proposed in this paper. Firstly, the initial Dubins path is generated for each UAV in the formation, of which the longest one is selected as the reference path. Intermediate waypoints are designed to modify the initial paths to avoid collisions with obstacles. Then, CCPSO algorithm is applied to optimize path parameters and adjust the path length to be equal to the reference path to ensure time coordination. Collision avoidance between UAVs is achieved in the collaborative evaluation phase. Finally, an formation rendezvous example and comparison with CCGA are given to explain the validity and excellent search ability of the proposed method.

<https://doi.org/10.1109/icus48101.2019.8995982>

### 2. How to Coordinate UAVs and UGVs for Efficient Mission Planning? Optimizing Energy-Constrained Cooperative Routing with a DRL Framework
*2025 ·  · 被引 1 · OpenAlex · OA*

Efficient mission planning for cooperative systems involving Unmanned Aerial Vehicles (UAVs) and Unmanned Ground Vehicles (UGVs) requires addressing energy constraints, scalability, and coordination challenges between agents.UAVs excel in rapidly covering large areas but are constrained by limited battery life, while UGVs, with their extended operational range and capability to serve as mobile recharging stations, are hindered by slower speeds.This heterogeneity makes coordination between UAVs and UGVs critical for achieving optimal mission outcomes.In this work, we propose a scalable deep reinforcement learning (DRL) framework to address the energyconstrained cooperative routing problem for multi-agent UAV-UGV teams, aiming to visit a set of task points in minimal time with UAVs relying on UGVs for recharging during the mission.The framework incorporates sortie-wise agent switching to efficiently manage multiple agents, by allocating task points and coordinating actions.Using an encoder-decoder transformer architecture, it optimizes routes and recharging rendezvous for the UAV-UGV team in the task scenario.Extensive computational experiments demonstrate the framework's superior performance over heuristic methods and a DRL baseline, delivering significant improvements in solution quality and runtime efficiency across diverse scenarios.Generalization studies validate its robustn…

<https://doi.org/10.15607/rss.2025.xxi.101>

### 3. Weather-Aware Asynchronous Vehicle–UAV Cooperative Scheduling for Distribution Network Inspection via Bi-Level MODDPG–NSGA-II Optimization
*2026 · Technologies · 被引 0 · OpenAlex · OA*

Extreme weather conditions impose significant challenges on distribution network inspection because UAV flight safety, energy consumption, vehicle mobility, and task coverage are strongly coupled under wind disturbances. To improve inspection efficiency and operational robustness, this paper proposes a weather-aware asynchronous vehicle–UAV cooperative scheduling method based on bi-level MODDPG–NSGA-II optimization. First, a dynamic wind field model and a wind-sensitive UAV energy model are established to describe the effects of background wind, vertical wind shear, and local gust disturbances on UAV motion and state-of-charge evolution. Then, an asynchronous vehicle–UAV collaboration mechanism is developed, allowing the vehicle to move toward downstream parking sites after UAV deployment while UAVs perform inspection and cross-site recovery under rendezvous and energy safety constraints. On this basis, a bi-level optimization framework is constructed, in which NSGA-II searches global coordination parameters and MODDPG learns adaptive multi-UAV scheduling policies in continuous decision spaces. Controlled wind-factor experiments show that, with the task scale fixed at 52 inspection tasks, the proposed method maintains 100% task coverage under 0–10 m/s wind conditions. As the reference wind speed increases from 0 m/s to 10 m/s, the mission completion time increases from 40.97 mi…

<https://doi.org/10.3390/technologies14060355>

### 4. An Experience-Guided MAPPO Framework for Multi-UAV Cooperative Tracking in Continuous Action Spaces
*2026 · Drones · 被引 0 · OpenAlex · OA*

A cooperative guidance law based on the experience-guided multi-agent proximal policy optimization (E-MAPPO) algorithm is proposed for multiple unmanned aerial vehicles (UAVs) to track dynamic points of interest in civilian applications, such as collaborative search and rescue and environmental monitoring. In multi-UAV cooperative tracking, accurate arrival-time coordination is important for improving collaborative task execution, but it remains challenging because of continuous action spaces, target maneuvering, uncertain time-to-go estimation, and inefficient exploration in multi-agent reinforcement learning. Specifically, a multi-UAV cooperative guidance environment is formulated, and the problem is modeled as a Markov decision process. To address the challenges of large action spaces and poor convergence in multi-agent reinforcement learning, an experience-guided MAPPO framework is introduced to enhance training efficiency and policy stability. Different from standard MAPPO, the proposed E-MAPPO introduces proportional-navigation-guided experience only during the early training stage to guide exploration, while the final policy is still optimized through the MAPPO objective. Subsequently, a composite reward function is designed by integrating distance-based heuristic terms with auxiliary guidance signals, thereby improving exploration efficiency and facilitating coordinated…

<https://doi.org/10.3390/drones10080583>
